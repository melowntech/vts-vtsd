/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <future>
#include <mutex>
#include <thread>
#include <algorithm>

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/chrono/duration.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "utility/rlimit.hpp"

#include "vts-libs/storage/error.hpp"
#include "vts-libs/storage/io.hpp"

#include "./cache.hpp"
#include "./vts/driver.hpp"
#include "./tileset/driver.hpp"
#include "./vts0/driver.hpp"

namespace asio = boost::asio;
namespace bs = boost::system;
namespace vts = vtslibs::vts;

namespace {

/** Time between flushes.
 */
// constexpr std::time_t CHECK_INTERVAL(60);
constexpr std::time_t CHECK_INTERVAL(5);

/** Maximal time between hits in cache for single record.
 */
constexpr std::time_t MAX_INTERVAL_BETWEEN_HITS(600);

struct Record {
    Record(const std::string &path)
        : path(path), lastHit(std::time(nullptr))
    {}

    ~Record() {}

    void update(std::time_t now) {
        lastHit = now;
    }

    /** Is the driver ready?
     */
    bool ready() const { return driver.get(); }

    /** Is this record invalid?
     */
    bool invalid() const { return !driver && openCallbacks.empty(); }

    // path to dataset
    std::string path;
    // pointer to driver
    DeliveryCache::Driver driver;

    // callbacks waiting for driver to be opened
    DeliveryCache::CallbackList openCallbacks;

    // time of last cache hit
    std::time_t lastHit;
};

typedef std::map<utility::FileId, Record> Drivers;

} // namespace

class DeliveryCache::Detail : boost::noncopyable {
public:
    Detail(DeliveryCache &cache, unsigned int threadCount
           , const vts::OpenOptions &openOptions)
        : cache_(cache), openOptions_(openOptions)
        , maintenanceTimer_(ios_)
    {
        cleanupLimit_.openFiles = utility::maxOpenFiles() / 2;
        cleanupLimit_.memory
            = std::numeric_limits<decltype(cleanupLimit_.memory)>::max();
        LOG(info3) << "Cleanup limits: " << cleanupLimit_ << ".";

        start(threadCount);
    }

    ~Detail() { stop(); }

    Driver get(const std::string &path);

    void get(const std::string &path, const Callback &callback);

private:
    void open(Record &record);
    void finishOpen(Record &record, const Expected &value);

    void get(std::unique_lock<std::mutex> &lock
             , const std::string &path, const utility::FileId &fid
             , Drivers::iterator idrivers
             , const Callback &callback);

    void start(std::size_t count);
    void stop();
    void worker(std::size_t id);
    void startMaintenance();

    void check();

    DeliveryCache &cache_;
    const vts::OpenOptions openOptions_;

    vs::Resources cleanupLimit_;

    asio::io_service ios_;

    /** Processing pool stuff.
     */
    boost::optional<asio::io_service::work> work_;
    std::vector<std::thread> workers_;
    asio::steady_timer maintenanceTimer_;

    // cache
    std::mutex mutex_;
    Drivers drivers_;
};

void DeliveryCache::Detail::start(std::size_t count)
{
    // make sure threads are released when something goes wrong
    struct Guard {
        Guard(const std::function<void()> &func) : func(func) {}
        ~Guard() { if (func) { func(); } }
        void release() { func = {}; }
        std::function<void()> func;
    } guard([this]() { stop(); });

    work_ = boost::in_place(std::ref(ios_));

    for (std::size_t id(1); id <= count; ++id) {
        workers_.emplace_back(&Detail::worker, this, id);
    }

    guard.release();

    startMaintenance();
}

void DeliveryCache::Detail::stop()
{
    LOG(info2) << "Stopping delivery cache workers.";
    work_ = boost::none;
    maintenanceTimer_.cancel();
    ios_.stop();

    while (!workers_.empty()) {
        workers_.back().join();
        workers_.pop_back();
    }

    {
        // cancel all pending opens
        std::unique_lock<std::mutex> guard(mutex_);
        drivers_.clear();
    }
}

void DeliveryCache::Detail::worker(std::size_t id)
{
    dbglog::thread_id(str(boost::format("cache:%u") % id));
    LOG(info2) << "Spawned cache worker id:" << id << ".";

    for (;;) {
        // try {
            ios_.run();
            LOG(info2) << "Terminated cache worker id:" << id << ".";
            return;
        // } catch (const std::exception &e) {
        //     LOG(err3)
        //         << "Uncaught exception in cache worker: <" << e.what()
        //         << ">. Going on.";
        // }
    }
}

void DeliveryCache::Detail::startMaintenance()
{
    maintenanceTimer_.expires_from_now
        (std::chrono::seconds(CHECK_INTERVAL));

    maintenanceTimer_.async_wait([this](const bs::error_code &ec)
    {
        if (!ec) {
            try {
                // run check
                check();
            } catch (const std::exception &e) {
                LOG(err3)
                    << "Uncaught exception in maintenance: <" << e.what()
                    << ">. Going on.";
            }

            // start maintenance again
            startMaintenance();
        };
    });
}

DeliveryCache::Driver openDriver(const std::string &path
                                 , const vts::OpenOptions &openOptions
                                 , DeliveryCache &cache
                                 , const DeliveryCache::Callback &callback)
{
    // try VTS
    try {
        return openVts(path, openOptions, cache, callback);
    } catch (vs::NoSuchTileSet) {}

    // try VTS0
    try { return openVts0(path); } catch (vs::NoSuchTileSet) {}

    // finaly, try old TS
    return openTileSet(path);
}

void DeliveryCache::Detail::finishOpen(Record &record, const Expected &value)
{
    const auto dispatch([this](CallbackList &callbacks
                               , const Expected &value)
    {
        for (const auto &callback : callbacks) {
            // dispatch in thread pool
            ios_.post([callback, value]() { callback(value); });
        }
    });

    if (value) {
        // driver open -> store in the record
        CallbackList callbacks;
        {
            std::unique_lock<std::mutex> guard(mutex_);
            // store driver and steal callbacks
            record.driver = value.get();
            std::swap(callbacks, record.openCallbacks);
        }

        // dispatch to callbacks (unlocked)
        dispatch(callbacks, value);
    } else {
        // open failed
        CallbackList callbacks;
        {
            std::unique_lock<std::mutex> guard(mutex_);

            // steal callbacks
            std::swap(callbacks, record.openCallbacks);

            // leave invalid record in the cache
        }

        // dispatch exception to interested parties
        dispatch(callbacks, value);
    }
}

void DeliveryCache::Detail::open(Record &record)
{
    // NB: using dispatch instead of post, i.e. we do not block any other thread
    ios_.dispatch([this, &record]() mutable
    {
        const auto dispatch([this](CallbackList &callbacks
                                   , const Expected &value)
        {
            for (const auto &callback : callbacks) {
                // dispatch in thread pool
                ios_.post([callback, value]() { callback(value); });
            }
        });

        try {
            // open driver
            auto driver
                (openDriver(record.path, openOptions_, cache_
                            , [this, &record](const Expected &value)
                            {
                                finishOpen(record, value);
                            }));
            if (!driver) {
                // async, ignore
                return;
            }
            finishOpen(record, driver);
        } catch (...) {
            finishOpen(record, std::current_exception());
        }
    });
}

DeliveryCache::DeliveryCache(unsigned int threadCount
                             , const vts::OpenOptions &openOptions)
    : workers_(new Detail(*this, threadCount, openOptions))
{}

DeliveryCache::~DeliveryCache() {}

void DeliveryCache::Detail
::get(std::unique_lock<std::mutex> &lock
      , const std::string &path, const utility::FileId &fid
      , Drivers::iterator idrivers
      , const Callback &callback)
{
    if (idrivers != drivers_.end()) {
        // record found
        auto &record(idrivers->second);

        if (record.ready()) {
            // valid record, grab driver
            const auto driver(record.driver);

            // unlock and call callback
            lock.unlock();
            callback(driver);
            return;
        } else if (record.invalid()) {
            // invalid record
            // unlock and call callback
            lock.unlock();
            callback(std::make_exception_ptr
                     (vs::NoSuchTileSet("(cached) No such tileset.")));
        }

        // pending open
        record.openCallbacks.push_back(callback);

        // unlock
        lock.unlock();
        return;
    }

    // create new entry and grab reference to it
    idrivers = drivers_.insert(Drivers::value_type(fid, Record(path))).first;
    idrivers->second.openCallbacks.push_back(callback);

    // call open unlocked
    lock.unlock();
    open(idrivers->second);
}

void DeliveryCache::Detail::get(const std::string &path
                                 , const Callback &callback)
{
    const auto fid(utility::FileId::from(path));

    std::unique_lock<std::mutex> guard(mutex_);
    auto idrivers(drivers_.find(fid));
    get(guard, path, fid, idrivers, callback);
}

namespace {

template <typename T>
class Promise {
public:
    Promise() = default;
    Promise(const Promise &o) : p_(std::move(o.promise())) {}

    std::promise<T>& promise() const {
        return const_cast<std::promise<T>&>(p_);
    }

private:
    std::promise<T> p_;
};

} // namespace

DeliveryCache::Driver DeliveryCache::Detail::get(const std::string &path)
{
    const auto fid(utility::FileId::from(path));
    std::unique_lock<std::mutex> guard(mutex_);
    auto idrivers(drivers_.find(fid));

    if (idrivers != drivers_.end()) {
        if (idrivers->second.ready()) {
            // already available -> shortcut
            return idrivers->second.driver;
        } else if (idrivers->second.invalid()) {
            throw vs::NoSuchTileSet("(cached) No such tileset.");
        }
    }

    // need to open or wait for a pending open
    Promise<Driver> promise;
    auto future(promise.promise().get_future());

    get(guard, path, fid, idrivers
        , [promise](const Expected &driver)
    {
        driver.get(promise.promise());
    });

    // wait without holding lock
    auto value(future.get());
    return value;
}

void DeliveryCache::get(const std::string &path
                        , const Callback &callback)
{
    workers_->get(path, callback);
}

DeliveryCache::Driver DeliveryCache::get(const std::string &path)
{
    return workers_->get(path);
}

namespace {

class Unlocker {
public:
    Unlocker(std::unique_lock<std::mutex> &lock)
        : lock_(lock) { lock_.unlock(); }
    ~Unlocker() { lock_.lock(); }
    std::unique_lock<std::mutex> &lock_;
};

struct RecordWrapper {
    typedef std::vector<RecordWrapper> list;

    Drivers::iterator idrivers;
    Record *record;
    vs::Resources resources;

    RecordWrapper(Drivers::iterator idrivers)
        : idrivers(idrivers), record(&idrivers->second)
        , resources(record->driver->resources())
    {}

    bool changed() const {
        return record->driver->externallyChanged();
    }

    bool operator<(const RecordWrapper &o) const {
        return resources < o.resources;
    }
};

} // namespace

void DeliveryCache::Detail::check()
{
    LOG(info2) << "Maintenance check.";

    // grab drivers (under lock)
    RecordWrapper::list records;

    vs::Resources resources;

    // traverse under lock
    std::unique_lock<std::mutex> guard(mutex_);

    auto erase([&](const Drivers::iterator &i) -> Drivers::iterator
    {
        LOG(info2) << "Removing driver for "
                   << i->second.path << ".";
        return drivers_.erase(i);
    });

    for (auto idrivers(drivers_.begin()), edrivers(drivers_.end());
         idrivers != edrivers; )
    {
        if (idrivers->second.invalid()) {
            // invalid -> remove
            idrivers = erase(idrivers);
            continue;
        } else if (!idrivers->second.ready()) {
            // not ready yet -> skip
            ++idrivers;
            continue;
        }

        // build record wrapper
        records.emplace_back(idrivers++);
        resources += records.back().resources;
    }

    std::vector<Drivers::iterator> toRemove;

    // analyze open drivers without lock
    {
        Unlocker unlock(guard);

        // sort by number of resources consumed
        std::sort(records.begin(), records.end());

        // process all records
        for (const auto &rw : records) {
            if (rw.changed()) {
                // dataset has been changed, plan removal
                toRemove.push_back(rw.idrivers);
                resources -= rw.resources;
                continue;
            }

            if (resources > cleanupLimit_) {
                // too much memory, remove
                toRemove.push_back(rw.idrivers);
                resources -= rw.resources;
                continue;
            }
        }
    }

    // under lock again

    // drop marked drivers
    for (const auto &idriver : toRemove) {  erase(idriver); }
}
