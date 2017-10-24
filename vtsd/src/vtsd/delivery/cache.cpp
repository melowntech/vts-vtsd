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

#include <mutex>
#include <thread>
#include <algorithm>

#include <boost/logic/tribool.hpp>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/chrono/duration.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/composite_key.hpp>

#include "utility/rlimit.hpp"
#include "utility/enum-io.hpp"
#include "utility/magic.hpp"

#include "vts-libs/storage/error.hpp"
#include "vts-libs/storage/io.hpp"

#include "../config-io.hpp"
#include "./cache.hpp"
#include "./vts/driver.hpp"
#include "./vts0/driver.hpp"
#include "./slpk/driver.hpp"

namespace asio = boost::asio;
namespace bs = boost::system;
namespace vts = vtslibs::vts;

namespace {

/** Time between flushes.
 */
constexpr std::time_t CHECK_INTERVAL(60);

struct Record {
    enum class Status {
        pending // driver is not ready yet
        , ready // driver is ready and usable
        , outdated // driver is ready but outdated, consider a reopen
        , invalid // driver was not opened due to error
     };

    Record(const std::string &path)
        : path(path), lastHit(std::time(nullptr)), hits(), serial()
    {}

    ~Record() {}

    void set(const DeliveryCache::Expected value) {
        driver = value.get();
        update();
    }

    void update(std::time_t now) {
        lastHit = now;
        ++hits;
    }

    void update() { update(std::time(nullptr)); }

    /** Driver status.
     *
     * \param checkForChange: check for underlying dataset change
     *            false never
     *            true always
     *            indeterminate only for hot content
     */
    Status status(boost::tribool checkForChange = boost::indeterminate) const {
        if (!driver) {
            // no driver, either not loaded yet or invalid
            return (openCallbacks.empty()
                    ? Status::invalid
                    : Status::pending);
        }

        if (checkForChange) {
            // always check for change
            return (driver->externallyChanged()
                    ? Status::outdated
                    : Status::ready);
        }

        if (!checkForChange) {
            // never check for change
            return Status::ready;
        }

        // check for change only if hotcontent
        return ((driver->hotContent() && driver->externallyChanged())
                ? Status::outdated
                : Status::ready);
    }

    void prepareReopen() {
        driver.reset();
        ++serial;
    }

    // path to dataset
    std::string path;
    // pointer to driver
    DeliveryCache::Driver driver;

    // callbacks waiting for driver to be opened
    DeliveryCache::CallbackList openCallbacks;

    // time of last cache hit
    std::time_t lastHit;

    // number of hits
    std::size_t hits;

    // serial number, updated on changes, used to repel maintenace loop from
    // removing
    std::size_t serial;
};

UTILITY_GENERATE_ENUM_IO(Record::Status,
    ((pending))
    ((ready))
    ((outdated))
    ((invalid))
)

typedef std::map<utility::FileId, Record> Drivers;

} // namespace

class DeliveryCache::Detail : boost::noncopyable {
public:
    Detail(DeliveryCache &cache, unsigned int threadCount)
        : cache_(cache), maintenanceTimer_(ios_)
    {
        cleanupLimit_.openFiles = utility::maxOpenFiles() / 2;
        cleanupLimit_.memory
            = std::numeric_limits<decltype(cleanupLimit_.memory)>::max();
        LOG(info3) << "Cleanup limits: " << cleanupLimit_ << ".";

        start(threadCount);
    }

    ~Detail() { stop(); }

    void get(const std::string &path, const Callback &callback
             , const OpenOptions &openOptions
             , boost::tribool checkForChange);

    void post(const DeliveryCache::Callback &callback
              , const std::function<void()> &callable);

private:
    void open(Record &record, const OpenOptions &openOptions);
    void finishOpen(Record &record, const Expected &value);

    void get(std::unique_lock<std::mutex> &lock
             , const std::string &path, const utility::FileId &fid
             , Drivers::iterator idrivers
             , const Callback &callback
             , OpenOptions openOptions
             , const boost::optional<Record::Status> &status
             , boost::tribool checkForChange);

    void start(std::size_t count);
    void stop();
    void worker(std::size_t id);
    void startMaintenance();

    void check();

    DeliveryCache &cache_;

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

    // mime detector
    utility::Magic magic_;
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
        try {
            ios_.run();
            LOG(info2) << "Terminated cache worker id:" << id << ".";
            return;
        } catch (const std::exception &e) {
            LOG(err3)
                << "Uncaught exception (" << typeid(e).name()
                << ") in cache worker: <" << e.what() << ">. Going on.";
        }
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
                    << "Uncaught exception (" << typeid(e).name()
                    << ") in maintenance: <" << e.what() << ">. Going on.";
            }

            // start maintenance again
            startMaintenance();
        };
    });
}

namespace {

struct Opener {
    const OpenInfo &openInfo;
    const OpenOptions &openOptions;
    DeliveryCache &cache;
    const DeliveryCache::Callback &callback;

    DatasetProvider providers;

    Opener(const OpenInfo &openInfo, const OpenOptions &openOptions
           , DeliveryCache &cache, const DeliveryCache::Callback &callback)
        : openInfo(openInfo), openOptions(openOptions), cache(cache)
        , callback(callback), providers(openOptions.datasetProvider)
    {}

    template <DatasetProvider::value_type checkProvider, typename OpenDriver>
    boost::optional<DeliveryCache::Driver> open(const OpenDriver &openDriver)
    {
        LOG(info1)
            << openInfo.path << ": Trying to open "
            << DatasetProvider(checkProvider)
            << " out of " << providers << ".";
        if (!openOptions.datasetProvider.enabled(checkProvider)) { return {}; }

        // unset flag(s) for this provider
        providers.unset(checkProvider);

        if (providers) {
            // this is not the last provider
            try {
                return openDriver(openInfo, openOptions, cache, callback);
            } catch (vs::NoSuchTileSet) {
                return boost::none;
            }
        }

        // last configured provider
        return openDriver(openInfo, openOptions, cache, callback);
    }
};

} // namespace

DeliveryCache::Driver openDriver(const OpenInfo &openInfo
                                 , const OpenOptions &openOptions
                                 , DeliveryCache &cache
                                 , const DeliveryCache::Callback &callback)
{
    LOG(info2) << "Opening driver for " << openInfo.path << " ["
               << openInfo.mime << "].";

    Opener opener(openInfo, openOptions, cache, callback);

    if (auto driver = opener.open<DatasetProvider::vts>(openVts)) {
        return *driver;
    }

    if (auto driver = opener.open<DatasetProvider::vtsLegacy>(openVts0)) {
        return *driver;
    }

    return *opener.open<DatasetProvider::slpk>(openSlpk);
}

void DeliveryCache::Detail::finishOpen(Record &record, const Expected &value)
{
    const auto dispatch([this](CallbackList &callbacks, const Expected &value)
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
            record.set(value);
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

void DeliveryCache::Detail::open(Record &record
                                 , const OpenOptions &openOptions)
{
    ios_.post([this, &record, openOptions]() mutable
    {
        // TODO: do not crash on invalid file
        const auto mime(magic_.mime(record.path));

        try {
            // open driver
            auto driver
                (openDriver(OpenInfo(record.path, mime), openOptions, cache_
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

DeliveryCache::DeliveryCache(unsigned int threadCount)
    : workers_(new Detail(*this, threadCount))
{}

DeliveryCache::~DeliveryCache() {}

void DeliveryCache::Detail
::get(std::unique_lock<std::mutex> &lock
      , const std::string &path, const utility::FileId &fid
      , Drivers::iterator idrivers, const Callback &callback
      , OpenOptions openOptions
      , const boost::optional<Record::Status> &status
      , boost::tribool checkForChange)
{
    openOptions.forcedReopen = false;

    if (idrivers != drivers_.end()) {
        // record found
        auto &record(idrivers->second);

        // check status (either given or fetch fresh)
        switch (status ? *status : record.status(checkForChange)) {
        case Record::Status::ready:
            {
                // valid record, grab driver
                const auto driver(record.driver);
                record.update();

                // unlock and call callback
                lock.unlock();

                callback(driver);
            }
            // done here
            return;

        case Record::Status::invalid:
            // invalid record
            // unlock and call callback
            lock.unlock();
            callback(std::make_exception_ptr
                     (vs::NoSuchTileSet("(cached) No such tileset.")));
            // done here
            return;

        case Record::Status::pending:
            // pending open
            record.openCallbacks.push_back(callback);
            lock.unlock();
            // done here
            return;

        case Record::Status::outdated:
            // dataset has been externally changed, drop driver
            LOG(info1) << "Scheduling outdated driver reopen.";
            record.prepareReopen();
            openOptions.forcedReopen = true;
            // schedule reopen
            break;
        }
    } else {
        // create new entry and grab reference to it
        idrivers = drivers_.insert(Drivers::value_type(fid, Record(path)))
            .first;
        // force recursive reopen on change
        openOptions.forcedReopen = true;
    }

    // remember callback
    idrivers->second.openCallbacks.push_back(callback);

    // call open unlocked
    lock.unlock();
    open(idrivers->second, openOptions);
}

void DeliveryCache::Detail::get(const std::string &path
                                , const Callback &callback
                                , const OpenOptions &openOptions
                                , boost::tribool checkForChange)
{
    const auto fid(utility::FileId::from(path));

    std::unique_lock<std::mutex> guard(mutex_);
    auto idrivers(drivers_.find(fid));
    get(guard, path, fid, idrivers, callback, openOptions
        , boost::none, checkForChange);
}

void DeliveryCache::Detail::post(const DeliveryCache::Callback &callback
                                 , const std::function<void()> &callable)
{
    ios_.post([=]()
    {
        try {
            callable();
        } catch (...) {
            callback(std::current_exception());
        }
    });
}

namespace {

boost::tribool checkForChange(const OpenOptions &openOptions)
{
    if (openOptions.forcedReopen) { return true; }
    return boost::indeterminate;
}

} // namespace

void DeliveryCache::get(const std::string &path, const Callback &callback
                        , const OpenOptions &openOptions)
{
    workers_->get(path, callback, openOptions, checkForChange(openOptions));
}

void DeliveryCache::post(const DeliveryCache::Callback &callback
                         , const std::function<void()> &callable)
{
    workers_->post(callback, callable);
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
    std::size_t oldSerial;

    RecordWrapper(Drivers::iterator idrivers)
        : idrivers(idrivers), record(&idrivers->second)
        , resources(record->driver->resources())
        , oldSerial(record->serial)
    {}

    bool changed() const {
        return record->driver->externallyChanged();
    }

    bool operator<(const RecordWrapper &o) const {
        return resources < o.resources;
    }

    bool reopened() const {
        return oldSerial != record->serial;
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
        // fetch status, do not check for change
        switch (idrivers->second.status(false)) {
        case Record::Status::ready:
        case Record::Status::outdated:
            // mark it down
            records.emplace_back(idrivers++);
            resources += records.back().resources;
            break;

        case Record::Status::invalid:
            // invalid -> remove immediately
            idrivers = erase(idrivers);
            break;

        case Record::Status::pending:
            // not ready yet -> skip
            ++idrivers;
            break;
        }
    }

    // drivers to remove
    RecordWrapper::list toRemove;

    // analyze open drivers without lock
    {
        Unlocker unlock(guard);

        // sort by number of resources consumed
        std::sort(records.begin(), records.end());

        // process all records
        for (const auto &rw : records) {
            if (rw.changed()) {
                // dataset has been changed, plan removal
                toRemove.push_back(rw);
                resources -= rw.resources;
                continue;
            }

            if (resources > cleanupLimit_) {
                // too much memory, remove
                toRemove.push_back(rw);
                resources -= rw.resources;
                continue;
            }
        }
    }

    // under lock again

    // drop marked drivers (unless they have been changed meanwhile)
    for (const auto &rw : toRemove) {
        if (!rw.reopened()) { erase(rw.idrivers); }
    }
}

SplitPath OpenOptions::splitFilePath(const boost::filesystem::path &filePath)
    const
{
    SplitPath sp;
    if (datasetProvider.enabled(DatasetProvider::vts)
        && vtsSplitFilePath(filePath, sp)) { return sp; }

    if (datasetProvider.enabled(DatasetProvider::vtsLegacy)
        && vts0SplitFilePath(filePath, sp)) { return sp; }

    if (datasetProvider.enabled(DatasetProvider::slpk)
        && slpkSplitFilePath(filePath, sp)) { return sp; }

    // default
    return SplitPath(filePath.parent_path(), filePath.filename());
}
