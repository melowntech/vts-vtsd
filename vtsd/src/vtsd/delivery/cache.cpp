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

#include <boost/asio.hpp>

#include "utility/rlimit.hpp"

#include "vts-libs/storage/error.hpp"
#include "vts-libs/storage/io.hpp"

#include "./cache.hpp"
#include "./vts/driver.hpp"
#include "./tileset/driver.hpp"
#include "./vts0/driver.hpp"

namespace asio = boost::asio;

/** Time between flushes.
 */
constexpr std::time_t CHECK_INTERVAL(60);

/** Maximal time between hits in cache for single record.
 */
constexpr std::time_t MAX_INTERVAL_BETWEEN_HITS(600);

typedef std::unique_lock<std::mutex> UniqueLock;

class DeliveryCache::Workers : boost::noncopyable {
public:
    Workers(unsigned int threadCount, std::mutex &mutex, Drivers &drivers)
        : work_(ios_), mutex_(mutex), drivers_(drivers)
    {
        start(threadCount);
    }

    ~Workers() { stop(); }

    void open(Drivers::iterator &irecord
              , const vtslibs::vts::OpenOptions &openOptions);

private:
    void start(std::size_t count);
    void stop();
    void worker(std::size_t id);
    // void post(const Generator::Task &task, Sink sink);

    asio::io_service ios_;

    /** Processing pool stuff.
     */
    asio::io_service::work work_;
    std::vector<std::thread> workers_;

    // cache
    std::mutex &mutex_;
    Drivers drivers_;
};

void DeliveryCache::Workers::start(std::size_t count)
{
    // make sure threads are released when something goes wrong
    struct Guard {
        Guard(const std::function<void()> &func) : func(func) {}
        ~Guard() { if (func) { func(); } }
        void release() { func = {}; }
        std::function<void()> func;
    } guard([this]() { stop(); });

    for (std::size_t id(1); id <= count; ++id) {
        workers_.emplace_back(&Workers::worker, this, id);
    }

    guard.release();
}

void DeliveryCache::Workers::stop()
{
    LOG(info2) << "Stopping delivery cache workers.";
    ios_.stop();

    while (!workers_.empty()) {
        workers_.back().join();
        workers_.pop_back();
    }
}

void DeliveryCache::Workers::worker(std::size_t id)
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
                << "Uncaught exception in cache worker: <" << e.what()
                << ">. Going on.";
        }
    }
}

DriverWrapper::pointer openDriver
(const std::string &path, const vtslibs::vts::OpenOptions &openOptions)
{
    // try VTS
    try { return openVts(path, openOptions); } catch (vs::NoSuchTileSet) {}

    // try VTS0
    try { return openVts0(path); } catch (vs::NoSuchTileSet) {}

    // finaly, try old TS
    return openTileSet(path);
}

void DeliveryCache::Workers
::open(Drivers::iterator &idrivers
       , const vtslibs::vts::OpenOptions &openOptions)
{
    ios_.post([this, idrivers, &openOptions]() mutable
    {
        const auto dispatch([this](CallbackList &callbacks
                                   , const Expected &value)
        {
            for (const auto &callback : callbacks) {
                // dispatch in thread pool
                LOG(info4) << "Dispatching open driver callback.";
                ios_.post([callback, value]() { callback(value); });
            }
        });

        try {
            // open driver
            auto &record(idrivers->second);
            auto driver(openDriver(record.path, openOptions));

            // driver open -> store in the record
            CallbackList callbacks;
            {
                std::unique_lock<std::mutex> guard(mutex_);
                // store driver and steal callbacks
                record.driver = driver;
                std::swap(callbacks, record.openCallbacks);
            }

            // dispatch to callbacks (unlocked)
            dispatch(callbacks, driver);
        } catch (...) {
            // open failed

            CallbackList callbacks;
            {
                std::unique_lock<std::mutex> guard(mutex_);
                // steal callbacks
                std::swap(callbacks, idrivers->second.openCallbacks);

                // kill driver record
                drivers_.erase(idrivers);
            }

            // dispatch exception to interested parties
            dispatch(callbacks, std::current_exception());
        }
    });
}

DeliveryCache::DeliveryCache(unsigned int threadCount)
    : running_(true), workers_(new Workers(threadCount, mutex_, drivers_))
{
    cleanupLimit_.openFiles = utility::maxOpenFiles() / 2;
    cleanupLimit_.memory
        = std::numeric_limits<decltype(cleanupLimit_.memory)>::max();

    LOG(info3) << "Cleanup limits: " << cleanupLimit_ << ".";

    maintenance_ = std::thread(std::bind(&DeliveryCache::maintenance, this));
}

DeliveryCache::~DeliveryCache()
{
    running_ = false;
    maintenance_.join();
}

void DeliveryCache::get(const std::string &path
                        , const vtslibs::vts::OpenOptions &openOptions
                        , const Callback &callback)
{
    const auto fid(utility::FileId::from(path));

    std::unique_lock<std::mutex> guard(mutex_);

    auto fdrivers(drivers_.find(fid));

    if (fdrivers != drivers_.end()) {
        // record found
        auto &record(fdrivers->second);

        if (record.driver) {
            // valid record, grab driver
            const auto driver(record.driver);

            // unlock and call callback
            guard.unlock();
            callback(driver);
            return;
        }

        // pending open
        record.openCallbacks.push_back(callback);
        return;
    }

    // create new entry and grab reference to it
    auto idrivers(drivers_.insert(Drivers::value_type(fid, Record(path)))
                  .first);

    idrivers->second.openCallbacks.push_back(callback);

    workers_->open(idrivers, openOptions);
}

DriverWrapper::pointer
DeliveryCache::get(const std::string &path
                   , const vtslibs::vts::OpenOptions &openOptions)
{
    LOG(info1) << "Getting driver for tileset at: \"" << path << "\".";

    std::promise<DriverWrapper::pointer> promise;
    get(path, openOptions, [&](const Expected &driver)
    {
        try {
            promise.set_value(driver.get());
        } catch (...) {
            promise.set_exception(std::current_exception());
        }
    });

    return promise.get_future().get();
}

void DeliveryCache::maintenance()
{
    dbglog::thread_id("cache");

    const auto nextCheck(std::time(nullptr) + CHECK_INTERVAL);

    while (running_) {
        try {
            LOG(info4) << "Maintenance";
            // sleep(CHECK_INTERVAL);
            const auto now(std::time(nullptr));
            if (now >= nextCheck) { check(now); }
            ::sleep(1);
        } catch (const std::exception &e) {
            LOG(warn2) << "Cache maintenance: unexpected exception: <"
                       << e.what() << ">; going on.";
        }
    };

    LOG(info3) << "Maintenance thread terminating";
}

void DeliveryCache::check(std::time_t now)
{
    (void) now;
}
