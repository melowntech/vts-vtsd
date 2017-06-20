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

#ifndef httpd_delivery_cache_hpp_included_
#define httpd_delivery_cache_hpp_included_

#include <ctime>
#include <memory>
#include <functional>
#include <vector>
#include <mutex>
#include <thread>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/composite_key.hpp>

#include "utility/expected.hpp"
#include "utility/filesystem.hpp"

#include "vts-libs/vts/options.hpp"

#include "./driver.hpp"

class DeliveryCache : boost::noncopyable {
public:
    DeliveryCache(unsigned int threadCount);
    ~DeliveryCache();

    typedef utility::Expected<DriverWrapper::pointer> Expected;
    typedef std::function<void(const Expected&)> Callback;

    typedef std::vector<Callback> CallbackList;

    /** Calls callback with driver for given path. Call is immediated if driver
     *  is already open or postponed when driver is available.
     */
    void get(const std::string &path
             , const vtslibs::vts::OpenOptions &openOptions
             , const Callback &callback);

    /** Returns driver for given path. Blocking call.
     */
    DriverWrapper::pointer get(const std::string &path
                               , const vtslibs::vts::OpenOptions &openOptions);

private:
    void maintenance();

    void check(std::time_t now);

    std::mutex mutex_;
    std::atomic<bool> running_;
    std::thread maintenance_;

    typedef DriverWrapper::pointer Driver;

    struct Record {
        Record(const std::string &path)
            : path(path), lastHit(std::time(nullptr))
        {}

        ~Record() {}

        void update(std::time_t now) {
            lastHit = now;
        }

        // path to dataset
        std::string path;
        // pointer to driver
        Driver driver;
        // callbacks waiting for driver to be opened
        CallbackList openCallbacks;

        // time of last cache hit
        std::time_t lastHit;
    };

    typedef std::map<utility::FileId, Record> Drivers;

    Drivers drivers_;

    vs::Resources totalResources_;
    vs::Resources cleanupLimit_;

    class Workers;
    std::unique_ptr<Workers> workers_;
};

#endif // httpd_delivery_cache_hpp_included_
