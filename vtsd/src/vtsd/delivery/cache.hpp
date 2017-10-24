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

#include "utility/expected.hpp"
#include "utility/filesystem.hpp"

#include "vts-libs/vts/options.hpp"

#include "./driver.hpp"

typedef std::pair<boost::filesystem::path, boost::filesystem::path> SplitPath;

struct OpenOptions {
    /** set to true if we are getting driver as a result of another driver
     *  reopen
     */
    vtslibs::vts::OpenOptions openOptions;
    boost::tribool forcedReopen;
    DatasetProvider datasetProvider;

    OpenOptions() : forcedReopen(false), datasetProvider() {}
    explicit OpenOptions(const vtslibs::vts::OpenOptions &openOptions)
        : openOptions(openOptions), forcedReopen(boost::indeterminate)
        , datasetProvider() {}
    explicit OpenOptions(boost::tribool forcedReopen)
        : forcedReopen(forcedReopen), datasetProvider() {}
    explicit OpenOptions(DatasetProvider datasetProvider)
        : forcedReopen(boost::indeterminate)
        , datasetProvider(datasetProvider) {}

    OpenOptions& setOpenOptions(const vtslibs::vts::OpenOptions &v) {
        openOptions = v; return *this; }
    OpenOptions& setForcedReopen(boost::tribool v) {
        forcedReopen = v; return *this;
    }
    OpenOptions& setDatasetProvider(const DatasetProvider &v) {
        datasetProvider = v; return *this;
    }

    SplitPath splitFilePath(const boost::filesystem::path &filePath) const;
};

class DeliveryCache : boost::noncopyable {
public:
    DeliveryCache(unsigned int threadCount);
    ~DeliveryCache();

    typedef DriverWrapper::pointer Driver;
    typedef utility::Expected<Driver> Expected;
    typedef std::function<void(const Expected&)> Callback;
    typedef std::vector<Callback> CallbackList;

    /** Calls callback with driver for given path. Call is immediate if driver
     *  is already open or postponed until driver is available.
     *
     * \param path filesystem path of dataset to open
     * \param callback completion handler
     */
    void get(const std::string &path, const Callback &callback
             , const OpenOptions &options);

    /** Use cache's async mechanism to run dunction at background.
     */
    void post(const DeliveryCache::Callback &callback
              , const std::function<void()> &callable);

private:
    class Detail;
    std::unique_ptr<Detail> workers_;
};

#endif // httpd_delivery_cache_hpp_included_
