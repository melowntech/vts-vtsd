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
#include <mutex>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/composite_key.hpp>

#include "./driver.hpp"

class DeliveryCache : boost::noncopyable {
public:
    DeliveryCache();

    DriverWrapper::pointer get(const std::string &path);

    void cleanup();

    void cleanup(std::unique_lock<std::mutex>&);

    void flush(std::unique_lock<std::mutex>&);

private:
    DriverWrapper::pointer openDriver(const std::string &path)
        const;

    std::mutex mutex_;
    typedef DriverWrapper::pointer Driver;

    struct Record {
        Record(const std::string &path, const Driver &driver
               , vs::Resources &totalResources)
            : path(path), flags(0) // TODO: remove flags
            , driver(driver), lastHit(std::time(nullptr)), hits(1)
            , resources(driver->resources())
            , totalResources(totalResources)
        {
            totalResources += resources;
        }

        ~Record() { totalResources -= resources; }

        void update() {
            lastHit = std::time(nullptr);
            ++hits;
            // update resources
            totalResources -= resources;
            resources = driver->resources();
            totalResources += resources;
        }

        decltype(vs::Resources::openFiles) openFiles() const {
            return resources.openFiles;
        }

        std::string path;
        int flags;
        mutable Driver driver;
        std::time_t lastHit;
        std::size_t hits;
        vs::Resources resources;
        vs::Resources &totalResources;
    };

    struct PathIdx {};
    struct ResourcesIdx {};

    // Resources are sorted in descending order => this helps to kill the
    // greatest resource usurpers first.
    typedef boost::multi_index_container<
        Record
        , boost::multi_index::indexed_by<
              boost::multi_index::ordered_unique<
                  boost::multi_index::tag<PathIdx>
                  , boost::multi_index::composite_key
                  <Record
                   , BOOST_MULTI_INDEX_MEMBER
                   (Record, decltype(Record::path), path)
                   , BOOST_MULTI_INDEX_MEMBER
                   (Record, decltype(Record::flags), flags)>
                  >
              , boost::multi_index::ordered_non_unique
              <boost::multi_index::tag<ResourcesIdx>
               , BOOST_MULTI_INDEX_MEMBER
               (Record, decltype(Record::resources), resources)
               , std::greater<decltype(Record::resources)> >
              >
        > Drivers;

    Drivers drivers_;

    vs::Resources totalResources_;
    vs::Resources cleanupLimit_;

    std::time_t nextFlush_;
};

#endif // httpd_delivery_cache_hpp_included_
