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

    DriverWrapper::pointer get(const std::string &path, int flags);

    void cleanup();

    void cleanup(std::unique_lock<std::mutex>&);

    void flush(std::unique_lock<std::mutex>&);

private:
    DriverWrapper::pointer openDriver(const std::string &path, int flags)
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
