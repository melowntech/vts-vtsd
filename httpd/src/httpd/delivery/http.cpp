#include <system_error>
#include <algorithm>
#include <thread>
#include <limits>
#include <ctime>
#include <new>

#include <boost/filesystem.hpp>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/composite_key.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"
#include "utility/rlimit.hpp"

#include "vts-libs/storage/error.hpp"
#include "vts-libs/storage/io.hpp"
#include "vts-libs/registry.hpp"

#include "./vadstena-http.h"
#include "./lock.hpp"
#include "./driver.hpp"
#include "./variables.hpp"

#include "./vts/driver.hpp"
#include "./tileset/driver.hpp"
#include "./vts0/driver.hpp"
#include "./ts2vts0/driver.hpp"

namespace fs = boost::filesystem;
namespace vs = vadstena::storage;
namespace vr = vadstena::registry;

/** Make all API functions visible.
 */
int tileset_errno()
    __attribute__ ((visibility("default")));

const char* tileset_errorMessage()
    __attribute__ ((visibility("default")));

void tileset_initialize(int threadSafe)
    __attribute__ ((visibility("default")));

void tileset_finish()
    __attribute__ ((visibility("default")));

int tileset_cleanup()
    __attribute__ ((visibility("default")));

int tileset_registry(const char *registry)
    __attribute__ ((visibility("default")));

int tileset_logMask(const char *mask, int len)
    __attribute__ ((visibility("default")));

int tileset_logFile(const char *filename, int len)
    __attribute__ ((visibility("default")));

int tileset_logConsole(int enabled)
    __attribute__ ((visibility("default")));

tileset_File* tileset_open(const char *path, int flags)
    __attribute__ ((visibility("default")));

tileset_File* tileset_open2(const char *path, int flags
                            , const tileset_Variables *variables)
    __attribute__ ((visibility("default")));

int tileset_close(tileset_File *file)
    __attribute__ ((visibility("default")));

int tileset_read(tileset_File *file, void *buf, size_t size)
    __attribute__ ((visibility("default")));

int tileset_stat(tileset_File *file, tileset_Stat *stat)
    __attribute__ ((visibility("default")));

int tileset_getOpenFile(tileset_File *file, tileset_OpenFile *openFile)
    __attribute__ ((visibility("default")));

int tileset_getMemory(tileset_File *file, tileset_Memory *memory)
    __attribute__ ((visibility("default")));

tileset_Variables* tileset_createVariables()
    __attribute__ ((visibility("default")));

int tileset_destroyVariables(tileset_Variables *variables)
    __attribute__ ((visibility("default")));

tileset_Variables* tileset_cloneVariables(const tileset_Variables *variables)
    __attribute__ ((visibility("default")));

int tileset_mergeVariables(tileset_Variables *dst, const tileset_Variables *src
                           , int overwrite)
    __attribute__ ((visibility("default")));

int tileset_addVariable(tileset_Variables *variables
                        , const char *key, size_t keySize
                        , const char *value, size_t valueSize
                        , int overwrite)
    __attribute__ ((visibility("default")));

namespace {

const tileset_Variables emptyVariables;

const tileset_Variables defaultVariables(tileset_Variables::Variables{
    {
        "VTS_BUILTIN_BROWSER_URL"
        , "//cdn.melown.com/libs/melownjs/builtin/stable"
    }
});

const tileset_Variables oldDefaultVariables(tileset_Variables::Variables{
    {
        "VTS_BUILTIN_BROWSER_URL"
        , "//cdn.iris-test.citationtech.net/libs/melownjs/builtin/devel"
    }
});


/** Time between flushes.
 */
constexpr std::time_t FLUSH_INTERVAL(60);

/** Maximal time between hits in cache for single record.
 */
constexpr std::time_t MAX_INTERVAL_BETWEEN_HITS(600);

struct Uninitialized {};

__thread int error(TILESET_OK);
__thread char *errorMessage(nullptr);

void setError(int e, const std::string &msg)
{
    error = e;
    delete errorMessage;
    errorMessage = 0;
    errorMessage = new char[msg.size() + 1];
    std::copy(msg.c_str(), msg.c_str() + msg.size() + 1, errorMessage);
    LOG(debug) << "Set error to: <" << errorMessage << ">.";
}

/** Swallows all exceptions and sets error and errorMessage
 */
template <typename Body>
inline auto safe(Body body, decltype(body()) dflt) -> decltype(body())
{
    try {
        error = TILESET_OK;
        return body();
    } catch (Uninitialized) {
        setError(TILESET_UNINITIALIZED, "libvadstena-http is not initialized");

    } catch (const std::system_error &e) {
        LOG(err1) << e.what();
        if (e.code().category() == std::system_category()) {
            if (e.code().value() == ENOENT) {
                setError(TILESET_NOTFOUND, e.what());
            } else {
                // value is an errno
                setError(TILESET_ERRNO, e.what());
                errno = e.code().value();
            }
        } else {
            // something other
            setError(TILESET_FAILED, e.what());
        }

    } catch (const vs::NoSuchTileSet &e) {
        LOG(err1) << e.what();
        setError(TILESET_NOTFOUND, e.what());

    } catch (const vs::NoSuchFile &e) {
        LOG(err1) << e.what();
        setError(TILESET_FILE_NOTFOUND, e.what());

    } catch (std::domain_error &e) {
        LOG(err1) << e.what();
        setError(TILESET_NOTFOUND, e.what());

    } catch (std::exception &e) {
        LOG(err1) << e.what();
        setError(TILESET_FAILED, e.what());

    } catch (const char *e) {
        LOG(err1) << e;
        setError(TILESET_FAILED, e);

    } catch (...) {
        setError(TILESET_FAILED, "operation failed; unknown exception");
    }
    return dflt;
}

} // namespace

int tileset_errno() { return error; }
const char* tileset_errorMessage() { return errorMessage; }

namespace {

const int DriverChangingFlags(TILESET_OPEN_ENABLE_VTS0_ADAPTER);

inline int driverChangingFlags(int flags)
{
    return flags & DriverChangingFlags;
}

class Cache : boost::noncopyable {
public:
    DriverWrapper::pointer get(const std::string &path, int flags);

    Handle::pointer openFile(const DriverWrapper::pointer &driver
                             , const std::string &path
                             , int flags
                             , const tileset_Variables::Wrapper &variables);

    void cleanup();

    void cleanup(LockGuard&);

    void flush(LockGuard&);

    static Cache& cache() {
        if (!cache_) { throw Uninitialized(); }
        return *cache_;
    };

    static void initialize(int threadSafe) {
        if (!cache_) { cache_.reset(new Cache(threadSafe)); }
    }

    static void finish() { if (cache_) { cache_.reset(); } }

private:
    Cache(int threadSafe);

    DriverWrapper::pointer openDriver(const std::string &path, int flags)
        const;

    static std::shared_ptr<Cache> cache_;
    LockGuard::OptionalMutex mutex_;
    typedef DriverWrapper::pointer Driver;

    struct Record {
        Record(const std::string &path, int flags
               , const Driver &driver, vs::Resources &totalResources)
            : path(path), flags(driverChangingFlags(flags))
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

std::shared_ptr<Cache> Cache::cache_;

Cache::Cache(int threadSafe)
    : nextFlush_(std::time(nullptr) + FLUSH_INTERVAL)
{
    if (threadSafe) {
        mutex_ = {};
    }

    cleanupLimit_.openFiles = utility::maxOpenFiles() / 2;
    cleanupLimit_.memory
        = std::numeric_limits<decltype(cleanupLimit_.memory)>::max();

    LOG(info3) << "Cleanup limits: " << cleanupLimit_ << ".";

    dbglog::set_mask(dbglog::level::none);
    dbglog::log_console(false);
}

DriverWrapper::pointer Cache::openDriver(const std::string &path, int flags)
    const
{
    // try VTS
    try { return openVts(path, flags); } catch (vs::NoSuchTileSet) {}

    // try VTS0
    try { return openVts0(path, flags); } catch (vs::NoSuchTileSet) {}

    // finaly, try old TS

    // check for adapter
    if (flags & TILESET_OPEN_ENABLE_VTS0_ADAPTER) {
        // use TS->VTS0 adapter
        return openTs2Vts0(path, flags);
    }

    return openTileSet(path, flags);
}

DriverWrapper::pointer Cache::get(const std::string &path, int flags)
{
    LOG(info1) << "Getting driver for tileset at: \"" << path << "\".";

    LockGuard guard(mutex_);

    // clean resource hoggers and flush changed tile sets
    cleanup(guard);
    flush(guard);

    auto fdrivers(drivers_.find
                  (boost::make_tuple(path, driverChangingFlags(flags))));

    bool replace(false);

    if (fdrivers != drivers_.end()) {
        auto driver(fdrivers->driver);
        replace = (driver->hotContent() && driver->externallyChanged());

        if (!replace) {
            // modify record
            // TODO: what if this fails (can it fail???)
            drivers_.modify(fdrivers, [](Record &r) { r.update(); });
            return fdrivers->driver;
        }
    }

    // open driver
    auto driver(openDriver(path, flags));

    if (replace) {
        // replace existing driver (it doesn't contribute to the key)
        fdrivers->driver = driver;
    } else {
        // cache new record
        drivers_.insert(Record(path, flags, driver, totalResources_));
    }

    // done
    return driver;
}

std::unique_ptr<Handle>
Cache::openFile(const DriverWrapper::pointer &driver
                , const std::string &path
                , int flags
                , const tileset_Variables::Wrapper &variables)
{
    return driver->openFile(mutex_, path, flags, variables);
}

void Cache::cleanup()
{
    LockGuard guard(mutex_);
    cleanup(guard);
    flush(guard);
}

void Cache::cleanup(LockGuard&)
{
    if (totalResources_ < cleanupLimit_) { return; }

    LOG(info2) << "Resource limit reached (total: " << totalResources_
               << " >= limit " << cleanupLimit_ << ".";

    auto &idx(drivers_.get<ResourcesIdx>());

    for (auto iidx(idx.begin());
         (totalResources_ < cleanupLimit_) && (iidx != idx.end()); )
    {
        const auto &r(*iidx);
        LOG(info1) << "Removing cached tileset <" << r.path << "> "
                   << "with resources " << r.resources << ".";
        iidx = idx.erase(iidx);
    }
}

void Cache::flush(LockGuard&)
{
    const auto now(std::time(nullptr));
    if (nextFlush_ > now) { return; }

    const auto killHit(now - MAX_INTERVAL_BETWEEN_HITS);

    for (auto iidx(drivers_.begin()); (iidx != drivers_.end()); ) {
        const auto &r(*iidx);
        // initialize remove flag based on the last hit
        bool remove(r.lastHit < killHit);
        if (!remove) {
            try {
                remove = r.driver->externallyChanged();
            } catch (const std::exception &e) {
                LOG(warn2) << "External change test failed: " << e.what()
                           << "; removing driver.";
                remove = true;
            } catch (...) {
                LOG(warn2)
                    << "External change test failed (unknown error);"
                    << " removing driver.";
                remove = true;
            }
        }

        if (remove) {
            LOG(info1) << "Removing cached tileset <" << r.path << "> "
                       << " that has been externally changed or timed out.";
            iidx = drivers_.erase(iidx);
        } else {
            ++iidx;
        }
    }

    nextFlush_ = now + FLUSH_INTERVAL;
}

} // namespace

void tileset_initialize(int threadSafe)
{
    Cache::initialize(threadSafe);
}

void tileset_finish()
{
    Cache::finish();
}

int tileset_cleanup()
{
    return safe([&]() -> int
    {
        Cache::cache().cleanup();
        return 0;
    }, -1);
}

int tileset_registry(const char *registry)
{
    return safe([&]() -> int
    {
        if (!registry) {
            vr::init(vr::defaultPath());
        } else {
            vr::init(registry);
        }
        return 0;
    }, -1);
}

namespace {

inline std::string asString(const char *mask, int len)
{
    if (len >= 0) { return { mask, len }; }
    return mask;
}

} // namespace

int tileset_logMask(const char *mask, int len)
{
    return safe([&]() -> int
    {
        dbglog::set_mask(asString(mask, len));
        return 0;
    }, -1);
}

int tileset_logFile(const char *filename, int len)
{
    return safe([&]() -> int
    {
        dbglog::log_file(asString(filename, len));
        return 0;
    }, -1);
}

int tileset_logConsole(int enabled)
{
    return safe([&]() -> int
    {
        dbglog::log_console(enabled);
        return 0;
    }, -1);
}

tileset_File* tileset_open2(const char *p, int flags
                            , const tileset_Variables *variables)
{
    return safe([&]() -> tileset_File*
    {
        auto &cache(Cache::cache());
        fs::path path(p);

        try {
            return cache.openFile
                (cache.get(path.parent_path().string(), flags)
                 , path.filename().string(), flags
                 , { (variables ? *variables : emptyVariables)
                        , defaultVariables}).release();
        } catch (vs::NoSuchTileSet) {
            // check whether path exists
            if (!exists(path)) { throw; }
            // OK, referenced file exists -> it could be tileset
            cache.get(path.string(), flags);
            // we are still here -> bingo
            // and now tell the upper layer it has append slash
            setError(TILESET_MUST_REDIRECT
                     , utility::formatError
                     ("Redirect to \"%s/\" to get index of the tileset."
                      , path.string()));
            return nullptr;
        } catch (NoBody) {
            // no body of given tileset -> tell upstream
            setError(TILESET_NOTFOUND, "no body");
            return nullptr;
        }
    }, nullptr);
}

tileset_File* tileset_open(const char *p, int flags)
{
    return tileset_open2(p, flags, &oldDefaultVariables);
}

int tileset_close(tileset_File *file)
{
    auto *handle(Handle::get(file));
    return safe([&]() -> int
    {
        handle->close();
        delete handle;
        return 0;
    }, -1);
}

int tileset_read(tileset_File *file, void *buf, size_t size)
{
    auto *handle(Handle::get(file));
    return safe([&]() -> int
    {
        return handle->read(static_cast<char*>(buf), size);
    }, -1);
}

int tileset_stat(tileset_File *file, tileset_Stat *stat)
{
    auto *handle(Handle::get(file));
    return safe([&]() -> int
    {
        handle->stat(*stat);
        return 0;
    }, -1);
}

int tileset_getOpenFile(tileset_File *file, tileset_OpenFile *openFile)
{
    auto *handle(Handle::get(file));
    return safe([&]() -> int
    {
        handle->fd(*openFile);
        return 0;
    }, -1);
}

int tileset_getMemory(tileset_File *file, tileset_Memory *memory)
{
    auto *handle(Handle::get(file));
    return safe([&]() -> int
    {
        handle->memory(*memory);
        return 0;
    }, -1);
}

tileset_Variables* tileset_createVariables()
{
    return new (std::nothrow) tileset_Variables();
}

int tileset_destroyVariables(tileset_Variables *variables)
{
    return safe([&]() -> int
    {
        delete variables;
        return 0;
    }, -1);
}

tileset_Variables* tileset_cloneVariables(const tileset_Variables *variables)
{
    return new (std::nothrow) tileset_Variables(*variables);
}

int tileset_mergeVariables(tileset_Variables *dst, const tileset_Variables *src
                           , int overwrite)
{
    return safe([&]() -> int
    {
        dst->update(*src, overwrite);
        return 0;
    }, -1);
}

int tileset_addVariable(tileset_Variables *variables
                        , const char *key, size_t keySize
                        , const char *value, size_t valueSize
                        , int overwrite)
{
    return safe([&]() -> int
    {
        if (overwrite) {
            variables->replace(std::string(key, keySize)
                               , std::string(value, valueSize));
        } else {
            variables->add(std::string(key, keySize)
                           , std::string(value, valueSize));
        }
        return 0;
    }, -1);
}
