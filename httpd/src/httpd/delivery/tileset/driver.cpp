#include "vts-libs/storage/error.hpp"
#include "vts-libs/tilestorage/driver.hpp"
#include "vts-libs/tilestorage/io.hpp"
#include "vts-libs/tilestorage/support.hpp"
#include "vts-libs/tilestorage/tileop.hpp"
#include "vts-libs/tilestorage/config.hpp"

#include "./driver.hpp"

namespace fs = boost::filesystem;
namespace ts = vadstena::tilestorage;
namespace vs = vadstena::storage;

namespace {

struct TsFileInfo : public FileInfo {
    // tileId, valid only when (type == Type::tileFile)
    ts::TileId tileId;

    TsFileInfo(const std::string &path, int flags);
};

namespace constants {
    const std::string Config("mapConfig.json");
    const std::string Self(".");
    const std::string Index("index.html");
}

TsFileInfo::TsFileInfo(const std::string &p, int flags)
    : FileInfo(p)
{
    if (ts::fromFilename(tileId, tileFile, path.c_str())) {
        type = Type::tileFile;
        return;
    }

    if (constants::Config == path) {
        type = Type::file;
        file = vs::File::config;
        return;
    }

    if (flags & TILESET_OPEN_DISABLE_BROWSER) {
        LOG(debug) << "Browser disabled, skipping browser files.";
        return;
    }

    LOG(debug) << "Browser enabled, checking browser files.";

    // translate "." -> index
    if (constants::Self == path) { path = constants::Index; }

    auto fsupport(ts::supportFiles.find(path));
    if (fsupport != ts::supportFiles.end()) {
        type = Type::support;
        support = &*fsupport;
    }
}

class TsDriver : public DriverWrapper
{
public:
    TsDriver(const ts::Driver::pointer &driver)
        : driver_(driver)
    {}

    virtual vs::Resources resources() const {
        return driver_->resources();
    }

    virtual bool externallyChanged() const {
        return driver_->externallyChanged();
    }

    virtual void handle(Sink sink, const std::string &path
                        , const LocationConfig &config);

#if 0
    virtual std::unique_ptr<Handle>
    openFile(LockGuard::OptionalMutex &mutex, const std::string &path
             , int flags, const tileset_Variables::Wrapper &variables);
#endif

private:
    ts::Driver::pointer driver_;
};

void TsDriver::handle(Sink sink, const std::string &path
                      , const LocationConfig &config)
{
    (void) sink;
    (void) path;
    (void) config;
    throw InternalError("Not implemented yet");
}

#if 0
std::unique_ptr<Handle>
TsDriver::openFile(LockGuard::OptionalMutex &mutex, const std::string &path
                   , int flags, const tileset_Variables::Wrapper &variables)
{
    TsFileInfo info(path, flags);

    LockGuard guard(mutex);

    switch (info.type) {
    case TsFileInfo::Type::file:
        return std::unique_ptr<Handle>
            (new StreamHandle
             (driver_->input(info.file), TILESET_FILETYPE_FILE));

    case TsFileInfo::Type::tileFile:
        return std::unique_ptr<Handle>
            (new StreamHandle
             (driver_->input(info.tileId, info.tileFile)
              , TILESET_FILETYPE_TILE));

    case TsFileInfo::Type::support:
        return std::unique_ptr<Handle>
            (new BrowserFileHandle(info.support->second, variables));

    default: break;
    }

    LOGTHROW(err1, vs::NoSuchFile)
        << "Unknown file to open: \"" << info.path << "\".";
    throw; // shut up, compiler
}
#endif

} // namespace

DriverWrapper::pointer openTileSet(const std::string &path, int flags)
{
    (void) flags;
    return std::make_shared<TsDriver>
        (ts::Driver::open(path, ts::OpenMode::readOnly));
}
