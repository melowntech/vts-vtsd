#include <mutex>

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

    TsFileInfo(const std::string &path, const LocationConfig &config);
};

namespace constants {
    const std::string Config("mapConfig.json");
    const std::string Self(".");
    const std::string Index("index.html");
}

TsFileInfo::TsFileInfo(const std::string &p, const LocationConfig &config)
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

    if (!config.enableBrowser) {
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

private:
    ts::Driver::pointer driver_;
    std::mutex mutex_;
};

void TsDriver::handle(Sink sink, const std::string &path
                      , const LocationConfig &config)
{
    TsFileInfo info(path, config);

    switch (info.type) {
    case FileInfo::Type::file: {
        std::unique_lock<std::mutex> guard(mutex_);
        return sink.content(driver_->input(info.file), FileClass::config);
    }

    case FileInfo::Type::tileFile:
    {
        std::unique_lock<std::mutex> guard(mutex_);
        return sink.content(driver_->input(info.tileId, info.tileFile)
                            , FileClass::data);
    }

    case FileInfo::Type::support:
        return sink.content(info.support->second);

    default: break;
    }

    // wtf?
    sink.error(utility::makeError<NotFound>("Unknown file."));
}

} // namespace

DriverWrapper::pointer openTileSet(const std::string &path)
{
    return std::make_shared<TsDriver>
        (ts::Driver::open(path, ts::OpenMode::readOnly));
}
