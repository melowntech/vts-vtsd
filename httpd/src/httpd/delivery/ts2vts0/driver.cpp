#include "vts-libs/storage/error.hpp"
#include "vts-libs/vts0/driver.hpp"
#include "vts-libs/vts0/support.hpp"
#include "vts-libs/vts0/tileop.hpp"
#include "vts-libs/vts0/driver/tilardriver.hpp"
#include "vts-libs/vts0/config.hpp"

#include "vts-libs/tilestorage/driver.hpp"
#include "vts-libs/tilestorage/io.hpp"
#include "vts-libs/tilestorage/support.hpp"
#include "vts-libs/tilestorage/tileop.hpp"
#include "vts-libs/tilestorage/config.hpp"

#include "../vadstena-http.h"
#include "../vts0/driver.hpp"
#include "./driver.hpp"
#include "./ts2vts0.hpp"
#include "./metatile-convert.hpp"

namespace fs = boost::filesystem;
namespace vts0 = vadstena::vts0;
namespace vs = vadstena::storage;

namespace {

struct Config {
    ts::Properties oldProperties;
    std::string data;
    vs::FileStat stat;

    Config(const ts::Properties &oldProperties, const std::string &data
           , const vs::FileStat &stat)
        : oldProperties(oldProperties), data(data), stat(stat)
    {}
};

Config convertConfig(const ts::Driver::pointer &driver)
{
    auto input(driver->input(vs::File::config));
    auto src(ts::loadConfig(input->get()));
    vts0::Properties dst;

    // convert
    dst.id = src.id;
    dst.srs = src.srs;
    dst.metaLevels = asVts(src.metaLevels);
    dst.extents
         = math::Extents2
        (src.alignment
         , math::Point2(src.alignment(0) + src.baseTileSize
                        , src.alignment(1) + src.baseTileSize));

    dst.textureQuality = src.textureQuality;
    dst.defaultPosition = src.defaultPosition;
    dst.defaultOrientation = src.defaultOrientation;
    dst.texelSize = src.texelSize;

    // templates
    dst.meshTemplate = "{lod}-{x}-{y}.bin";
    dst.textureTemplate = "{lod}-{x}-{y}.jpg";
    dst.metaTemplate = "{lod}-{x}-{y}.meta";

    std::ostringstream os;
    vts0::saveConfig(os, dst);

    return { src, os.str(), input->stat() };
}

class Ts2Vts0Driver : public DriverWrapper
{
public:
    Ts2Vts0Driver(const ts::Driver::pointer &driver)
        : driver_(driver), config_(convertConfig(driver))
        , metaTree_(driver, config_.oldProperties)
    {
        LOG(info3) << "VTS adapter in charge.";
    }

    virtual vs::Resources resources() const {
        return driver_->resources();
    }

    virtual bool externallyChanged() const {
        return driver_->externallyChanged();
    }

    virtual std::unique_ptr<Handle>
    openFile(LockGuard::OptionalMutex &mutex, const std::string &path
             , int flags, const tileset_Variables::Wrapper &variables);

private:
    ts::Driver::pointer driver_;
    Config config_;

    MetaTree metaTree_;
};

std::unique_ptr<Handle>
Ts2Vts0Driver::openFile(LockGuard::OptionalMutex &mutex
                        , const std::string &path, int flags
                        , const tileset_Variables::Wrapper &variables)
{
    Vts0FileInfo info(path, flags);

    LockGuard guard(mutex);

    switch (info.type) {
    case Vts0FileInfo::Type::file:
        if (info.file == vs::File::config) {
            // serve updated config
            return std::unique_ptr<Handle>
                (new GeneratedFileHandle
                 (config_.data, config_.stat, TILESET_FILETYPE_FILE));
        }
        return std::unique_ptr<Handle>
            (new StreamHandle
             (driver_->input(info.file), TILESET_FILETYPE_FILE));

    case Vts0FileInfo::Type::tileFile:
        if (info.tileFile == vs::TileFile::meta) {
            // metatile, check for virtual metatiles
            if (const auto *file = metaTree_.file(info.tileId)) {
                // we have virtual file, serve it
                return std::unique_ptr<Handle>
                    (new GeneratedFileHandle
                     (file->data, file->stat, TILESET_FILETYPE_TILE));
            }
        }

        // other tile files
        return std::unique_ptr<Handle>
            (new StreamHandle
             (driver_->input(asTs(config_.oldProperties, info.tileId)
                             , info.tileFile)
              , TILESET_FILETYPE_TILE));

    case Vts0FileInfo::Type::support:
        return std::unique_ptr<Handle>
            (new BrowserFileHandle(info.support->second, variables));

    default: break;
    }

    LOGTHROW(err1, vs::NoSuchFile)
        << "Unknown file to open: \"" << info.path << "\".";
    throw; // shut up, compiler
}

} // namespace

DriverWrapper::pointer openTs2Vts0(const std::string &path, int flags)
{
    (void) flags;
    return std::make_shared<Ts2Vts0Driver>
         (ts::Driver::open(path, ts::OpenMode::readOnly));
}
