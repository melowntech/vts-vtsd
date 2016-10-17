#include <mutex>

#include "vts-libs/vts/support.hpp"
#include "vts-libs/vts.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/tileset/driver.hpp"
#include "vts-libs/vts/tileset/delivery.hpp"
#include "vts-libs/storage/fstreams.hpp"

#include "./driver.hpp"

namespace fs = boost::filesystem;
namespace vts = vadstena::vts;
namespace vs = vadstena::storage;
namespace vr = vadstena::registry;

namespace {

typedef std::map<std::string, boost::filesystem::path> RegistryFiles;

namespace ExtraFlags { enum {
    none = 0x0
    , enableTilesetInternals = 0x01
}; }

struct VtsFileInfo : public FileInfo {
    /** Distinguishes non-regular file from interpreted (i.e. tileset.conf from
     *  mapConfig.json)
     */
    vts::FileFlavor flavor;

    /** tileId; valid only when (type == Type::tileFile)
     */
    vadstena::vts::TileId tileId;

    /** Sub tile file. Used for textures in atlas.
     */
    unsigned int subTileFile;

    const vr::DataFile *registry;

    VtsFileInfo(const std::string &path, const LocationConfig &config
                , int extraFlags = ExtraFlags::none);
};

namespace constants {
    const std::string Config("mapConfig.json");
    const std::string Dirs("dirs.json");
    const std::string FreeLayerDefinition("freelayer.json");
    const std::string DebugConfig("debug.json");
    const std::string Self(".");
    const std::string Index("index.html");

    // tileset internals
    namespace tileset {
        const std::string Config("tileset.conf");
        const std::string Index("tileset.index");
        const std::string Registry("tileset.registry");
    }
}

VtsFileInfo::VtsFileInfo(const std::string &p, const LocationConfig &config
                         , int extraFlags)
    : FileInfo(p), flavor(vts::FileFlavor::regular), subTileFile(0), registry()
{
    if (vts::fromFilename(tileId, tileFile, subTileFile, path, 0, &flavor)) {
        type = Type::tileFile;
        return;
    }

    if (constants::Config == path) {
        type = Type::file;
        file = vs::File::config;
        return;
    }

    if (constants::FreeLayerDefinition == path) {
        type = Type::definition;
        return;
    }

    if (constants::Dirs == path) {
        type = Type::dirs;
        return;
    }

    if (!config.enableBrowser) {
        LOG(debug) << "Browser disabled, skipping browser files.";
    } else {
        LOG(debug) << "Browser enabled, checking browser files.";

        // translate "." -> index
        if (constants::Self == path) { path = constants::Index; }

        // support files
        auto fsupport(vts::supportFiles.find(path));
        if (fsupport != vts::supportFiles.end()) {
            type = Type::support;
            support = &*fsupport;
            return;
        }
    }

    // extra files, unknown to common machinery
    registry = vr::dataFile
        (path, vr::DataFile::Key::filename, std::nothrow);

    // internals are checked as last resort (the are requested only once in a
    // blue moon)
    if (extraFlags & ExtraFlags::enableTilesetInternals) {
        if (constants::tileset::Config == path) {
            type = Type::file;
            file = vs::File::config;
            // we need raw config file
            flavor = vts::FileFlavor::raw;
            return;
        }

        if (constants::tileset::Index == path) {
            type = Type::file;
            file = vs::File::tileIndex;
            return;
        }

        if (constants::tileset::Registry == path) {
            type = Type::file;
            file = vs::File::registry;
            return;
        }
    }

    // debug config file
    if (constants::DebugConfig == path) {
        type = Type::file;
        file = vs::File::config;
        flavor = vts::FileFlavor::debug;
        return;
    }
}

struct MapConfig {
    std::string data;
    vs::FileStat stat;

    std::string dirsData;
    vs::FileStat dirsStat;

    RegistryFiles registryFiles;

    template <typename Source>
    MapConfig(const Source &source) {
        // build map configuration
        auto mc(source.mapConfig());
        mc.srs.for_each([](vr::Srs &srs)
        {
            if (!srs.geoidGrid) { return; }
            // geoid grid -> use only filename in definition
            srs.geoidGrid->definition
                = fs::path(srs.geoidGrid->definition).filename().string();
        });

        // serialize map configuration
        {
            std::ostringstream os;
            saveMapConfig(mc, os);
            data = os.str();

            stat.lastModified = source.lastModified();
            stat.size = data.size();
            stat.contentType = vts::MapConfig::contentType;
        }

        // serialize dirs
        {
            std::ostringstream os;
            saveDirs(mc, os);
            dirsData = os.str();

            dirsStat.lastModified = source.lastModified();
            dirsStat.size = dirsData.size();
            dirsStat.contentType = vts::MapConfig::contentType;
        }

    }
};

struct Definition {
    std::string data;
    vs::FileStat stat;
    std::string debugData;
    vs::FileStat debugStat;

    template <typename Source>
    Definition(Source &source) {
        const auto mtc(source.meshTilesConfig());

        {
            const auto fl(vts::freeLayer(mtc));

            std::ostringstream os;
            vr::saveFreeLayer(os, fl);
            data = os.str();

            stat.lastModified = source.lastModified();
            stat.size = data.size();
            stat.contentType = vts::MeshTilesConfig::contentType;
        }

        {
            const auto dc(vts::debugConfig(mtc));

            std::ostringstream os;
            vts::saveDebug(os, dc);
            debugData = os.str();

            debugStat.lastModified = source.lastModified();
            debugStat.size = debugData.size();
            debugStat.contentType = vts::DebugConfig::contentType;
        }
    }
};

class VtsTileSet : public DriverWrapper
{
public:
    VtsTileSet(const std::string &path)
        : delivery_(vts::Delivery::open(path))
        , mapConfig_(*delivery_), definition_(*delivery_)
    {}

    virtual vs::Resources resources() const {
        return delivery_->resources();
    }

    virtual bool externallyChanged() const {
        return delivery_->externallyChanged();
    }

    virtual void handle(Sink sink, const std::string &path
                        , const LocationConfig &config);

private:
    vts::Delivery::pointer delivery_;
    MapConfig mapConfig_;
    Definition definition_;
    std::mutex mutex_;
};

void tileFileStream(Sink &sink, const VtsFileInfo &info
                    , const vs::IStream::pointer &is)
{
    switch (info.tileFile) {
    case vs::TileFile::mesh: {
        // read sub-file table from stream
        auto entry(vts::readMeshTable(*is, is->name())
                   [vts::Mesh::meshIndex()]);

        return sink.content(is, FileClass::data, entry.start, entry.size
                            , vs::gzipped(is, entry.start));
    }

    case vs::TileFile::atlas: {
        // read sub-file table from stream
        auto table(vts::Atlas::readTable(*is, is->name()));

        if (info.subTileFile >= table.size()) {
            LOGTHROW(err1, vs::NoSuchFile)
                << "Atlas index " << info.subTileFile
                << " out of range in file: \"" << info.path << "\".";
        }

        const auto &entry(table[info.subTileFile]);

        return sink.content(is, FileClass::data, entry.start, entry.size);
    }

    case vs::TileFile::navtile: {
        // read sub-file table from stream
        auto entry(vts::NavTile::readTable(*is, is->name())
                   [vts::NavTile::imageIndex()]);

        return sink.content(is, FileClass::data, entry.start, entry.size);
    }

    default: break;
    }

    // default handler
    return sink.content(is, FileClass::data);
}

void VtsTileSet::handle(Sink sink, const std::string &path
                        , const LocationConfig &config)
{
    // we want internals
    VtsFileInfo info(path, config, ExtraFlags::enableTilesetInternals);

    switch (info.type) {
    case FileInfo::Type::definition:
        return sink.content(definition_.data, fileinfo(definition_.stat, -1));

    case FileInfo::Type::dirs:
        return sink.content(mapConfig_.dirsData
                            , fileinfo(mapConfig_.dirsStat, -1));

    case FileInfo::Type::file:
        if (info.file == vs::File::config) {
            switch (info.flavor) {
            case vts::FileFlavor::regular:
                // serve updated map config
                return sink.content(mapConfig_.data
                                    , fileinfo(mapConfig_.stat, -1));

            case vts::FileFlavor::debug:
                return sink.content(definition_.debugData
                                    , fileinfo(definition_.debugStat, -1));
            default:
                // pass
                break;
            }
        }

        // serve internal (raw) file
        {
            std::unique_lock<std::mutex> guard(mutex_);
            return sink.content(delivery_->input(info.file), FileClass::data);
        }

    case FileInfo::Type::tileFile: {
        // get input stream (locked)
        auto is([&]() -> vs::IStream::pointer
        {
            std::unique_lock<std::mutex> guard(mutex_);
            return delivery_->input
                (info.tileId, info.tileFile, info.flavor);
        }());

        // raw data?
        if (info.flavor == vts::FileFlavor::raw) {
            return sink.content(is, FileClass::data);
        }

        // browser frendly
        return tileFileStream(sink, info, is);
    }

    case FileInfo::Type::support:
        sink.content(info.support->second);
        return;

    case FileInfo::Type::unknown:
        // unknown file, let's test other members
        if (info.registry) {
            // it's registry file!
            return  sink.content(vs::fileIStream(info.registry->contentType
                                                 , info.registry->path)
                                 , FileClass::registry);
        }
        break;

    default:
        // TODO: implement me
        break;
    }

    // wtf?
    sink.error(utility::makeError<NotFound>("Unknown file."));
}

class VtsStorage : public DriverWrapper
{
public:
    VtsStorage(vts::Storage &&storage)
        : storage_(std::move(storage)), mapConfig_(storage_)
    {}

    virtual vs::Resources resources() const {
        return storage_.resources();
    }

    virtual bool externallyChanged() const {
        return storage_.externallyChanged();
    }

    virtual void handle(Sink sink, const std::string &path
                        , const LocationConfig &config);

private:
    vts::Storage storage_;

    MapConfig mapConfig_;
};

void VtsStorage::handle(Sink sink, const std::string &path
                        , const LocationConfig &config)
{
    VtsFileInfo info(path, config);

    if (path == constants::Config) {
        return sink.content(mapConfig_.data, fileinfo(mapConfig_.stat, -1));
    }

    if (path == constants::Dirs) {
        return sink.content(mapConfig_.dirsData
                            , fileinfo(mapConfig_.dirsStat, -1));
    }

    // unknonw file, let's test other members
    if (info.registry) {
        // it's registry file!
        return sink.content(vs::fileIStream(info.registry->contentType
                                            , info.registry->path)
                            , FileClass::registry);
    }

    if (info.support) {
        if (path == constants::Self) {
            // enabled browser and asked to serve dir
            throw ListContent({ { "index.html" } });
        }

        // support file
        return sink.content(info.support->second);
    }

    // wtf?
    sink.error(utility::makeError<NotFound>("Unknown file."));
}

class VtsStorageView : public DriverWrapper
{
public:
    VtsStorageView(vts::StorageView &&storageView)
        : storageView_(std::move(storageView)), mapConfig_(storageView_)
    {}

    virtual vs::Resources resources() const {
        return storageView_.resources();
    }

    virtual bool externallyChanged() const {
        return storageView_.externallyChanged();
    }

    virtual void handle(Sink sink, const std::string &path
                        , const LocationConfig &config);

    /** We consider storage view a hot-content. I.e. it is not cached.
     */
    virtual bool hotContent() const { return true; }

private:
    vts::StorageView storageView_;

    MapConfig mapConfig_;
};

void VtsStorageView::handle(Sink sink, const std::string &path
                            , const LocationConfig &config)
{
    VtsFileInfo info(path, config);

    if (path == constants::Config) {
        return sink.content(mapConfig_.data, fileinfo(mapConfig_.stat, -1));
    }

    if (path == constants::Dirs) {
        return sink.content(mapConfig_.dirsData
                            , fileinfo(mapConfig_.dirsStat, -1));
    }

    // unknonw file, let's test other members
    if (info.registry) {
        // it's registry file!
        return sink.content(vs::fileIStream(info.registry->contentType
                                            , info.registry->path)
                            , FileClass::registry);
    }

    if (info.support) {
        // support file
        return sink.content(info.support->second);
    }

    // wtf?
    sink.error(utility::makeError<NotFound>("Unknown file."));
}

} // namespace

DriverWrapper::pointer openVts(const std::string &path)
{
    switch (vts::datasetType(path)) {
    case vts::DatasetType::TileSet:
        return std::make_shared<VtsTileSet>(path);

    case vts::DatasetType::Storage:
        return std::make_shared<VtsStorage>(vts::openStorage(path));

    case vts::DatasetType::StorageView:
        return std::make_shared<VtsStorageView>
            (vts::openStorageView(path));

    default:
        // no such tileset
        throw vs::NoSuchTileSet(path);
    }

    return {}; // never reached
}
