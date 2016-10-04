#include "vts-libs/vts/support.hpp"
#include "vts-libs/vts.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/tileset/driver.hpp"
#include "vts-libs/vts/tileset/delivery.hpp"
#include "vts-libs/storage/fstreams.hpp"

#include "../vadstena-http.h"
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
    /** Request for raw file, not translation.
     */
    bool raw;

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
    const std::string Self(".");
    const std::string Index("index.html");

    // tileset inrernals
    namespace tileset {
        const std::string Config("tileset.conf");
        const std::string Index("tileset.index");
    }
}

VtsFileInfo::VtsFileInfo(const std::string &p, const LocationConfig &config
                         , int extraFlags)
    : FileInfo(p), raw(false), subTileFile(0), registry()
{
    if (vts::fromFilename(tileId, tileFile, subTileFile, path, 0, &raw)) {
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
            raw = true;
            return;
        }

        if (constants::tileset::Index == path) {
            type = Type::file;
            file = vs::File::tileIndex;
            return;
        }
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

    template <typename Source>
    Definition(const Source &source) {
        auto fl(vts::freeLayer(source.meshTilesConfig()));

        std::ostringstream os;
        vr::saveFreeLayer(os, fl);
        data = os.str();

        stat.lastModified = source.lastModified();
        stat.size = data.size();
        stat.contentType = vts::MeshTilesConfig::contentType;
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
};

#if 0
Handle::pointer tileFileStream(const VtsFileInfo &info
                               , const vs::IStream::pointer &is)
{
    switch (info.tileFile) {
    case vs::TileFile::mesh: {
        // read sub-file table from stream
        auto entry(vts::readMeshTable(*is, is->name())
                   [vts::Mesh::meshIndex()]);

        return Handle::pointer
            (new SubStreamHandle(is, TILESET_FILETYPE_TILE
                                 , entry.start, entry.size));
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

        return Handle::pointer
            (new SubStreamHandle(is, TILESET_FILETYPE_TILE
                                 , entry.start, entry.size));
    }

    case vs::TileFile::navtile: {
        // read sub-file table from stream
        auto entry(vts::NavTile::readTable(*is, is->name())
                   [vts::NavTile::imageIndex()]);

        return Handle::pointer
            (new SubStreamHandle(is, TILESET_FILETYPE_TILE
                                 , entry.start, entry.size));
    }

    default: break;
    }

    // default handler
    return std::unique_ptr<Handle>
        (new StreamHandle(is, TILESET_FILETYPE_TILE));
}
#endif

void VtsTileSet::handle(Sink sink, const std::string &path
                        , const LocationConfig &config)
{
    (void) sink;
    (void) path;
    (void) config;
    throw InternalError("Not implemented yet");
}

#if 0
Handle::pointer
VtsTileSet::openFile(LockGuard::OptionalMutex &mutex, const std::string &path
                     , int flags, const tileset_Variables::Wrapper &variables)
{
    // we want internals
    VtsFileInfo info(path, flags, ExtraFlags::enableTilesetInternals);

    LockGuard guard(mutex);

    switch (info.type) {
    case FileInfo::Type::definition:
        return Handle::pointer
            (new GeneratedFileHandle
             (definition_.data, definition_.stat, TILESET_FILETYPE_FILE));

    case FileInfo::Type::dirs:
        // serve updated dirs config
        return Handle::pointer
            (new GeneratedFileHandle
             (mapConfig_.dirsData, mapConfig_.dirsStat
              , TILESET_FILETYPE_FILE));

    case FileInfo::Type::file:
        if ((info.file == vs::File::config) && !info.raw) {
            // serve updated map config
            return Handle::pointer
                (new GeneratedFileHandle
                 (mapConfig_.data, mapConfig_.stat, TILESET_FILETYPE_FILE));
        }

        // serve internal file
        return Handle::pointer
            (new StreamHandle
             (delivery_->input(info.file), TILESET_FILETYPE_FILE));

    case FileInfo::Type::tileFile: {
        auto is(delivery_->input(info.tileId, info.tileFile));
        if (info.raw) {
            return Handle::pointer
                (new StreamHandle(is, TILESET_FILETYPE_FILE));
        }
        return tileFileStream(info, is);
    }

    case FileInfo::Type::support:
        return Handle::pointer
            (new BrowserFileHandle(info.support->second, variables));

    case FileInfo::Type::unknown:
        // unknown file, let's test other members
        if (info.registry) {
            // it's registry file!
            return Handle::pointer
                (new StreamHandle(vs::fileIStream(info.registry->contentType
                                                  , info.registry->path)
                                  , TILESET_FILETYPE_BROWSER));
        }
        break;

        // nothing appropriate
    default: break;
    }

    LOGTHROW(err1, vs::NoSuchFile)
        << "Unknown file to open: \"" << info.path << "\".";
    throw; // shut up, compiler
}
#endif

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

#if 0
    virtual std::unique_ptr<Handle>
    openFile(LockGuard::OptionalMutex&, const std::string &path, int flags
             , const tileset_Variables::Wrapper &variables)
    {
        VtsFileInfo info(path, flags);

        if (path == constants::Config) {
            return Handle::pointer
                (new GeneratedFileHandle
                 (mapConfig_.data, mapConfig_.stat, TILESET_FILETYPE_FILE));
        }

        if (path == constants::Dirs) {
            return Handle::pointer
                (new GeneratedFileHandle
                 (mapConfig_.dirsData, mapConfig_.dirsStat
                  , TILESET_FILETYPE_FILE));
        }

        // unknonw file, let's test other members
        if (info.registry) {
            // it's registry file!
            return Handle::pointer
                (new StreamHandle(vs::fileIStream(info.registry->contentType
                                                  , info.registry->path)
                                  , TILESET_FILETYPE_BROWSER));
        }

        switch (info.type) {
        case FileInfo::Type::support:
            if (path == constants::Self) {
                // enabled browser and asked to serve dir -> forced redirect
                // inside
                throw NoBody();
            }
            return Handle::pointer
                (new BrowserFileHandle(info.support->second, variables));

        case FileInfo::Type::unknown:
            // unknonw file, let's test other members
            if (info.registry) {
                // it's registry file!
                return Handle::pointer
                    (new StreamHandle(vs::fileIStream
                                      (info.registry->contentType
                                       , info.registry->path)
                                      , TILESET_FILETYPE_BROWSER));
            }
            break;

        default:
            break;
        }

        // unknown file
        LOGTHROW(err1, vs::NoSuchFile)
            << "Unknown file to open: \"" << info.path << "\".";
        throw; // shut up, compiler
    }
#endif

private:
    vts::Storage storage_;

    MapConfig mapConfig_;
};

void VtsStorage::handle(Sink sink, const std::string &path
                        , const LocationConfig &config)
{
    (void) sink;
    (void) path;
    (void) config;
    throw InternalError("Not implemented yet");
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
        sink.content(mapConfig_.data, fileinfo(mapConfig_.stat, -1));
        return;
    }

    if (path == constants::Dirs) {
        sink.content(mapConfig_.dirsData, fileinfo(mapConfig_.dirsStat, -1));
        return;
    }

    // unknonw file, let's test other members
    if (info.registry) {
        // it's registry file!
        sink.content(vs::fileIStream(info.registry->contentType
                                     , info.registry->path)
                     , FileClass::registry);
    }

#if 0

    if (info.support) {
        return Handle::pointer
            (new BrowserFileHandle(info.support->second, variables));
    }

    // unknown file
    LOGTHROW(err1, vs::NoSuchFile)
        << "Unknown file to open: \"" << info.path << "\".";
    throw; // shut up, compiler
#endif

    sink.error(utility::makeError<NotFound>("Unknown file."));
}

} // namespace

DriverWrapper::pointer openVts(const std::string &path, int)
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
