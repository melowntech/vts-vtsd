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

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "imgproc/png.hpp"

#include "vts-libs/vts/support.hpp"
#include "vts-libs/vts.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/tileset/driver.hpp"
#include "vts-libs/vts/tileset/delivery.hpp"
#include "vts-libs/storage/fstreams.hpp"
#include "vts-libs/vts/2d.hpp"
#include "vts-libs/vts/debug.hpp"
#include "vts-libs/vts/virtualsurface.hpp"
#include "vts-libs/vts/service.hpp"

#include "./driver.hpp"
#include "./mapconfig.hpp"

namespace fs = boost::filesystem;
namespace vts = vtslibs::vts;
namespace vs = vtslibs::storage;
namespace vr = vtslibs::registry;
namespace mc = mapconfig;

namespace {

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
    vtslibs::vts::TileId tileId;

    /** Sub tile file. Used for textures in atlas.
     */
    unsigned int subTileFile;

    /** Registry file if non-null.
     */
    const vr::DataFile *registry;

    /** Service file if non-zero
     */
    unsigned int service;

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
    , service()
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

    if (vts::VirtualSurface::TilesetMappingPath == path) {
        type = Type::tilesetMapping;
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

    if ((service = vts::service::match(path))) {
        type = Type::unknown;
        return;
    }
}

class VtsTileSet : public DriverWrapper
{
public:
    VtsTileSet(const std::string &path
               , const vtslibs::vts::OpenOptions &openOptions)
        : delivery_(vts::Delivery::open(path, openOptions))
    {}

    VtsTileSet(std::shared_ptr<vts::Driver> driver)
        : delivery_(vts::Delivery::open(std::move(driver)))
    {}

    virtual vs::Resources resources() const {
        return delivery_->resources();
    }

    virtual bool externallyChanged() const {
        return delivery_->externallyChanged();
    }

    virtual void handle(Sink sink, const Location &location
                        , const LocationConfig &config);

    static std::shared_ptr<vts::Driver>
    asDriver(const DriverWrapper::pointer &driver);

private:
    const mc::MapConfig& mapConfig() { return mapConfig_(*delivery_); }
    const mc::Definition& definition() { return definition_(*delivery_); }

    vts::Delivery::pointer delivery_;
    mc::LazyConfigHolder<mc::MapConfig> mapConfig_;
    mc::LazyConfigHolder<mc::Definition> definition_;
};

void tileFileStream(Sink &sink, const Location &location
                    , const VtsFileInfo &info
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

        // find first entry referencing the same file and redirect if different
        // than info.subTileFile
        std::size_t index(info.subTileFile);
        while (index && (table[index - 1] == entry)) { --index; }

        if (index != info.subTileFile) {
            // build redirect file path
            auto fp(vts::filePath(info.tileFile, info.tileId, index));
            // append query if present
            if (!location.query.empty()) {
                fp.push_back('?');
                fp.append(location.query);
            }

            return sink.redirect
                (fp, utility::HttpCode::Found, FileClass::data);
        }

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

void VtsTileSet::handle(Sink sink, const Location &location
                        , const LocationConfig &config)
{
    // we want internals
    VtsFileInfo info(location.path, config
                     , ExtraFlags::enableTilesetInternals);

    const auto sendRawFile([&](FileClass fc) -> void
    {
        return sink.content(delivery_->input(info.file), fc);
    });

    switch (info.type) {
    case FileInfo::Type::definition:
        return definition().def.send(sink, config.configClass);

    case FileInfo::Type::dirs:
        return mapConfig().dirs.send(sink, config.configClass);

    case FileInfo::Type::file:
        if (info.file == vs::File::config) {
            switch (info.flavor) {
            case vts::FileFlavor::regular:
                // serve updated map config
                return mapConfig().send(sink, config.configClass);

            case vts::FileFlavor::debug:
                return definition().debug.send(sink, config.configClass);

            case vts::FileFlavor::raw:
                // tileset.conf is always ephemeral
                return sendRawFile(FileClass::ephemeral);

            default:
                // pass
                break;
            }
        } else if (info.file == vs::File::registry) {
            // tileset.registry is always ephemeral
            return sendRawFile(FileClass::ephemeral);
        }

        // serve internal (raw) file
        return sendRawFile(FileClass::data);

    case FileInfo::Type::tileFile: {
        auto is(delivery_->input(info.tileId, info.tileFile, info.flavor));

        // raw data?
        if (info.flavor == vts::FileFlavor::raw) {
            return sink.content(is, FileClass::data);
        }

        // browser frendly
        return tileFileStream(sink, location, info, is);
    }

    case FileInfo::Type::support:
        sink.content(info.support->second);
        return;

    case FileInfo::Type::tilesetMapping: {
        // get input stream (locked)
        return sink.content
            (delivery_->input(vts::VirtualSurface::TilesetMappingPath)
             , FileClass::data);
    }

    case FileInfo::Type::unknown:
        // unknown file, let's test other members
        if (info.registry) {
            // it's registry file!
            return sink.content(vs::fileIStream(info.registry->contentType
                                                , info.registry->path)
                                , FileClass::registry);
        }

        if (info.service) {
            return sink.content(vts::service::generate
                                (info.service, info.path, location.query)
                                , FileClass::data);
        }
        break;

    default:
        break;
    }

    // wtf?
    sink.error(utility::makeError<NotFound>("Unknown file."));
}

std::shared_ptr<vts::Driver>
VtsTileSet::asDriver(const DriverWrapper::pointer &driver)
{
    if (auto d = dynamic_cast<VtsTileSet*>(driver.get())) {
        return d->delivery_->driver();
    }
    LOGTHROW(err1, vs::NoSuchTileSet)
        << "Not a vts tileset.";
    throw;
}

class VtsStorage : public DriverWrapper
{
public:
    VtsStorage(vts::Storage &&storage, bool proxiesAllowed)
        : storage_(std::move(storage))
        , proxiesAllowed_(proxiesAllowed)
    {}

    virtual vs::Resources resources() const {
        return storage_.resources();
    }

    virtual bool externallyChanged() const {
        return storage_.externallyChanged();
    }

    virtual void handle(Sink sink, const Location &location
                        , const LocationConfig &config);

    static vts::Storage asStorage(const DriverWrapper::pointer &driver);

private:
    const mc::PotentiallyProxiedMapConfig& mapConfig() {
        return mapConfig_(storage_, proxiesAllowed_);
    }

    vts::Storage storage_;
    bool proxiesAllowed_;
    mc::PotentiallyProxiedMapConfig::Lazy mapConfig_;
};

vts::Storage VtsStorage::asStorage(const DriverWrapper::pointer &driver)
{
    if (auto d = dynamic_cast<VtsStorage*>(driver.get())) {
        return d->storage_;
    }
    LOGTHROW(err1, vs::NoSuchTileSet)
        << "Not a vts storage.";
    throw;
}


void VtsStorage::handle(Sink sink, const Location &location
                        , const LocationConfig &config)
{
    VtsFileInfo info(location.path, config);

    if (location.path == constants::Config) {
        return mapConfig().sendMapConfig
            (sink, location.proxy, config.configClass);
    }

    if (location.path == constants::Dirs) {
        return mapConfig().sendDirs
            (sink, location.proxy, config.configClass);
    }

    // unknonw file, let's test other members
    if (info.registry) {
        // it's registry file!
        return sink.content(vs::fileIStream(info.registry->contentType
                                            , info.registry->path)
                            , FileClass::registry);
    }

    if (info.support) {
        if (location.path == constants::Self) {
            // enabled browser and asked to serve dir
            throw ListContent({ { "index.html" } });
        }

        // support file
        return sink.content(info.support->second);
    }

    if (info.service) {
        return sink.content(vts::service::generate
                            (info.service, info.path, location.query)
                            , FileClass::data);
    }

    // wtf?
    sink.error(utility::makeError<NotFound>("Unknown file."));
}

class VtsStorageView : public DriverWrapper
{
public:
    VtsStorageView(vts::StorageView storageView, bool proxiesAllowed)
        : storageView_(std::move(storageView))
        , mapConfig_(mc::PotentiallyProxiedMapConfig::factory
                     (storageView_, proxiesAllowed))
    {}

    virtual vs::Resources resources() const {
        return storageView_.resources();
    }

    virtual bool externallyChanged() const {
        return storageView_.externallyChanged();
    }

    virtual void handle(Sink sink, const Location &location
                        , const LocationConfig &config);

    /** We consider storage view a hot-content. I.e. it is always watched for
     *  change.
     */
    virtual bool hotContent() const { return true; }

private:
    vts::StorageView storageView_;
    mc::PotentiallyProxiedMapConfig::pointer mapConfig_;
};

void VtsStorageView::handle(Sink sink, const Location &location
                            , const LocationConfig &config)
{
    VtsFileInfo info(location.path, config);

    if (location.path == constants::Config) {
        return mapConfig_->sendMapConfig(sink, location.proxy
                                         , config.configClass);
    }

    if (location.path == constants::Dirs) {
        return mapConfig_->sendDirs(sink, location.proxy, config.configClass);
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

    if (info.service) {
        return sink.content(vts::service::generate
                            (info.service, info.path, location.query)
                            , FileClass::data);
    }

    // wtf?
    sink.error(utility::makeError<NotFound>("Unknown file."));
}

class VtsTileIndex : public DriverWrapper
{
public:
    VtsTileIndex(const fs::path &path)
        : path_(path), stat_(vs::FileStat::stat(path))
    {
        // load tile index
        ti_.load(path);

        // and generate debug data

        vts::DebugConfig dc;
        {
            dc.meta = vts::fileTemplate(vs::TileFile::meta
                                        , vts::FileFlavor::debug
                                        , stat_.lastModified);
            dc.mask = vts::fileTemplate(vs::TileFile::mask
                                        , vts::FileFlavor::debug
                                        , stat_.lastModified);
            const auto ranges(ti_.ranges(vts::TileIndex::Flag::mesh));
            dc.lodRange = ranges.first;
            dc.tileRange = ranges.second;
        }

        std::ostringstream os;
        vts::saveDebug(os, dc);
        debugData_ = os.str();

        debugStat_ = stat_;
        debugStat_.contentType = vs::contentType(vs::File::config);

        maskStat_ = stat_;
        maskStat_.contentType = vs::contentType(vs::TileFile::mask);
    }

    virtual vs::Resources resources() const {
        return {};
    }

    virtual bool externallyChanged() const {
        return stat_.changed(vs::FileStat::stat(path_));
    }

    virtual void handle(Sink sink, const Location &location
                        , const LocationConfig &config);

    virtual bool hotContent() const { return false; }

private:
    const fs::path path_;
    vts::TileIndex ti_;
    vs::FileStat stat_;
    std::string debugData_;
    vs::FileStat debugStat_;
    vs::FileStat maskStat_;
};

const auto emptyDebugMask([]() -> std::vector<char>
{
    return imgproc::png::serialize(vts::emptyDebugMask(), 9);
}());

const auto fullDebugMask([]() -> std::vector<char>
{
    return imgproc::png::serialize(vts::fullDebugMask(), 9);
}());

void VtsTileIndex::handle(Sink sink, const Location &location
                          , const LocationConfig &config)
{
    VtsFileInfo info(location.path, config);

    switch (info.type) {
    case FileInfo::Type::file:
        if (info.flavor == vts::FileFlavor::debug) {
            switch (info.file) {
            case vs::File::config:
                return sink.content
                    (debugData_, fileinfo(debugStat_, FileClass::config));

            default: break;
            }
        }
        break;

    case FileInfo::Type::tileFile:
        if (info.flavor == vts::FileFlavor::debug) {
            switch (info.tileFile) {
            case vs::TileFile::meta: {
                std::ostringstream os;
                vts::saveDebug
                    (os, vts::getNodeDebugInfo(ti_, info.tileId));
                return sink.content
                    (os.str(), fileinfo(debugStat_, FileClass::data));
            }

            case vs::TileFile::mask:
                return sink.content
                    ((ti_.checkMask(info.tileId, vts::TileIndex::Flag::real)
                      ? fullDebugMask : emptyDebugMask)
                     , fileinfo(maskStat_, FileClass::data));

            default: break;
            }
        }
        break;

    default:
        break;
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

template <typename CallbackType>
void asyncOpenStorage(const fs::path &path, DeliveryCache &cache
                      , CallbackType &callback, bool forcedReopen)
{
    cache.get(path.c_str(), [callback](const DeliveryCache::Expected &value)
    {
        try {
            callback->done(std::move(VtsStorage::asStorage(value.get())));
        } catch (...) {
            callback->error(std::current_exception());
        }
    }, forcedReopen);
}

DriverWrapper::pointer
openStorageView(const std::string &path
                , DeliveryCache &cache, bool forcedReopen
                , bool proxiesAllowed
                , const DeliveryCache::Callback &callback)
{
    struct StorageViewOpenCallback : vts::StorageViewOpenCallback {
        StorageViewOpenCallback(DeliveryCache &cache
                                , const DeliveryCache::Callback &callback
                                , bool forcedReopen
                                , bool proxiesAllowed)
            : cache(cache), callback(callback), forcedReopen(forcedReopen)
            , proxiesAllowed(proxiesAllowed)
        {}

        virtual void error(const std::exception_ptr &exc) {
            callback(exc);
        }

        virtual void done(vts::StorageView storageView) {
            callback(DriverWrapper::pointer
                     (std::make_shared<VtsStorageView>
                      (std::move(storageView), proxiesAllowed)));
        }

        virtual void openStorage(const boost::filesystem::path &path
                                 , const vts::StorageOpenCallback::pointer
                                 &storageOpenCallback)
        {
            asyncOpenStorage(path, cache, storageOpenCallback, forcedReopen);
        }

        DeliveryCache &cache;
        const DeliveryCache::Callback callback;
        bool forcedReopen;
        bool proxiesAllowed;
    };

    // async open
    vts::openStorageView(path, std::make_shared<StorageViewOpenCallback>
                         (cache, callback, forcedReopen, proxiesAllowed));

    // nothing available so far
    return {};
}

DriverWrapper::pointer
openTileSet(const std::string &path
            , DeliveryCache &cache, const OpenOptions &openOptions
            , const DeliveryCache::Callback &callback)
{
    struct DriverOpenCallback : vts::DriverOpenCallback {
        DriverOpenCallback(DeliveryCache &cache
                           , const DeliveryCache::Callback &callback
                           , bool forcedReopen)
            : cache(cache), callback(callback), forcedReopen(forcedReopen)
        {}

        virtual void error(const std::exception_ptr &exc) {
            callback(exc);
        }

        virtual void done(std::shared_ptr<vts::Driver> driver) {
            callback(DriverWrapper::pointer
                     (std::make_shared<VtsTileSet>(std::move(driver))));
        }

        virtual void openStorage(const boost::filesystem::path &path
                                 , const vts::StorageOpenCallback::pointer
                                 &storageOpenCallback)
        {
            asyncOpenStorage(path, cache, storageOpenCallback, forcedReopen);
        }

        // NB: open options are ignore since we have our own
        virtual void openDriver(const boost::filesystem::path &path
                                , const vts::OpenOptions&
                                , const DriverOpenCallback::pointer
                                &driverOpenCallback)
        {
            cache.get(path.c_str(), [this, driverOpenCallback]
                      (const DeliveryCache::Expected &value)
            {
                try {
                    driverOpenCallback->done
                        (std::move(VtsTileSet::asDriver(value.get())));
                } catch (...) {
                    driverOpenCallback->error(std::current_exception());
                }
            }, forcedReopen);
        }

        DeliveryCache &cache;
        const DeliveryCache::Callback callback;
        bool forcedReopen;
    };

    // async open
    vts::openTilesetDriver(path, openOptions.openOptions
                           , std::make_shared<DriverOpenCallback>
                           (cache, callback, openOptions.forcedReopen));

    // nothing available so far
    return {};
}

} // namespace

DriverWrapper::pointer openVts(const std::string &path
                               , const OpenOptions &openOptions
                               , DeliveryCache &cache
                               , const DeliveryCache::Callback &callback
                               , bool proxiesAllowed)
{
    switch (vts::datasetType(path)) {
    case vts::DatasetType::TileSet:
        return openTileSet(path, cache, openOptions, callback);

    case vts::DatasetType::Storage:
        // TODO: make async as well
        return std::make_shared<VtsStorage>
            (vts::openStorage(path), proxiesAllowed);

    case vts::DatasetType::StorageView:
        return openStorageView
            (path, cache, openOptions.forcedReopen, proxiesAllowed
             , callback);

    case vts::DatasetType::TileIndex:
        return std::make_shared<VtsTileIndex>(path);

    default:
        // no such tileset
        throw vs::NoSuchTileSet(path);
    }

    return {}; // never reached
}
