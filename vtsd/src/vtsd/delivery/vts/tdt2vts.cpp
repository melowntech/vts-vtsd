/**
 * Copyright (c) 2021 Melown Technologies SE
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

#include "utility/gzipper.hpp"

#include "vts-libs/vts/tileset/driver.hpp"

#include "3dtiles/3dtiles.hpp"
#include "3dtiles/mesh.hpp"

#include "support.hpp"
#include "tdt2vts.hpp"
#include "tdt2vts/support.hpp"
#include "tdt2vts/metabuilder.hpp"
#include "tdt2vts/constants.hpp"

namespace fs = boost::filesystem;
namespace vts = vtslibs::vts;
namespace vs = vtslibs::storage;
namespace vr = vtslibs::registry;

namespace tdt = threedtiles;

namespace vts2tdt {

namespace constants {
    const std::string Self(".");
    const std::string Index("index.html");
    const std::string Config("tileset.json");
}

struct FileInfo : ::FileInfo {
    /** tileId; valid only when (type == Type::tileFile)
     */
    vtslibs::vts::TileId tileId;

    /** Sub tile file. Used for textures in atlas.
     */
    unsigned int subTileFile = 0;

    FileInfo(const std::string &path, const LocationConfig &config);
};

FileInfo::FileInfo(const std::string &p, const LocationConfig &config)
    : ::FileInfo(p)
{
    boost::optional<unsigned int> stf;
    if (const char *ext = vts::parseTileIdPrefix(tileId, p, &stf)) {
        // parse extension
        if (constants::JsonExt == ext) {
            if (!stf) {
                type = Type::tileFile;
                tileFile = vts::TileFile::meta;
                return;
            }
        } else if (constants::B3dmExt == ext) {
            if (!stf) {
                type = Type::tileFile;
                tileFile = vts::TileFile::mesh;
                return;
            }
        } else if (constants::JpegExt == ext) {
            if (stf) { // sub-tilefile mandatory
                type = Type::tileFile;
                tileFile = vts::TileFile::atlas;
                subTileFile = *stf;
                return;
            }
        }
        return;
    }

    if (constants::Config == path) {
        type = Type::file;
        file = vs::File::config;
        return;
    }

    if (!config.enableBrowser) {
        LOG(debug) << "Browser disabled, skipping browser files.";
    } else {
        LOG(debug) << "Browser enabled, checking browser files.";

        // translate "." -> index
        if (constants::Self == path) { path = constants::Index; }

        // support files
        auto fsupport(supportFiles.find(path));
        if (fsupport != supportFiles.end()) {
            type = Type::support;
            support = &*fsupport;
            return;
        }
    }
}

void finishMeta(Sink &sink, const Location &location, MetaBuilder &mb)
{
    auto &delivery(mb.delivery());

    tdt::Tileset ts;
    if (!mb.run(ts)) {
        return sink.error(utility::makeError<NotFound>
                          ("No metanodes in this subtree."));
    }
    mb.send(sink, location, ts, vs::File::config, FileClass::data
            , delivery.lastModified());
}

void generateMeta(Sink &sink, const Location &location
                  , const ErrorHandler::pointer &errorHandler
                  , const vts::Delivery::pointer &delivery
                  , const vr::ReferenceFrame &referenceFrame
                  , const PerThreadConvertors::pointer &convertors
                  , const vts::TileId &rootId)
{
    // we need to load metatile pyramid starting at rootId

    // check if tileId is root of metanode pyramid
    if (rootId.lod % referenceFrame.metaBinaryOrder) {
        return sink.error(utility::makeError<NotFound>
                          ("Not a metanode pyramid root."));
    }

    if (!delivery->async()) {
        MetaBuilder mb(delivery, referenceFrame, convertors, rootId);
        mb.load();
        return finishMeta(sink, location, mb);
    }

    MetaBuilder::load_async(std::make_shared<MetaBuilder>
                            (delivery, referenceFrame, convertors, rootId)
                            , errorHandler
                            , [=](MetaBuilder &mb) mutable
    {
        try {
            finishMeta(sink, location, mb);
        } catch (...) {
            (*errorHandler)();
        }
    });
}

void finishTileset(Sink &sink, const Location &location
                   , const LocationConfig &config, MetaBuilder &mb)
{
    auto &delivery(mb.delivery());

    tdt::Tileset ts;
    if (!mb.run(ts, 0)) {
        return sink.error(utility::makeError<NotFound>
                          ("No metanodes in this subtree."));
    }

    // use revision as tileset version
    ts.asset.tilesetVersion
        = utility::format("%s", delivery.properties().revision);

    mb.send(sink, location, ts, vs::File::config, config.configClass
            , delivery.lastModified());
}

void generateTileset(Sink &sink, const Location &location
                     , const LocationConfig &config
                     , const ErrorHandler::pointer &errorHandler
                     , const vts::Delivery::pointer &delivery
                     , const vr::ReferenceFrame &referenceFrame
                     , const PerThreadConvertors::pointer &convertors)
{
    if (!delivery->async()) {
        MetaBuilder mb(delivery, referenceFrame, convertors, {}, false);
        mb.load(referenceFrame.division.rootLodRange.max);
        return finishTileset(sink, location, config, mb);
    }

    MetaBuilder::load_async(std::make_shared<MetaBuilder>
                            (delivery, referenceFrame, convertors
                             , vts::TileId(), false)
                            , errorHandler
                            , [=](MetaBuilder &mb) mutable
    {
        try {
            finishTileset(sink, location, config, mb);
        } catch (...) {
            (*errorHandler)();
        }
    }, referenceFrame.division.rootLodRange.max);
}

/** Image source.
 */
class ImageUriSource : public tdt::ImageSource {
public:
    ImageUriSource(const vts::TileId &tileId, const vts::Mesh &mesh)
        : tileId_(tileId), mesh_(mesh)
    {}

    gltf::Image image(int idx) const override {
        gltf::ExternalImage image;
        // use extra flag to not inline "local" image
        image.uri = ":" + uri(idx);
        return image;
    }

    std::string info(int idx) const override {
        return "uri:" + uri(idx);
    }

private:
    std::string uri(int idx) const {
        if ((idx < 0) || (idx > int(mesh_.submeshes.size()))) {
            return {};
        }

        const auto &sm(mesh_.submeshes[idx]);

        if (sm.textureMode != vts::SubMesh::TextureMode::internal) {
            // return custom surface URI
            // may be handled by the renderer in the future
            std::ostringstream os;
            os << "surface:" << (unsigned int)(sm.surfaceReference);
            return os.str();
        }

        return filename(tileId_, constants::JpegExt, idx);
    }

    vts::TileId tileId_;
    const vts::Mesh &mesh_;
};

void generateMesh(Sink &sink, const Location &location
                  , const vts::Delivery &delivery
                  , ErrorHandler::pointer errorHandler
                  , PerThreadConvertors::pointer ptc
                  , const vts::TileId &tileId)
{
    // run asynchronously
    delivery.driver()->input
        (tileId, vs::TileFile::mesh
         , [&, sink{std::move(sink)}, location
            , errorHandler{std::move(errorHandler)}
            , ptc{std::move(ptc)}]
         (const vts::EIStream &eis)
         mutable -> void
    {
        if (auto is = eis.get(*errorHandler)) {
            try {
                auto mesh(vts::loadMesh(is));

                // convert from physical system to destination, if different
                // than destination system
                const auto &fromPhys(ptc->get()());
                if (fromPhys) {
                    for (auto &sm : mesh) {
                        for (auto &v : sm.vertices) {
                            v = fromPhys(v);
                        }
                    }
                }

                // serialize, use external URIs to images
                auto io(std::make_shared<vs::StringIStream>
                        (vs::TileFile::mesh, location.path
                         , is->stat().lastModified));

                tdt::saveTile(utility::Gzipper(io->sink()), location.path
                              , tileId, vts::ConstSubMeshRange(mesh.submeshes)
                              , ImageUriSource(tileId, mesh));
                io->updateSize();

                return sink.content(io, FileClass::data, true);
            } catch (...) {
                (*errorHandler)();
            }
        }
    });
}

void generateAtlas(Sink &sink, const Location &location
                   , const ErrorHandler::pointer &errorHandler
                   , const vts::Delivery &delivery
                   , const FileInfo &fileInfo)
{
    // run asynchronously
    delivery.input(fileInfo.tileId, fileInfo.tileFile
                   , vts::FileFlavor::raw
                   , [=, errorHandler{std::move(errorHandler)}]
                   (const vts::EIStream &eis) mutable
    {
        if (auto is = eis.get(*errorHandler)) {
            try {
                tileFileStream(sink, location, fileInfo
                               , fileInfo.subTileFile, fileInfo.tileId
                               , std::move(is));
            } catch (...) {
                (*errorHandler)();
            }
        }
    });
}

} // namespace vts2tdt

Tdt2VtsTileSet::Tdt2VtsTileSet(const vts::Delivery::pointer &delivery)
    : delivery_(delivery)
    , referenceFrame_(vr::system.referenceFrames
                      (delivery_->properties().referenceFrame))
    , convertors_(std::make_shared<vts2tdt::PerThreadConvertors>
                  (referenceFrame_))
{
}

void Tdt2VtsTileSet::handle(Sink sink, const Location &location
                            , const LocationConfig &config
                            , const ErrorHandler::pointer &errorHandler)
{
    const vts2tdt::FileInfo info(location.path, config);

    try {
        switch (info.type) {
        case FileInfo::Type::file:
            if (info.file != vs::File::config) {
                return sink.error(utility::makeError<NotFound>
                                  ("Unknown file type."));
            }

            return vts2tdt::generateTileset(sink, location, config
                                            , errorHandler
                                            , delivery_, referenceFrame_
                                            , convertors_);

        case FileInfo::Type::tileFile:
            switch (info.tileFile) {
            case vs::TileFile::meta:
                return vts2tdt::generateMeta(sink, location, errorHandler
                                             , delivery_, referenceFrame_
                                             , convertors_, info.tileId);

            case vs::TileFile::mesh:
                return vts2tdt::generateMesh(sink, location, *delivery_
                                             , errorHandler
                                             , convertors_
                                             , info.tileId);

            case vs::TileFile::atlas:
                return vts2tdt::generateAtlas(sink, location, errorHandler
                                              , *delivery_, info);

            default: break;
            }

            return sink.error(utility::makeError<NotFound>
                              ("Unhandled file type."));

        case FileInfo::Type::support:
            sink.content(info.support->second);
            return;

        default: break;
        }

        // huh?
        sink.error(utility::makeError<NotFound>("Unknown file type."));
    } catch (...) {
        (*errorHandler)();
    }
}
