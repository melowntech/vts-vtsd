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

#include "vts-libs/vts/tileset/driver.hpp"

#include "3dtiles/3dtiles.hpp"
#include "3dtiles/mesh.hpp"

#include "support.hpp"
#include "tdt2vts.hpp"
#include "tdt2vts/support.hpp"

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

    const std::string JsonExt("json");
    const std::string B3dmExt("b3dm");
    const std::string JpegExt("jpg");
}

using FT = vts::TileIndex::Flag;

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

template <typename T>
vs::IStream::pointer serialize(const tdt::Tileset &ts
                               , const std::string &path
                               , T &&type
                               , std::time_t lastModified)
{
    auto is(std::make_shared<vs::StringIStream>(type, path, lastModified));
    tdt::write(is->sink(), ts);
    is->updateSize();
    return is;
}


vts::MetaTile loadMetaTile(const vts::TileId &tileId
                           , const vts::Delivery &delivery
                           , unsigned int metaBinaryOrder
                           , std::time_t *lastModified = nullptr)
{
    auto is(delivery.driver()->input(tileId, vts::TileFile::meta));
    if (lastModified) { *lastModified = is->stat().lastModified; }
    return vts::loadMetaTile(*is, metaBinaryOrder, is->name());
}

math::Extents3 regionExtents(const vts::NodeInfo &ni
                             , const vts::GeomExtents::ZRange &z
                             , const Convertors &convertors)
{
    const auto &conv(convertors(ni.srs()));

    // make 3D extents from SDS tile extents and geom extents Z-range
    auto e(math::extents3(ni.extents()));
    e.ll(2) = z.min;
    e.ur(2) = z.max;

    math::Extents3 region(math::InvalidExtents{});

    // conver all eight corners
    for (const auto &v : math::vertices(e)) {
        math::update(region, conv(v));
    }

    return region;
}

tdt::Region region(const vts::MetaNode &node
                   , const vts::NodeInfo &ni
                   , const Convertors &convertors)
{
    tdt::Region region;
    region.extents = regionExtents(ni, node.geomExtents.z, convertors);
    return region;
}

tdt::BoundingVolume boundingVolume(const vts::MetaNode &node
                                   , const vts::NodeInfo &ni
                                   , const Convertors &convertors)
{
    if (node.real()) {
        // real data, make region from SDS and geom extents
        return region(node, ni, convertors);
    }

    const auto &fromPhys(convertors());

    auto e(vr::denormalizedExtents(ni.referenceFrame(), node.extents));

    if (fromPhys) {
        // TODO: we have to convert bbox from physical SRS to destination SRS,
        // probably all 8 corners and then create new bounding box around them
        // in destination SRS
    }

    auto c(math::center(e));

    // make axis-aligned box (because we have no other way to make it)
    tdt::Box box;
    box.center = box.x = box.y = box.z = c;
    box.x(0) = e.ur(0) - c(0);
    box.y(1) = e.ur(1) - c(1);
    box.z(2) = e.ur(2) - c(2);

    return box;
}

inline std::string filename(const vts::TileId &tileId
                            , const std::string &ext
                            , const boost::optional<int> &sub = boost::none)
{
    std::ostringstream os;
    os << tileId.lod << '-' << tileId.x << '-' << tileId.y;
    if (sub) { os << '-' << *sub; }
    os << '.' << ext;
    return os.str();
}

struct MetaBuilder {
    MetaBuilder(Sink &sink, const Location &location
                , const vts::Delivery &delivery
                , const vr::ReferenceFrame &referenceFrame
                , const Convertors &convertors
                , const vts::TileId &rootId
                , unsigned int depth = 1024
                , bool optimizeBottom = true)
        : sink(sink), location(location)
        , delivery(delivery)
        , referenceFrame(referenceFrame)
        , convertors(convertors)
        , rootId(rootId)
        , mbo(referenceFrame.metaBinaryOrder)
        , ti(delivery.index())
        , lastModified()
        , optimizeBottom(optimizeBottom)
    {
        if (depth > mbo) { depth = mbo; }

        // 2) find stack of binary-order metatiles
        auto tid(rootId);
        for (unsigned int i = 0; i <= depth;
             ++i, tid = vts::lowestChild(tid))
        {
            const auto mid(vts::metaId(tid, mbo));
            if (!ti.meta(mid)) { break; }
            std::time_t lm;
            metas.push_back(loadMetaTile(mid, delivery, mbo, &lm));
            lastModified = std::max(lastModified, lm);
        }
    }

    void run(tdt::Tileset &&ts = {}) {
        if (metas.empty()) {
            return sink.error(utility::makeError<NotFound>
                              ("No metanodes in this subtree."));
        }

        // what version to use in subtilesets?
        ts.asset.tilesetVersion
            = utility::format("%s", delivery.properties().revision);

        ts.root = traverse(vts::NodeInfo(referenceFrame, rootId)
                               , metas.begin(), metas.end());
        // copy geometric error?
        ts.geometricError = ts.root->geometricError;

        return sink.content
            (serialize(ts, location.path, vs::File::config, lastModified)
             , FileClass::data);
    };

    tdt::Tile::pointer
    traverse(const vts::NodeInfo &ni
             , vts::MetaTile::list::const_iterator imeta
             , const vts::MetaTile::list::const_iterator &emeta)
    {
        const vts::TileId tileId(ni.nodeId());

        LOG(info1) << "Processing " << tileId
                   << " (" << imeta->origin() << "), SDS: "
                   << std::fixed << ni.extents();
        const auto node(imeta->get(tileId, std::nothrow));

        // ignore nonexistent node or invalid node
        if (!node || !node->flags()) { return {}; }

        // OK, we have tile
        auto tile(std::make_unique<tdt::Tile>());

        tile->geometricError = 1e6; // how can I know?

        tdt::Region br;

        if (node->real()) {
            br = region(*node, ni, convertors);
        }

        // move to next metatile
        std::advance(imeta, 1);

        const auto bottom(imeta == emeta);
        const auto hasChildren(node->childFlags());

        // should we use a link to other meta pyramid?
        bool link(bottom && hasChildren);
        if (bottom && !optimizeBottom && !hasChildren) {
            // special case if we should not optimize meta tree
            link = true;
        }

        if (node->real()) {
            /* TODO: make error factor configurable
             *
             * NB: we divide geometric error by 2 because we should use
             * geometric error from children
             */
            tile->geometricError = 16.0 * node->texelSize / 2.0;

            // real content
            if (!link) {
                tile->content.emplace();
                // either inside node or leaf at the bottom
                tile->content->uri = filename(tileId, constants::B3dmExt);
            }
        }

        if (link) {
            // non-real node at bottom with children
            tile->content.emplace();
            tile->content->uri = filename(tileId, constants::JsonExt);
        }

        // process children if not at the bottom
        if (!bottom) {
            for (const auto &child : vts::children(*node, tileId)) {
                tile->children.push_back
                    (traverse(ni.child(child), imeta, emeta));

                if (!node->real()) {
                    // accumulate children bounding volume's in non-real
                    // tiles
                    const auto &ctile(*tile->children.back());
                    if (auto *creg = boost::get<tdt::Region>
                        (&ctile.boundingVolume))
                        {
                            math::update(br.extents, creg->extents);
                        }
                }
            }
        } else if (!node->real()) {
            /* non-real tile at the bottom of the tree, we need to find real
             * nodes to guess much tighter geometry than the one we currenty
             * have available here
             *
             * use this tile's geometic extents Z-range since we are goint
             * to traverse tile index
             */

            // TODO: generate resolution as well
            const auto extents(measure(ni, node->geomExtents.z));

            // use subtree extents
            br.extents = extents;
        }

        if (!valid(br.extents)) {
            // last resort: use this node's geometric extents
            br = region(*node, ni, convertors);
        }

        // set bounding volume
        tile->boundingVolume = br;

        return tile;
    };

    math::Extents3
    measure(const vts::NodeInfo &ni
            , const vts::GeomExtents::ZRange &z)
    {
        // TODO: limit depth?

        if (!ni.valid()) { return math::Extents3(math::InvalidExtents{}); }

        const vts::TileId tileId(ni.nodeId());

        const auto flags(ti.tileIndex.get(tileId));

        // real tile -> use extents
        if (FT::isReal(flags)) {
            return regionExtents(ni, z, convertors);
        }

        math::Extents3 e(math::InvalidExtents{});

        // check if there are any children, i.e. subtree starting at this tile
        // ID must be valid in next lod's tree (yes, quadtree magic)
        if (!ti.tileIndex.validSubtree(tileId.lod + 1, tileId)) { return e; }

        // let's traverse children
        for (const auto &child : vts::children(tileId)) {
            auto ce(measure(ni.child(child), z));
            if (math::valid(ce)) {
                math::update(e, ce);
            }
        }

        return e;
    }

    Sink &sink;
    const Location &location;
    const vts::Delivery &delivery;
    const vr::ReferenceFrame &referenceFrame;
    const Convertors &convertors;
    const vts::TileId &rootId;

    const unsigned int mbo;
    const vts::tileset::Index &ti;

    std::time_t lastModified;
    bool optimizeBottom;

    vts::MetaTile::list metas;
};

void generateMeta(Sink &sink, const Location &location
                  , const vts::Delivery &delivery
                  , const vr::ReferenceFrame &referenceFrame
                  , const Convertors &convertors
                  , const vts::TileId &rootId)
{
    // we need to load metatile pyramid starting at rootId

    // 1) check if tileId is root of metanode pyramid
    if (rootId.lod % referenceFrame.metaBinaryOrder) {
        return sink.error(utility::makeError<NotFound>
                          ("Not a metanode pyramid root."));
    }


    return MetaBuilder(sink, location, delivery, referenceFrame
                       , convertors, rootId).run();
}

void generateTileset(Sink &sink, const Location &location
                     , const vts::Delivery &delivery
                     , const vr::ReferenceFrame &referenceFrame
                     , const Convertors &convertors)
{
    const auto &props(delivery.properties());

    tdt::Tileset ts;
    // use revision as tileset version
    ts.asset.tilesetVersion = utility::format("%s", props.revision);

    return MetaBuilder(sink, location, delivery, referenceFrame
                       , convertors, {}, 0, false).run(std::move(ts));
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
            os << "surface:" << sm.surfaceReference;
            return os.str();
        }

        return filename(tileId_, constants::JpegExt, idx);
    }

    vts::TileId tileId_;
    const vts::Mesh &mesh_;
};

void generateMesh(Sink &sink, const Location &location
                  , const vts::Delivery &delivery
                  , const ErrorHandler::pointer &errorHandler
                  , const vts::CsConvertor &fromPhys
                  , const vts::TileId &tileId)
{
    // run asynchronously
    delivery.driver()->input
        (tileId, vs::TileFile::mesh
         , [&, sink, location](const vts::EIStream &eis)
         mutable -> void
    {
        if (auto is = eis.get(*errorHandler)) {
            try {
                auto mesh(vts::loadMesh(is));

                // convert from physical system to destination, if different
                if (fromPhys) {
                    for (auto &sm : mesh) {
                        for (auto &v : sm.vertices) { v = fromPhys(v); }
                    }
                }

                // serialize, use external URIs to images
                auto io(std::make_shared<vs::StringIStream>
                        (vs::TileFile::mesh, location.path
                         , is->stat().lastModified));

                tdt::saveTile(io->sink(), location.path
                              , tileId, vts::ConstSubMeshRange(mesh.submeshes)
                              , ImageUriSource(tileId, mesh));
                io->updateSize();

                return sink.content(io, FileClass::data);
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
                   , [=](const vts::EIStream &eis) mutable -> void
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

vts::CsConvertor makePhysConv(const vr::ReferenceFrame &rf)
{
    const auto &phys(rf.model.physicalSrs);

    const auto &srs(vr::system.srs.get(rf.model.physicalSrs));

    if (srs.type == vr::Srs::Type::cartesian) {
        // should be geocent -> no conversion
        return {};
    }

    return vts::CsConvertor(phys, geo::geocentric(srs.srsDef));
}

} // namespace vts2tdt

Tdt2VtsTileSet::Tdt2VtsTileSet(const vts::Delivery::pointer &delivery)
    : delivery_(delivery)
    , referenceFrame_(vr::system.referenceFrames
                      (delivery_->properties().referenceFrame))
    , convertors_(referenceFrame_)
{
}

void Tdt2VtsTileSet::handle(Sink sink, const Location &location
                            , const LocationConfig &config
                            , const ErrorHandler::pointer &errorHandler)
{
    const vts2tdt::FileInfo info(location.path, config);

    // make sure we have local CS convertors available
    const auto convertors(convertors_.get());

    try {
        switch (info.type) {
        case FileInfo::Type::file:
            if (info.file != vs::File::config) {
                return sink.error(utility::makeError<NotFound>
                                  ("Unknown file type."));
            }

            return vts2tdt::generateTileset(sink, location, *delivery_
                                            , referenceFrame_
                                            , convertors);

        case FileInfo::Type::tileFile:
            switch (info.tileFile) {
            case vs::TileFile::meta:
                return vts2tdt::generateMeta(sink, location, *delivery_
                                             , referenceFrame_
                                             , convertors, info.tileId);

            case vs::TileFile::mesh:
                return vts2tdt::generateMesh(sink, location, *delivery_
                                             , errorHandler
                                             , convertors()
                                             , info.tileId);

            case vs::TileFile::atlas:
                return vts2tdt::generateAtlas(sink, location, errorHandler
                                              , *delivery_, info);

            default: break;
            }

            return sink.error(utility::makeError<InternalError>
                              ("TODO: implement me"));

        case FileInfo::Type::support:
            sink.content(info.support->second);
            return;

        default: break;
        }

        // wtf?
        sink.error(utility::makeError<NotFound>("Unknown file type."));
    } catch (...) {
        (*errorHandler)();
    }
}
