/**
 * Copyright (c) 2021 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the conditions are met:
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

#include "3dtiles/3dtiles.hpp"

#include "vts-libs/vts/io.hpp"

#include "../../driver.hpp"

#include "support.hpp"
#include "constants.hpp"
#include "metabuilder.hpp"

namespace vts = vtslibs::vts;
namespace vs = vtslibs::storage;
namespace vr = vtslibs::registry;

namespace tdt = threedtiles;

namespace vts2tdt {

namespace {

using FT = vts::TileIndex::Flag;

math::Extents3 asRegionExtents(const vts::CsConvertor &conv
                               , const math::Extents3 &e)
{
    math::Extents3 region(math::InvalidExtents{});

    // conver all eight corners
    // TODO: use more vertices
    for (const auto &v : math::vertices(e)) {
        math::update(region, conv(v));
    }

    return region;
}

math::Extents3 regionExtents(const vts::NodeInfo &ni
                             , const vts::GeomExtents::ZRange &z
                             , const Convertors &convertors)
{
    // make 3D extents from SDS tile extents and geom extents Z-range
    auto e(math::extents3(ni.extents()));
    e.ll(2) = z.min;
    e.ur(2) = z.max;

    return asRegionExtents(convertors(ni.srs()), e);
}

math::Extents3 regionExtents(const vts::NodeInfo &ni
                             , const vts::GeomExtents &ge
                             , const Convertors &convertors)
{
    // make 3D extents from SDS tile extents and geom extents Z-range
    const math::Extents3 e(ge.extents.ll(0), ge.extents.ll(1), ge.z.min
                           , ge.extents.ur(0), ge.extents.ur(1), ge.z.max);

    return asRegionExtents(convertors(ni.srs()), e);
}

tdt::Region region(const vts::MetaNode &node
                   , const vts::NodeInfo &ni
                   , const Convertors &convertors)
{
    tdt::Region region;
    if (vts::complete(node.geomExtents)) {
        region.extents = regionExtents(ni, node.geomExtents, convertors);
    } else {
        region.extents = regionExtents(ni, node.geomExtents.z, convertors);
    }
    return region;
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

struct AsyncMeta {
    vts::MetaTile meta;
    std::time_t lastModified;

    AsyncMeta(vts::MetaTile meta, std::time_t lastModified)
        : meta(std::move(meta)), lastModified(lastModified)
    {}

    using callback = std::function<void(AsyncMeta)>;
};

void loadMetaTile(const vts::TileId &tileId, const vts::Delivery &delivery
                  , unsigned int metaBinaryOrder
                  , ErrorHandler::pointer errorHandler
                  , AsyncMeta::callback cb)
{
    delivery.input(tileId, vts::TileFile::meta, vts::FileFlavor::raw
                   , [=, errorHandler{std::move(errorHandler)}
                      , cb{std::move(cb)}]
                   (const vts::EIStream &eis) mutable
    {
        if (auto is = eis.get(*errorHandler)) {
            try {
                cb(AsyncMeta
                   (vts::loadMetaTile(*is, metaBinaryOrder, is->name())
                    , is->stat().lastModified));
            } catch (...) {
                (*errorHandler)();
            }
        }
    });
}

} // namespace

void MetaBuilder::load(int depth)
{
    const auto mbo(referenceFrame_.metaBinaryOrder);
    if (depth < 0) { depth = mbo; }

    // find stack of binary-order metatiles
    auto tid(rootId_);

    for (int i = 0; i <= depth; ++i, tid = vts::lowestChild(tid)) {
        const auto mid(vts::metaId(tid, mbo));
        if (!ti_.meta(mid)) { break; }
        std::time_t lm;
        metas_.push_back(loadMetaTile(mid, *delivery_, mbo, &lm));
        lastModified_ = std::max(lastModified_, lm);
    }
}

void MetaBuilder::load_async(const pointer &self
                             , const ErrorHandler::pointer &errorHandler
                             , const CompletionHandler &cb
                             , int depth)
{
    if (depth < 0) { depth = self->referenceFrame_.metaBinaryOrder; }

    // find stack of binary-order metatiles
    loadNext(self, errorHandler, cb, self->rootId_, depth);
}

void MetaBuilder::loadNext(pointer self
                           , const ErrorHandler::pointer &errorHandler
                           , CompletionHandler cb
                           , const vts::TileId &tileId, int left)
{
    const auto mbo(self->referenceFrame_.metaBinaryOrder);
    const auto metaId(vts::metaId(tileId, mbo));
    if (!self->ti_.meta(metaId)) { return cb(*self); }

    // capture "delivery" because "self" is moved into lambda
    auto &delivery(*self->delivery_);

    // run asynchronously
    loadMetaTile(metaId, delivery, mbo, errorHandler
                 , [=, self{std::move(self)}, cb{std::move(cb)}]
                 (AsyncMeta ameta)
    {
        self->lastModified_
            = std::max(ameta.lastModified, self->lastModified_);
        self->metas_.push_back(std::move(ameta.meta));

        if (!left) { cb(*self); return; }

        loadNext(std::move(self), errorHandler, std::move(cb)
                 , vts::lowestChild(tileId), left - 1);
    });
}

namespace {

inline bool extentsSource(const vts::MetaNode &node) {
    return node.real() || vts::complete(node.geomExtents);
}

inline bool hasFullExtents(const vts::MetaNode &node) {
    return vts::complete(node.geomExtents);
}

class Helper {
public:
    Helper(const vts::TileIndex &ti, bool optimizeBottom
           , int clipDepth = -1)
        : ti_(ti), optimizeBottom_(optimizeBottom)
        , clipDepth_(clipDepth)
    {}

    tdt::Tile::pointer
    traverse(const Convertors &convertors, const vts::NodeInfo &ni
             , vtslibs::vts::MetaTile::list metas)
    {
        return traverse(convertors, ni, metas.begin(), metas.end(), 0);
    }

private:
    tdt::Tile::pointer
    traverse(const Convertors &convertors, const vts::NodeInfo &ni
             , vts::MetaTile::list::const_iterator imeta
             , const vts::MetaTile::list::const_iterator &emeta
             , int depth)
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

        // first, use only full geometric extents
        const auto hfe(hasFullExtents(*node));
        if (hfe) { br = region(*node, ni, convertors); }

        // move to next metatile
        std::advance(imeta, 1);

        const auto bottom(imeta == emeta);
        const auto hasChildren(node->childFlags());
        const auto clip(depth == clipDepth_);

        // should we use a link to other meta pyramid?
        bool link((bottom && hasChildren) || clip);
        if (bottom && !optimizeBottom_ && !hasChildren) {
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
                    (traverse(convertors, ni.child(child), imeta, emeta
                              , depth + 1));

                if (!hfe) {
                    // accumulate children bounding volumes if this tile does
                    // not have full extents
                    const auto &ctile(*tile->children.back());
                    if (auto *creg = boost::get<tdt::Region>
                        (&ctile.boundingVolume))
                    {
                        math::update(br.extents, creg->extents);
                    }
                }
            }
        } else if (!hfe && !node->real()) {
            /* bottom tile with neither full extents nor any content, we need to
             * find real nodes to guess much tighter geometry than the one we
             * currenty have available here
             *
             * use this tile's geometic extents Z-range since we are going
             * to traverse tile index
             */

            // TODO: generate resolution as well
            const auto extents(measure(convertors, ni, node->geomExtents.z));

            // use subtree extents
            br.extents = extents;
        }

        if (!valid(br.extents)) {
            // last resort: use this node's (incomplete) geometric extents
            br = region(*node, ni, convertors);
        }

        // set bounding volume
        tile->boundingVolume = br;

        // clip here
        if (clip) { tile->children.clear(); }

        return tile;
    };

    math::Extents3
    measure(const Convertors &convertors
            , const vts::NodeInfo &ni
            , const vts::GeomExtents::ZRange &z)
    {
        // TODO: limit depth?

        if (!ni.valid()) { return math::Extents3(math::InvalidExtents{}); }

        const vts::TileId tileId(ni.nodeId());

        const auto flags(ti_.get(tileId));

        // real tile -> use extents
        if (FT::isReal(flags)) {
            return regionExtents(ni, z, convertors);
        }

        math::Extents3 e(math::InvalidExtents{});

        // check if there are any children, i.e. subtree starting at this tile
        // ID must be valid in next lod's tree (yes, quadtree magic)
        if (!ti_.validSubtree(tileId.lod + 1, tileId)) { return e; }

        // let's traverse children
        for (const auto &child : vts::children(tileId)) {
            auto ce(measure(convertors, ni.child(child), z));
            if (math::valid(ce)) {
                math::update(e, ce);
            }
        }

        return e;
    }

    const vts::TileIndex &ti_;
    bool optimizeBottom_;
    int clipDepth_;
};

} // namespace

bool MetaBuilder::run(tdt::Tileset &ts, int clipDepth)
{
    if (metas_.empty()) { return false; }

    // what version to use in subtilesets?
    ts.asset.tilesetVersion
        = utility::format("%s", delivery_->properties().revision);

    Helper helper(ti_.tileIndex, optimizeBottom_, clipDepth);
    ts.root = helper.traverse(ptc_->get()
                              , vts::NodeInfo(referenceFrame_, rootId_)
                              , metas_);

    // copy geometric error?
    ts.geometricError = ts.root->geometricError;

    return true;
}

} // namespace vts2tdt
