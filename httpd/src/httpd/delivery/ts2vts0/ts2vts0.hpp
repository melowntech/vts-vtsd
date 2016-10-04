#ifndef libvadstena_http_metatile_ts2vts_hpp_included_
#define libvadstena_http_metatile_ts2vts_hpp_included_

#include "vts-libs/tilestorage/tileop.hpp"
#include "vts-libs/tilestorage/metatile.hpp"

#include "vts-libs/vts0/tileop.hpp"
#include "vts-libs/vts0/metatile.hpp"

namespace vts0 = vadstena::vts0;
namespace ts = vadstena::tilestorage;
namespace vs = vadstena::storage;

inline vts0::TileId asVts(const ts::Index &i)
{
    // since alignment was switched to upper-left corner we have to negate
    // northing; original TileId is tile's lower-left corner therefore we must
    // move to its upper-left corner by subtracting 1
    return { i.lod, i.easting, -1 - i.northing };
}

inline vts0::TileId asVts(const ts::Alignment &alignment, long baseTileSize
                         , const ts::TileId &tid)
{
    return asVts
        (tileIndex(ts::Alignment(alignment(0), alignment(1) + baseTileSize)
                   , baseTileSize, tid));
}

inline vts0::TileId asVts(const ts::Properties &properties
                         , const ts::TileId &tid)
{
    return asVts(properties.alignment, properties.baseTileSize, tid);
}

inline vts0::TileFile asVts(const ts::TileFile &f)
{
    switch (f) {
    case ts::TileFile::mesh: return vts0::TileFile::mesh;
    case ts::TileFile::atlas: return vts0::TileFile::atlas;
    case ts::TileFile::meta: return vts0::TileFile::meta;
    default: break;
    }
    // TODO: throw
    throw;
}

inline vts0::MetaNode asVts(const ts::MetaNode &src)
{
    vts0::MetaNode dst;
    dst.zmin = src.zmin;
    dst.zmax = src.zmax;
    dst.gsd = src.gsd;
    dst.coarseness = src.coarseness;

    std::copy(&src.heightmap[0][0]
              , &src.heightmap[ts::MetaNode::HMSize - 1][ts::MetaNode::HMSize]
              , &dst.heightmap[0][0]);
    std::copy(&src.pixelSize[0][0]
              , &src.pixelSize[1][2]
              , &dst.pixelSize[0][0]);
    return dst;
}

inline vts0::LodLevels asVts(const ts::LodLevels &src)
{
    vts0::LodLevels dst;
    dst.lod = src.lod;
    dst.delta = src.delta;
    return dst;
}

inline ts::TileId asTs(const ts::Properties &properties
                       , const vts0::TileId &tid)
{
    auto ts(ts::tileSize(properties.baseTileSize, tid.lod));
    return { tid.lod, properties.alignment(0) + ts * tid.x
            , properties.alignment(1) + properties.baseTileSize
            - ts * (tid.y + 1) };
}

#endif // libvadstena_http_metatile_ts2vts_hpp_included_
