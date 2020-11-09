/**
 * Copyright (c) 2020 Melown Technologies SE
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

#include "3dtiles/3dtiles.hpp"

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
    (void) config;

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

void generateTileset(Sink &sink, const Location &location
                     , const vts::Delivery &delivery)
{
    const auto &props(delivery.properties());

    tdt::Tileset ts;
    // use revision as tileset version
    ts.asset.tilesetVersion = utility::format("%s", props.revision);
    ts.geometricError = 1e6; // how can I know?

    ts.root = std::make_unique<tdt::Tile>();
    ts.root->children.emplace_back(std::make_unique<tdt::Tile>());
    auto &tile(*ts.root->children.back());
    // tile.boundingVolueme = ;
    tile.content.emplace();
    tile.content->uri = "0-0-0.json";

    return sink.content(serialize(ts, location.path, vs::File::config
                                  , delivery.lastModified())
                        , FileClass::config);
}

} // namespace vts2tdt

Tdt2VtsTileSet::Tdt2VtsTileSet(const vts::Delivery::pointer &delivery)
    : delivery_(delivery)
{}

void Tdt2VtsTileSet::handle(Sink sink, const Location &location
                            , const LocationConfig &config
                            , const ErrorHandler::pointer &errorHandler)
{
    // we want internals
    const vts2tdt::FileInfo info(location.path, config);

    try {
        switch (info.type) {
        case FileInfo::Type::file:
            if (info.file != vs::File::config) {
                return sink.error(utility::makeError<NotFound>
                                  ("Unknown file type."));
            }

            return vts2tdt::generateTileset(sink, location, *delivery_);

        case FileInfo::Type::tileFile:
            return sink.error(utility::makeError<InternalError>
                              ("TODO: implement me"));
            // return handleTile(sink, location, config, errorHandler, info);

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
