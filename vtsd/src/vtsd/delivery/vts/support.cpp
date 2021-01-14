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

#include <boost/program_options.hpp>

#include "dbglog/dbglog.hpp"

#include "po.hpp"
#include "tdt2vts/po.hpp"
#include "support.hpp"

namespace po = boost::program_options;
namespace vts = vtslibs::vts;
namespace vs = vtslibs::storage;
namespace vr = vtslibs::registry;

void tileFileStream(Sink &sink, const Location &location
                    , const FileInfo &info
                    , unsigned int subTileFile
                    , const vts::TileId &tileId
                    , vs::IStream::pointer &&is)
{
    switch (info.tileFile) {
    case vs::TileFile::mesh: {
        // read sub-file table from stream
        auto entry(vts::readMeshTable(*is, is->name())
                   [vts::Mesh::meshIndex()]);

        return sink.content(std::move(is), FileClass::data
                            , entry.start, entry.size
                            , vs::gzipped(is, entry.start));
    }

    case vs::TileFile::atlas: {
        // read sub-file table from stream
        auto table(vts::Atlas::readTable(*is, is->name()));

        if (subTileFile >= table.size()) {
            LOGTHROW(err1, vs::NoSuchFile)
                << "Atlas index " << subTileFile
                << " out of range in file: \"" << info.path << "\".";
        }

        const auto &entry(table[subTileFile]);

        // find first entry referencing the same file and redirect if different
        // than subTileFile
        std::size_t index(subTileFile);
        while (index && (table[index - 1] == entry)) { --index; }

        if (index != subTileFile) {
            // build redirect file path
            auto fp(vts::filePath(info.tileFile, tileId, index));
            // append query if present
            if (!location.query.empty()) {
                fp.push_back('?');
                fp.append(location.query);
            }

            return sink.redirect
                (fp, utility::HttpCode::Found, FileClass::data);
        }

        return sink.content(std::move(is), FileClass::data
                            , entry.start, entry.size);
    }

    case vs::TileFile::navtile: {
        // read sub-file table from stream
        auto entry(vts::NavTile::readTable(*is, is->name())
                   [vts::NavTile::imageIndex()]);

        return sink.content(std::move(is), FileClass::data
                            , entry.start, entry.size);
    }

    default: break;
    }

    // default handler
    return sink.content(std::move(is), FileClass::data);
}

void varsConfiguration(boost::program_options::options_description &od
                       , const std::string &prefix
                       , vtslibs::storage::SupportFile::Vars &vars)
{
    if (vars.count("VTS_BUILTIN_BROWSER_URL")) {
        od.add_options()
            ((prefix + "vts.builtinBrowserUrl").c_str()
             , po::value(&vars["VTS_BUILTIN_BROWSER_URL"])
             , "URL of VTS built-in browser.")
            ;
    }

    vts2tdt::varsConfiguration(od, prefix, vars);
}
