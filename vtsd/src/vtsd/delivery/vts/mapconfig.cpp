/**
 * Copyright (c) 2018 Melown Technologies SE
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

#include "vts-libs/vts/service.hpp"

#include "mapconfig.hpp"

namespace fs = boost::filesystem;

namespace mapconfig {

vts::MapConfig augmentMapConfig(vts::MapConfig &&mc)
{
    mc.srs.for_each([](vr::Srs &srs)
    {
        if (!srs.geoidGrid) { return; }
        // geoid grid -> use only filename in definition
        srs.geoidGrid->definition
            = fs::path(srs.geoidGrid->definition).filename().string();
    });

    // add services
    vts::service::addLocal(mc);

    return std::move(mc);
}

BaseMapConfig::BaseMapConfig(const vts::MapConfig &mc, std::time_t lastModified)
{
    // serialize map configuration
    {
        std::ostringstream os;
        saveMapConfig(mc, os);
        set(os.str(), lastModified, vts::MapConfig::contentType);
    }

    // serialize dirs
    {
        std::ostringstream os;
        saveDirs(mc, os);
        dirs.set(os.str(), lastModified, vts::MapConfig::contentType);
    }
}

void SimpleMapConfig::sendMapConfig(Sink &sink, const vts::OProxy&
                                    , FileClass fileClass)
    const
{
    send(sink, fileClass);
}

void SimpleMapConfig::sendDirs(Sink &sink, const vts::OProxy&
                               , FileClass fileClass) const
{
    dirs.send(sink, fileClass);
}

void ProxiedMapConfig::sendMapConfig(Sink &sink, const vts::OProxy &proxy
                                     , FileClass fileClass)
    const
{
    vts::MapConfigOptions mco;
    mco.proxy = proxy;
    std::ostringstream os;
    saveMapConfig(mc_, os, &mco);

    return SerializedConfig
        (os.str(), lastModified_, vts::MapConfig::contentType)
        .send(sink, fileClass);
}

void ProxiedMapConfig::sendDirs(Sink &sink, const vts::OProxy &proxy
                                , FileClass fileClass) const
{
    vts::MapConfigOptions mco;
    mco.proxy = proxy;
    std::ostringstream os;
    saveDirs(mc_, os, &mco);

    return SerializedConfig
        (os.str(), lastModified_, vts::MapConfig::contentType)
        .send(sink, fileClass);
}

void Definition::init(const vts::MeshTilesConfig &mtc
                      , std::time_t lastModified)
{
    {
        const auto fl(vts::freeLayer(mtc));

        std::ostringstream os;
        vr::saveFreeLayer(os, fl);
        def.set(os.str(), lastModified, vts::MeshTilesConfig::contentType);
    }

    {
        const auto dc(vts::debugConfig(mtc));

        std::ostringstream os;
        vts::saveDebug(os, dc);
        debug.set(os.str(), lastModified, vts::DebugConfig::contentType);
    }
}

} // namespace mapconfig
