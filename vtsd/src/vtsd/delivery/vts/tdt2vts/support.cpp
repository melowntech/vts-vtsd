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

#include <boost/program_options.hpp>

#include "dbglog/dbglog.hpp"

#include "support.hpp"

#include "delivery/vts/tdt2vts/index.html.hpp"

namespace po = boost::program_options;
namespace vs = vtslibs::storage;

namespace vts2tdt {

const vs::SupportFile::Files supportFiles =
{
    { "index.html"
      , {
            browser::index_html
            , sizeof(browser::index_html)
            , browser::index_html_attr_lastModified
            , "text/html; charset=utf-8"
            , true
        }
    }
};

const vs::SupportFile::Vars defaultSupportVars([]()
    -> vs::SupportFile::Vars
{
    vs::SupportFile::Vars vars;
    vars["CESIUM_TERRAIN_PROVIDER_URL"]= "";
    vars["CESIUM_IMAGERY_PROVIDER_URL"]= "";
    return vars;
}());

void varsConfiguration(boost::program_options::options_description &od
                       , const std::string &prefix
                       , vtslibs::storage::SupportFile::Vars &vars)
{
    if (vars.count("CESIUM_TERRAIN_PROVIDER_URL")) {
        od.add_options()
            ((prefix + "cesium.terrainProviderUrl").c_str()
             , po::value(&vars["CESIUM_TERRAIN_PROVIDER_URL"])
             , "URL to Cesium terrain provider root directory.")
            ;
    }

    if (vars.count("CESIUM_IMAGERY_PROVIDER_URL")) {
        od.add_options()
            ((prefix + "cesium.imageryProviderUrl").c_str()
             , po::value(&vars["CESIUM_IMAGERY_PROVIDER_URL"])
             , "Cesium imagery provider URL template.")
            ;
    }
}

} // namespace vts2tdt
