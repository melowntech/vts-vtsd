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

#include "dbglog/dbglog.hpp"

#include "../../../error.hpp"

#include "convertors.hpp"

namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;

namespace vts2tdt {

PerThreadConvertors::PerThreadConvertors(const vr::ReferenceFrame &rf)
    : physical_(rf.model.physicalSrs)
{
    auto physical(vr::system.srs.get(rf.model.physicalSrs));
    vr::Srs region;

    // we need to have physical->region system
    srsSet_.insert(rf.model.physicalSrs);
    for (const auto &item : rf.division.nodes) {
        const auto &node(item.second);
        if (!node.real()) { continue; }
        srsSet_.insert(node.srs);
        if (region.srsDef.srs.empty()) {
            region = vr::system.srs.get(node.srs);
        }
    }

    if (physical.type != vr::Srs::Type::cartesian) {
        world_ = geo::geocentric(physical.srsDef);
    }

    region_ = geo::setAngularUnit(geo::geographic(region.srsDef)
                                  , geo::AngularUnit::radian);
}

Convertors PerThreadConvertors::get()
    const
{
    if (auto csMap = csMap_.get()) { return csMap; }

    // new thread -> initialize mapping
    auto csMap(new CsMap());
    csMap_.reset(csMap);

    if (world_.srs.empty()) {
        // world is in model physical system -> no conversion
        csMap->insert(CsMap::value_type());
    } else {
        // needs conversion
        csMap->insert
            (CsMap::value_type({}, vts::CsConvertor(physical_, world_)));
    }

    for (const auto &srs : srsSet_) {
        csMap->insert
            (CsMap::value_type(srs, vts::CsConvertor(srs, region_)));
    }

    return csMap;
}

const vts::CsConvertor& Convertors::get(const std::string &srsId) const
{
    // lookup
    auto fcsMap(csMap_->find(srsId));
    if (fcsMap == csMap_->end()) {
        LOGTHROW(err2, Error)
            << "vts2tdt: Convertor from <" << srsId << "> not found.";
    }
    return fcsMap->second;
}

} // namespace vts2tdt
