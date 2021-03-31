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

#ifndef vtsd_delivery_vts_vts2tdt_convertors_hpp_included_
#define vtsd_delivery_vts_vts2tdt_convertors_hpp_included_

#include <new>
#include <map>

#include <boost/thread/tss.hpp>

#include "vts-libs/vts/csconvertor.hpp"

namespace vts2tdt {

using CsMap = std::map<std::string, vtslibs::vts::CsConvertor>;

class Convertors {
public:
    enum class BoundingVolume { region, box };

    Convertors(const CsMap *csMap, BoundingVolume boundingVolume)
        : csMap_(csMap), boundingVolume_(boundingVolume)
    {}

    const vtslibs::vts::CsConvertor& get(const std::string &srsId = {}) const;

    const vtslibs::vts::CsConvertor& operator()(const std::string &srsId = {})
        const { return get(srsId); }

    BoundingVolume boundingVolume() const { return boundingVolume_; }

private:
    const CsMap *csMap_;
    BoundingVolume boundingVolume_;
};

/** Holds per-thread convertors for converting VTS data to 3DTiles.
 *
 *  convention:
 *      unnamed convertor: model.physicalSrs -> 3DTiles (geocentric)
 *      named: SDS SRS -> 3DTiles geographic in radians
 */
class PerThreadConvertors {
public:
    using pointer = std::shared_ptr<PerThreadConvertors>;

    PerThreadConvertors(const vtslibs::registry::ReferenceFrame &rf);

    Convertors get() const;

private:
    std::string physical_;

    using SrsSet = std::set<std::string>;
    SrsSet srsSet_;

    /** 3D Tiles world. May be empty when model.physicalSrs is already
     *  geocentric system.
     *
     *  TODO: optimize if known system
     */
    geo::SrsDefinition world_;

    /** Bounding volume SRS. TODO: optimize if known system
     */
    geo::SrsDefinition boundingVolumeSrs_;

    /** What type of bounding volume we can produce.
     */
    Convertors::BoundingVolume boundingVolume_;

    /** Cached convertors.
     */
    mutable boost::thread_specific_ptr<CsMap> csMap_;
};

} // namespace vts2tdt

#endif // vtsd_delivery_vts_vts2tdt_convertors_hpp_included_
