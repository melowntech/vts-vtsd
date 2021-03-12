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

#ifndef vtsd_delivery_vts_vts2tdt_metabuilder_hpp_included_
#define vtsd_delivery_vts_vts2tdt_metabuilder_hpp_included_

#include "utility/gzipper.hpp"

#include "vts-libs/storage/support.hpp"
#include "vts-libs/vts/tileset/driver.hpp"
#include "vts-libs/vts/tileset/delivery.hpp"

#include "3dtiles/3dtiles.hpp"

#include "../../driver.hpp"
#include "convertors.hpp"

namespace vts2tdt {

template <typename T>
vtslibs::storage::IStream::pointer serialize(const threedtiles::Tileset &ts
                                             , const std::string &path
                                             , T &&type
                                             , std::time_t lastModified)
{
    auto is(std::make_shared<vs::StringIStream>(type, path, lastModified));
    threedtiles::write(utility::Gzipper(is->sink()), ts);
    is->updateSize();
    return is;
}

class MetaBuilder {
public:
    using pointer = std::shared_ptr<MetaBuilder>;
    using CompletionHandler = std::function<void(MetaBuilder&)>;

    MetaBuilder(const vtslibs::vts::Delivery::pointer &delivery
                , const vtslibs::registry::ReferenceFrame &referenceFrame
                , PerThreadConvertors::pointer ptc
                , const vtslibs::vts::TileId &rootId
                , bool optimizeBottom = true)
        : delivery_(delivery)
        , referenceFrame_(referenceFrame)
        , ptc_(std::move(ptc))
        , rootId_(rootId)
        , ti_(delivery->index())
        , lastModified_()
        , optimizeBottom_(optimizeBottom)
    {}

    /** Load metatiles to given depth. Negative depth is replace by metatile
     *  binary order.
     */
    void load(int depth = -1);

    /** Ansync loader.
     */
    static void load_async(const pointer &self
                           , const ErrorHandler::pointer &errorHandler
                           , const CompletionHandler &cb
                           , int depth = -1);

    /** Generate 3DTiles tileset pyramid from loaded metatiles.
     *
     *  Generated pyramid can be clipped at given depth. If clipDepth > depth or
     *  < 0 then clipDepth is internally set to depth, where depth is depth of
     *  loaded metatile subtree.
     *
     * \param ts tileset to fill-in
     * \param clipDepth clip pyramid at given depth
     */
    bool run(threedtiles::Tileset &ts, int clipDepth = -1);

    /** Send tileset to peer.
     */
    template <typename FileType>
    void send(Sink &sink, const Location &location
              , const threedtiles::Tileset &ts
              , FileType fileType, FileClass fileClass
              , std::time_t lastModified)
    {
        return sink.content
            (serialize(ts, location.path, fileType, lastModified)
             , fileClass, true);
    }

    /** Send tileset to peer.
     */
    void send(Sink &sink, const Location &location
              , const threedtiles::Tileset &ts)
    {
        return send(sink, location, ts, vs::TileFile::meta, FileClass::data
                    , lastModified_);
    }

    vtslibs::vts::Delivery& delivery() const { return *delivery_; }

private:
    static void loadNext(pointer self
                         , const ErrorHandler::pointer &errorHandler
                         , CompletionHandler cb
                         , const vtslibs::vts::TileId &tileId, int left);

    vtslibs::vts::Delivery::pointer delivery_;
    const vtslibs::registry::ReferenceFrame &referenceFrame_;
    const PerThreadConvertors::pointer ptc_;
    const vtslibs::vts::TileId rootId_;

    const vtslibs::vts::tileset::Index &ti_;

    std::time_t lastModified_;
    bool optimizeBottom_;

    vtslibs::vts::MetaTile::list metas_;
};


} // namespace vts2tdt

#endif // vtsd_delivery_vts_vts2tdt_metabuilder_hpp_included_
