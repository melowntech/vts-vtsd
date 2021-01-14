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

#ifndef vtsd_delivery_vts_3dtiles_hpp_included_
#define vtsd_delivery_vts_3dtiles_hpp_included_

#include "vts-libs/vts/tileset/delivery.hpp"
#include "vts-libs/vts/csconvertor.hpp"

#include "../driver.hpp"

#include "tdt2vts/convertors.hpp"

class Tdt2VtsTileSet : public DriverWrapper
{
public:
    Tdt2VtsTileSet(const vtslibs::vts::Delivery::pointer &delivery);

    virtual vs::Resources resources() const {
        return delivery_->resources();
    }

    virtual bool externallyChanged() const {
        return delivery_->externallyChanged();
    }

    virtual void handle(Sink sink, const Location &location
                        , const LocationConfig &config
                        , const ErrorHandler::pointer &errorHandler);

private:
    vtslibs::vts::Delivery::pointer delivery_;
    vtslibs::registry::ReferenceFrame referenceFrame_;

    vts2tdt::PerThreadConvertors convertors_;
};

#endif // vtsd_delivery_vts_3dtiles_hpp_included_
