/**
 * Copyright (c) 2017 Melown Technologies SE
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

#include <mutex>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/algorithm/string/find.hpp>

#include "imgproc/imagesize.hpp"

#include "slpk/restapi.hpp"

#include "driver.hpp"

namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace {

class SlpkDriver : public DriverWrapper
{
public:
    SlpkDriver(slpk::Archive &&reader);

    /** Report memory?
     */
    virtual vs::Resources resources() const { return { 1, 0 }; }

    virtual bool externallyChanged() const { return api_.changed(); }

    virtual void handle(Sink sink, const Location &location
                        , const LocationConfig &config);

private:
    slpk::RestApi api_;
    std::string name_;
};

SlpkDriver::SlpkDriver(slpk::Archive &&reader)
    : api_(std::move(reader))
{}

void SlpkDriver::handle(Sink sink, const Location &location
                        , const LocationConfig &)
{
    // do not allow empty path
    if (location.path.empty()) { throw RedirectToDir(); }

    // list root
    if (location.path == "/") {
        throw ListContent({
            { "SceneServer" }
        }, true);
    }

    roarchive::IStream::pointer is;
    const slpk::ApiFile *apiFile;
    std::tie(is, apiFile) = api_.file(location.path);;

    // get content type
    auto ct(apiFile->contentType);
    auto te(apiFile->transferEncoding);
    if (ct.empty()) {
        // binary by default
        ct = "application/octet-stream";
        if (is && te.empty()) {
            // guess from magic
            auto detected(imgproc::imageMimeType(is->get()));
            if (!detected.empty()) { ct = detected; }
        }
    } else if (is && te.empty()) {
        // try to detect gzip
        auto head(is->get().peek());
        if (head == 0x1f) { te = "gzip"; }
    }

    if (is) {
        // stream
        return sink.content(is, ct, FileClass::data, te);
    }

    // data (TODO: handle different transfer encoding)
    return sink.content
        (apiFile->content
         , Sink::FileInfo(apiFile->contentType, -1)
         .setFileClass(FileClass::data));
}

} // namespace

DriverWrapper::pointer openSlpk(const std::string &path
                               , const OpenOptions&
                               , DeliveryCache &cache
                               , const DeliveryCache::Callback &callback)
{
    // TODO: check mime
    LOG(info1) << "about to open <" << path << ">";

    cache.post(callback, [=]() -> void
    {
        // TODO: handle exceptions

        slpk::Archive reader(path);
        callback(DriverWrapper::pointer
                 (std::make_shared<SlpkDriver>(std::move(reader))));
    });
    return {};
}
