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

#include "vts-libs/storage/error.hpp"
#include "vts-libs/tilestorage/driver.hpp"
#include "vts-libs/tilestorage/io.hpp"
#include "vts-libs/tilestorage/support.hpp"
#include "vts-libs/tilestorage/tileop.hpp"
#include "vts-libs/tilestorage/config.hpp"

#include "./driver.hpp"

namespace fs = boost::filesystem;
namespace ts = vtslibs::tilestorage;
namespace vs = vtslibs::storage;

namespace {

struct TsFileInfo : public FileInfo {
    // tileId, valid only when (type == Type::tileFile)
    ts::TileId tileId;

    TsFileInfo(const std::string &path, const LocationConfig &config);
};

namespace constants {
    const std::string Config("mapConfig.json");
    const std::string Self(".");
    const std::string Index("index.html");
}

TsFileInfo::TsFileInfo(const std::string &p, const LocationConfig &config)
    : FileInfo(p)
{
    if (ts::fromFilename(tileId, tileFile, path.c_str())) {
        type = Type::tileFile;
        return;
    }

    if (constants::Config == path) {
        type = Type::file;
        file = vs::File::config;
        return;
    }

    if (!config.enableBrowser) {
        LOG(debug) << "Browser disabled, skipping browser files.";
        return;
    }

    LOG(debug) << "Browser enabled, checking browser files.";

    // translate "." -> index
    if (constants::Self == path) { path = constants::Index; }

    auto fsupport(ts::supportFiles.find(path));
    if (fsupport != ts::supportFiles.end()) {
        type = Type::support;
        support = &*fsupport;
    }
}

class TsDriver : public DriverWrapper
{
public:
    TsDriver(const ts::Driver::pointer &driver)
        : driver_(driver)
    {}

    virtual vs::Resources resources() const {
        return driver_->resources();
    }

    virtual bool externallyChanged() const {
        return driver_->externallyChanged();
    }

    virtual void handle(Sink sink, const Location &location
                        , const LocationConfig &config);

private:
    ts::Driver::pointer driver_;
    std::mutex mutex_;
};

void TsDriver::handle(Sink sink, const Location &location
                      , const LocationConfig &config)
{
    TsFileInfo info(location.path, config);

    switch (info.type) {
    case FileInfo::Type::file: {
        std::unique_lock<std::mutex> guard(mutex_);
        return sink.content(driver_->input(info.file), FileClass::config);
    }

    case FileInfo::Type::tileFile:
    {
        std::unique_lock<std::mutex> guard(mutex_);
        return sink.content(driver_->input(info.tileId, info.tileFile)
                            , FileClass::data);
    }

    case FileInfo::Type::support:
        return sink.content(info.support->second);

    default: break;
    }

    // wtf?
    sink.error(utility::makeError<NotFound>("Unknown file."));
}

} // namespace

DriverWrapper::pointer openTileSet(const std::string &path)
{
    return std::make_shared<TsDriver>
        (ts::Driver::open(path, ts::OpenMode::readOnly));
}
