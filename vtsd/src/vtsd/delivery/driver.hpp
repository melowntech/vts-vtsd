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

#ifndef httpd_delivery_driver_hpp_included_
#define httpd_delivery_driver_hpp_included_

#include "vts-libs/storage/resources.hpp"
#include "vts-libs/storage/streams.hpp"
#include "vts-libs/storage/support.hpp"

#include "utility/raise.hpp"

#include "../sink.hpp"
#include "../config.hpp"

#include "../sink.hpp"

namespace vs = vtslibs::storage;

struct OpenInfo {
    boost::filesystem::path path;
    std::string mime;

    OpenInfo(const boost::filesystem::path &path
             , const std::string &mime)
        : path(path), mime(mime)
    {}
};

/** Can be thrown by handler to force redirect.
 */
struct ListContent {
    Sink::Listing listingBootstrap;

    ListContent(Sink::Listing listingBootstrap = Sink::Listing())
        : listingBootstrap(listingBootstrap)
    {}
};

struct FileInfo {
    enum class Type {
        unknown, file, tileFile, support, definition, dirs, tilesetMapping
    };

    std::string path;

    // type of file
    Type type;

    // tileset file, valid only when (type == Type::file)
    vs::File file;

    // tileset tile file, valid only when (type == Type::tileFile)
    vs::TileFile tileFile;

    // support file, valid only when (type == Type::support)
    const vs::SupportFile::Files::value_type *support;

    FileInfo(const std::string &path)
        : path(path), type(), file(), tileFile(), support()
    {}
};

class DriverWrapper : boost::noncopyable {
public:
    typedef std::shared_ptr<DriverWrapper> pointer;

    DriverWrapper(const DatasetProvider &datasetProvider)
        : datasetProvider_(datasetProvider)
    {}
    virtual ~DriverWrapper() {}

    virtual vs::Resources resources() const = 0;
    virtual bool externallyChanged() const = 0;

    virtual void handle(Sink sink, const std::string &path
                        , const LocationConfig &config) = 0;

    /** Open driver for hot content is never cached.
     */
    virtual bool hotContent() const { return false; }

    Sink::FileInfo fileinfo(const vs::FileStat &fileStat, FileClass fc);
    Sink::FileInfo fileinfo(const vs::FileStat &fileStat
                            , const boost::optional<long> &maxAge
                            = boost::none);
    Sink::FileInfo fileinfo(const vs::SupportFile &file, FileClass fc);

    const DatasetProvider& datasetProvider() const { return datasetProvider_; }

private:
    const DatasetProvider datasetProvider_;
};

// inlines

inline Sink::FileInfo
DriverWrapper::fileinfo(const vs::FileStat &fileStat, FileClass fc)
{
    return Sink::FileInfo(fileStat.contentType, fileStat.lastModified)
        .setFileClass(fc);
}

inline Sink::FileInfo
DriverWrapper::fileinfo(const vs::FileStat &fileStat
                        , const boost::optional<long> &maxAge)
{
    return Sink::FileInfo(fileStat.contentType, fileStat.lastModified
                          , maxAge);
}

#endif // httpd_delivery_driver_hpp_included_
