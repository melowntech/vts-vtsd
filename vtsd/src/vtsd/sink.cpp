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

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "http/error.hpp"

#include "sink.hpp"
#include "error.hpp"

namespace fs = boost::filesystem;

namespace {

http::SinkBase::CacheControl
cacheControl(FileClass fileClass, const FileClassSettings *fileClassSettings)
{
    http::SinkBase::CacheControl cc;

    if (!fileClassSettings) { return cc; }

    const auto &fc(*fileClassSettings);
    cc.maxAge = fc.getMaxAge(fileClass);
    if (*cc.maxAge > 0) {
        cc.staleWhileRevalidate = fc.getStaleWhileRevalidate(fileClass);
    }

    return cc;
}

Sink::FileInfo update(const Sink::FileInfo &inStat
                      , const FileClassSettings *fileClassSettings)
{
    if (inStat.cacheControl.maxAge) { return inStat; }
    auto stat(inStat);
    auto &cc(stat.cacheControl);

    if (!fileClassSettings) {
        // no file class attached, no caching
        cc.maxAge = -1;
        return stat;
    }

    // set max age based on fileClass settings
    const auto &fc(*fileClassSettings);
    cc.maxAge = fc.getMaxAge(stat.fileClass);
    if (*cc.maxAge > 0) {
        // we are caching, what about stale settings?
        cc.staleWhileRevalidate = fc.getStaleWhileRevalidate(stat.fileClass);
    }

    // done
    return stat;
}

namespace {

const http::Header::list emptyHeaders;

const http::Header::list& fromOpt(const http::Header::list *headers) {
    if (headers) { return *headers; }
    return emptyHeaders;
}

} // namespace

class IStreamDataSource : public http::ServerSink::DataSource {
public:
    IStreamDataSource(const vs::IStream::pointer &stream
                      , FileClass fileClass
                      , const FileClassSettings *fileClassSettings
                      , bool gzipped = false
                      , const http::Header::list *headers = nullptr)
        : stream_(stream), stat_(stream->stat())
        , fs_(Sink::FileInfo(stat_.contentType, stat_.lastModified
                             , cacheControl(fileClass, fileClassSettings)))
        , headers_(fromOpt(headers))
    {
        // do not fail on eof
        stream->get().exceptions(std::ios::badbit);

         if (gzipped) {
            headers_.emplace_back("Content-Encoding", "gzip");
        }
   }

    virtual http::SinkBase::FileInfo stat() const {
        return fs_;
    }

    virtual std::size_t read(char *buf, std::size_t size
                                 , std::size_t off)
    {
        return stream_->read(buf, size, off);
    }

    virtual std::string name() const { return stream_->name(); }

    virtual void close() const { stream_->close(); }

    virtual long size() const { return stat_.size; }

    virtual const http::Header::list *headers() const { return &headers_; }

private:
    vs::IStream::pointer stream_;
    vs::FileStat stat_;
    Sink::FileInfo fs_;
    http::Header::list headers_;
};

class SubIStreamDataSource : public http::ServerSink::DataSource {
public:
    SubIStreamDataSource(const vs::IStream::pointer &stream
                         , FileClass fileClass
                         , const FileClassSettings *fileClassSettings
                         , std::size_t offset, std::size_t size
                         , bool gzipped
                         , const http::Header::list *headers = nullptr)
        : stream_(stream), stat_(stream->stat())
        , fs_(Sink::FileInfo(stat_.contentType, stat_.lastModified
                             , cacheControl(fileClass, fileClassSettings)))
        , headers_(fromOpt(headers))
        , offset_(offset), end_(offset + size)
    {
        // sanity check
        if (end_ > stat_.size) { end_ = stat_.size; }

        // update size
        stat_.size = (end_ - offset_);

        // do not fail on eof
        stream->get().exceptions(std::ios::badbit);

        if (gzipped) {
            headers_.emplace_back("Content-Encoding", "gzip");
        }
    }

    virtual http::SinkBase::FileInfo stat() const { return fs_; }

    virtual std::size_t read(char *buf, std::size_t size, std::size_t off) {
        // fix-ups
        auto offset(off + offset_);
        if (offset > end_) { return 0; }
        auto left(end_ - offset);
        if (size > left) { size = left; }
        return stream_->read(buf, size, offset);
    }

    virtual std::string name() const { return stream_->name(); }

    virtual void close() const { stream_->close(); }

    virtual long size() const { return stat_.size; }

    virtual const http::Header::list *headers() const { return &headers_; }

private:
    vs::IStream::pointer stream_;
    vs::FileStat stat_;
    Sink::FileInfo fs_;
    http::Header::list headers_;
    std::size_t offset_;
    std::size_t end_;
};

class RoArchiveDataSource : public http::SinkBase::DataSource
{
public:
    RoArchiveDataSource(roarchive::IStream::pointer &&is
                        , const std::string &contentType, FileClass fileClass
                        , const FileClassSettings *fileClassSettings
                        , const std::string &trasferEncoding
                        , const http::Header::list *headers = nullptr)
        : http::SinkBase::DataSource(true), is_(std::move(is))
        , size_(is_->size() ? *is_->size() : -1)
        , seekable_(is_->seekable()), off_()
        , headers_(fromOpt(headers))
    {
        fi_.lastModified = is_->timestamp();
        fi_.contentType = contentType;
        fi_.cacheControl = cacheControl(fileClass, fileClassSettings);

        if (!trasferEncoding.empty()) {
            headers_.emplace_back("Content-Encoding", trasferEncoding);
        }
    }

private:
    virtual http::SinkBase::FileInfo stat() const { return fi_; }

    virtual std::size_t read(char *buf, std::size_t size
                             , std::size_t off);

    virtual void close() const { is_->close(); }

    virtual long size() const { return size_; }

    virtual const http::Header::list *headers() const { return &headers_; }

    roarchive::IStream::pointer is_;
    http::SinkBase::FileInfo fi_;

    long size_;
    bool seekable_;
    std::size_t off_;
    http::Header::list headers_;
};

std::size_t RoArchiveDataSource::read(char *buf, std::size_t size
                                      , std::size_t off)
{
    if (off != off_) {
        if (seekable_) {
            is_->get().seekg(off);
            off_ = off;
        } else {
            LOGTHROW(err2, std::runtime_error)
                << "This archive file is unseekable.";
        }
    }

    // fix size if too long
    {
        long end(off_ + size);
        if (end > size_) { size = size_ - off_; }
    }

    is_->get().read(buf, size);
    auto read(is_->get().gcount());

    off_ = off + read;
    return read;
}

} // namespace

void Sink::content(vs::IStream::pointer &&stream, FileClass fileClass
                   , bool gzipped)
{
    sink_->content(std::make_shared<IStreamDataSource>
                   (std::move(stream), fileClass
                    , &locationConfig_.fileClassSettings
                    , gzipped, &locationConfig_.httpHeaders));
}

void Sink::content(vs::IStream::pointer &&stream
                   , FileClass fileClass, std::size_t offset, std::size_t size
                   , bool gzipped)
{
    sink_->content(std::make_shared<SubIStreamDataSource>
                   (std::move(stream), fileClass
                    , &locationConfig_.fileClassSettings
                    , offset, size, gzipped, &locationConfig_.httpHeaders));
}

void Sink::content(roarchive::IStream::pointer &&stream
                   , const std::string &contentType, FileClass fileClass
                   , const std::string &trasferEncoding)
{
    sink_->content(std::make_shared<RoArchiveDataSource>
                   (std::move(stream), contentType
                    , fileClass, &locationConfig_.fileClassSettings
                    , trasferEncoding, &locationConfig_.httpHeaders));
}

void Sink::content(const vs::SupportFile &data)
{
    if (!data.isTemplate) {
        FileInfo stat(data.contentType, data.lastModified);
        stat.setFileClass(FileClass::support);
        // not a template
        content(data.data, data.size, stat, false);
        return;
    }

    // content is expanded -> modified now!
    FileInfo stat(data.contentType);
    stat.setFileClass(FileClass::support);
    content(data.expand(&locationConfig_.vars, nullptr), stat);
}

void Sink::error(const std::exception_ptr &exc)
{
    sink_->error(exc);
}

Sink::FileInfo& Sink::FileInfo::setFileClass(FileClass fc)
{
    fileClass = fc;
    return *this;
}

Sink::FileInfo& Sink::FileInfo::setMaxAge(const boost::optional<long> &ma)
{
    cacheControl.maxAge = ma;
    return *this;
}

Sink::FileInfo& Sink::FileInfo::setStaleWhileRevalidate(long stale)
{
    cacheControl.staleWhileRevalidate = stale;
    return *this;
}

Sink::FileInfo Sink::update(const FileInfo &stat) const
{
    return ::update(stat, &locationConfig_.fileClassSettings);
}

void Sink::listing(const boost::filesystem::path &path
                   , const Sink::Listing &bootstrap)
{
    http::ServerSink::Listing list(bootstrap);

    for (fs::directory_iterator ipath(path), epath; ipath != epath; ++ipath) {
        const auto &entry(*ipath);
        list.emplace_back(entry.path().filename().string()
                          , (fs::is_directory(entry.status())
                             ? ListingItem::Type::dir
                                : ListingItem::Type::file));
    }

    listing(list);
}

void Sink::redirect(const std::string &url, utility::HttpCode code
                    , FileClass fileClass)
{
    sink_->redirect
        (url, code
         , update(Sink::FileInfo().setFileClass(fileClass)).cacheControl);
}
