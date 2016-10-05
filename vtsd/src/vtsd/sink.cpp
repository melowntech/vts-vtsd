#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "http/error.hpp"

#include "./sink.hpp"
#include "./error.hpp"

namespace {

boost::optional<long> maxAge(FileClass fileClass
                             , const FileClassSettings *fileClassSettings)
{
    if (!fileClassSettings) { return boost::none; }
    return fileClassSettings->getMaxAge(fileClass);
}

Sink::FileInfo update(const Sink::FileInfo &inStat
                      , const FileClassSettings *fileClassSettings)
{
    if (inStat.maxAge) { return inStat; }
    auto stat(inStat);

    if (!fileClassSettings) {
        // no file class attached, no caching
        stat.maxAge = -1;
        return stat;
    }

    // set max age based on fileClass settings
    stat.maxAge = fileClassSettings->getMaxAge(stat.fileClass);

    // done
    return stat;
}

class IStreamDataSource : public http::ServerSink::DataSource {
public:
    IStreamDataSource(const vs::IStream::pointer &stream
                      , FileClass fileClass
                      , const FileClassSettings *fileClassSettings)
        : stream_(stream), stat_(stream->stat())
        , fs_(Sink::FileInfo(stat_.contentType, stat_.lastModified
                             , maxAge(fileClass, fileClassSettings)))
    {
        // do not fail on eof
        stream->get().exceptions(std::ios::badbit);
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
                         , bool gzipped)
        : stream_(stream), stat_(stream->stat())
        , fs_(Sink::FileInfo(stat_.contentType, stat_.lastModified
                             , maxAge(fileClass, fileClassSettings)))
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

} //namesapce

void Sink::content(const vs::IStream::pointer &stream
                   , FileClass fileClass)
{
    sink_->content(std::make_shared<IStreamDataSource>
                   (stream, fileClass, &locationConfig_.fileClassSettings));
}

void Sink::content(const vs::IStream::pointer &stream
                   , FileClass fileClass, std::size_t offset, std::size_t size
                   , bool gzipped)
{
    sink_->content(std::make_shared<SubIStreamDataSource>
                   (stream, fileClass, &locationConfig_.fileClassSettings
                    , offset, size, gzipped));
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
    maxAge = ma;
    return *this;
}

Sink::FileInfo Sink::update(const FileInfo &stat) const
{
    return ::update(stat, &locationConfig_.fileClassSettings);
}
