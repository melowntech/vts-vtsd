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

#ifndef mapproxy_sink_hpp_included_
#define mapproxy_sink_hpp_included_

#include <ctime>
#include <string>
#include <memory>
#include <exception>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"

#include "http/contentgenerator.hpp"

#include "roarchive/istream.hpp"

#include "vts-libs/storage/streams.hpp"
#include "vts-libs/storage/support.hpp"

#include "fileclass.hpp"
#include "error.hpp"
#include "config.hpp"

namespace vs = vtslibs::storage;

/** Aborter helper.
 */
struct Aborter {
    typedef http::ServerSink::AbortedCallback AbortedCallback;

    virtual ~Aborter() {}

    /** Defaults to dummy aborter
     */
    virtual void setAborter(const AbortedCallback&) {};
};

/** Wraps libhttp's sink.
 */
class Sink : public Aborter {
public:
    typedef http::ServerSink::AbortedCallback AbortedCallback;
    typedef http::ServerSink::ListingItem ListingItem;
    typedef std::vector<ListingItem> Listing;

    struct FileInfo : http::SinkBase::FileInfo {
        FileInfo(const std::string &contentType = "application/octet-stream"
                 , std::time_t lastModified = -1
                 , const http::SinkBase::CacheControl &cacheControl
                 = http::SinkBase::CacheControl())
            : http::SinkBase::FileInfo(contentType, lastModified, cacheControl)
        {}

        FileInfo& setFileClass(FileClass fc);

        FileInfo& setMaxAge(const boost::optional<long> &maxAge);
        FileInfo& setStaleWhileRevalidate(long stale);

        FileClass fileClass;
        http::Header::list headers;
    };

    Sink(const http::ServerSink::pointer &sink
         , const LocationConfig &locationConfig)
        : sink_(sink), locationConfig_(locationConfig) {}

    /** Sends content to client.
     * \param data data top send
     * \param stat file info (size is ignored)
     */
    void content(const std::string &data, const FileInfo &stat);

    /** Sends support file to client.
     * \param data data to send
     */
    void content(const vs::SupportFile &data);

    /** Sends content to client.
     * \param data data top send
     * \param stat file info (size is ignored)
     */
    template <typename T>
    void content(const std::vector<T> &data, const FileInfo &stat);

    /** Sends content to client.
     * \param data data top send
     * \param size size of data
     * \param stat file info (size is ignored)
     * \param needCopy data are copied if set to true
     */
    void content(const void *data, std::size_t size
                 , const FileInfo &stat, bool needCopy);

    /** Sends content to client.
     * \param stream stream to send
     * \param fileClass file class
     */
    void content(const vs::IStream::pointer &stream
                 , FileClass fileClass);

    /** Sends content to client.
     * \param stream stream to send
     * \param fileClass file class
     * \param offset start offset in file
     * \param size size of content
     * \param gzipped adds Content-Encoding: gzip
     */
    void content(const vs::IStream::pointer &stream
                 , FileClass fileClass, std::size_t offset, std::size_t size
                 , bool gzipped = false);

    /** Sends content to client.
     * \param stream stream to send
     * \param contentType mime type
     * \param fileClass file class
     * \param gzipped adds Content-Encoding: gzip
     */
    void content(const roarchive::IStream::pointer &stream
                 , const std::string &contentType
                 , FileClass fileClass
                 , const std::string &trasferEncoding = "");

    /** Tell client to look somewhere else.
     */
    void redirect(const std::string &url, utility::HttpCode code) {
        sink_->redirect(url, code);
    }

    /** Tell client to look somewhere else. Add file class to set caching.
     */
    void redirect(const std::string &url, utility::HttpCode code
                  , FileClass fileClass);

    /** Generates listing.
     */
    void listing(const Listing &list) {
        sink_->listing(list);
    }

    /** Generate listing from given path.
     */
    void listing(const boost::filesystem::path &path
                 , const Sink::Listing &bootstrap = Sink::Listing());

    /** Sends current exception to the client.
     */
    void error();

    /** Sends given error to the client.
     */
    template <typename T> void error(const T &exc);

    /** Checks wheter client aborted request.
     *  Throws RequestAborted exception when true.
     */
    void checkAborted() const {
        sink_->checkAborted();
    }

    /** Sets aborted callback.
     */
    virtual void setAborter(const AbortedCallback &ac) {
        sink_->setAborter(ac);
    }

private:
    /** Sends given error to the client.
     */
    void error(const std::exception_ptr &exc);

    FileInfo update(const FileInfo &stat) const;

    http::ServerSink::pointer sink_;

    const LocationConfig &locationConfig_;
};

// inlines

template <typename T>
inline void Sink::error(const T &exc)
{
    error(std::make_exception_ptr(exc));
}

inline void Sink::error() { error(std::current_exception()); }

inline void Sink::content(const std::string &data, const FileInfo &stat) {
    sink_->content(data, update(stat), &stat.headers);
}

template <typename T>
inline void Sink::content(const std::vector<T> &data, const FileInfo &stat) {
    sink_->content(data, update(stat), &stat.headers);
}

inline void Sink::content(const void *data, std::size_t size
                          , const FileInfo &stat, bool needCopy)
{
    sink_->content(data, size, update(stat), needCopy, &stat.headers);
}

#endif // mapproxy_sink_hpp_included_

