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

#ifndef vtsd_daemon_hpp_included_
#define vtsd_daemon_hpp_included_

#include <cstdlib>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/tcpendpoint-io.hpp"
#include "service/service.hpp"

#include "http/http.hpp"

#include "./config.hpp"
#include "./sink.hpp"
#include "./delivery/cache.hpp"

namespace po = boost::program_options;

/** Helper function.
 */
void sendListing(const boost::filesystem::path &path, Sink &sink
                 , const Sink::Listing &bootstrap = Sink::Listing());

class Daemon
    : public service::Service
    , public http::ContentGenerator
{
public:
    Daemon(const std::string &name, const utility::TcpEndpoint &httpListen
           , const LocationConfig &defaultConfig);

protected:
    void configurationImpl(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd);

    service::UnrecognizedParser::optional
    configureImpl(const po::variables_map &vars
                  , const service::UnrecognizedOptions &unrecognized);

    void configureImpl(const po::variables_map &vars);

    bool helpImpl(std::ostream &out, const std::string &what) const;

    std::vector<std::string> listHelpsImpl() const;

    /** http::ContentGenerator::generate_impl
     *
     * Can be overloaded to handle additional content.
     */
    virtual void generate_impl(const http::Request &request
                               , const http::ServerSink::pointer &sink);

private:
    /** Called for matched dataset-serving location.
     */
    virtual void handleDataset(DeliveryCache &deliveryCache
                               , const boost::filesystem::path &filePath
                               , const http::Request &request, Sink sink
                               , const LocationConfig &location) = 0;

    /** Returns function used to open driver.
     */
    virtual DeliveryCache::OpenDriver openDriver() = 0;

    void handle(const boost::filesystem::path &filePath
                , const http::Request &request
                , const http::ServerSink::pointer &sink
                , const LocationConfig &location);

    void handlePlain(const boost::filesystem::path &filePath
                     , const http::Request &request
                     , Sink sink, const LocationConfig &location);

    void handlePrefix(const LocationConfig &location
                     , const http::Request &request
                     , const http::ServerSink::pointer &sink);

    void handleRegex(const LocationConfig &location
                     , const LocationConfig::MatchResult &m
                     , const http::Request &request
                     , const http::ServerSink::pointer &sink);

    // service::Service
    virtual Service::Cleanup start();

    // service::Service
    virtual int run();

    // service::Service
    virtual void stat(std::ostream &os);

    // service::Service
    virtual void cleanup();

    struct Stopper {
        Stopper(Daemon &d) : d(d) { }
        ~Stopper() { d.cleanup(); }
        Daemon &d;
    };
    friend struct Stopper;

    utility::TcpEndpoint httpListen_;
    unsigned int httpThreadCount_;
    unsigned int coreThreadCount_;

    vtslibs::vts::OpenOptions openOptions_;
    LocationConfig defaultConfig_;
    LocationConfig::list locations_;
    LocationConfig::list prefixLocations_;
    LocationConfig::list regexLocations_;

    boost::optional<http::Http> http_;

    boost::optional<DeliveryCache> deliveryCache_;
};

#endif // vtsd_daemon_hpp_included_
