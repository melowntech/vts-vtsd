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

#include <cstdlib>
#include <utility>
#include <functional>
#include <algorithm>
#include <map>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>

#include "utility/streams.hpp"
#include "utility/raise.hpp"
#include "utility/tcpendpoint-io.hpp"
#include "utility/buildsys.hpp"
#include "service/service.hpp"

#include "http/http.hpp"

#include "roarchive/error.hpp"

#include "./error.hpp"
#include "./config.hpp"
#include "./delivery/cache.hpp"
#include "./delivery/slpk/driver.hpp"

#include "./daemon.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace vr = vtslibs::registry;
namespace vts = vtslibs::vts;

namespace {

namespace paths {

    const std::string Rest("/rest/");
    const std::string RestInfo("/rest/info");
    const std::string RestServices("/rest/services");
    const std::string SceneServer("/SceneServer");

} // namespace Paths

LocationConfig defaultConfig()
{
    LocationConfig dc;

    // some file class defaults
    auto &fcs(dc.fileClassSettings);
    fcs.setMaxAge(FileClass::config, 60);
    fcs.setMaxAge(FileClass::data, 604800);

    return dc;
}

} // namespace

class I3sd : public Daemon
{
public:
    I3sd()
        : Daemon("i3sd", 3061, defaultConfig())
    {}

private:
    // service::Service
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd);

    // service::Service
    virtual service::UnrecognizedParser::optional
    configure(const po::variables_map &vars
              , const service::UnrecognizedOptions &unrecognized);

    // service::Service
    virtual void configure(const po::variables_map &vars);

    // service::Service
    virtual std::vector<std::string> listHelps() const;

    // service::Service
    virtual bool help(std::ostream &out, const std::string &what) const;

    // Daemon
    virtual void handleDataset(DeliveryCache &deliveryCache
                               , const fs::path &filePath
                               , const http::Request &request
                               , Sink sink, const LocationConfig &location);

    // Daemon
    virtual DeliveryCache::OpenDriver openDriver();

    // http::ContentGenerator::generate_impl
    virtual void generate_impl(const http::Request &request
                               , const http::ServerSink::pointer &sink);
};

void I3sd::configuration(po::options_description &cmdline
                         , po::options_description &config
                         , po::positional_options_description &pd)
{
    return Daemon::configurationImpl(cmdline, config, pd);
}

service::UnrecognizedParser::optional
I3sd::configure(const po::variables_map &vars
                  , const service::UnrecognizedOptions &unrecognized)
{
    return Daemon::configureImpl(vars, unrecognized);
}

void I3sd::configure(const po::variables_map &vars)
{
    return Daemon::configureImpl(vars);
}

std::vector<std::string> I3sd::listHelps() const
{
    return Daemon::listHelpsImpl();
}

bool I3sd::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("I3S delivery daemon\n"
                "\n"
                );

        return true;
    }

    return Daemon::helpImpl(out, what);
}

DeliveryCache::OpenDriver I3sd::openDriver()
{
    // only SLPK is supported by this server
    return DeliveryCache::OpenDriver(&openSlpk);
}

void I3sd::handleDataset(DeliveryCache &deliveryCache
                         , const fs::path &filePath, const http::Request&
                         , Sink sink, const LocationConfig &location)
{
    const auto sfilePath(filePath.string());
    auto sceneServer(sfilePath.find(paths::SceneServer));
    if (sceneServer == std::string::npos) {
        return sink.error(utility::makeError<NotFound>
                          ("No /SceneServer found in the path."));
    }

    const auto afterSceneServer(sceneServer + paths::SceneServer.size());
    if ((afterSceneServer < sfilePath.size())
        && sfilePath[afterSceneServer] != '/')
    {
        return sink.error(utility::makeError<NotFound>
                          ("SceneServer is not a path element the path."));
    }

    const std::string dataset(sfilePath.substr(0, sceneServer));
    const std::string resource(sfilePath.substr(sceneServer + 1));

    deliveryCache.get
        (dataset
         , [=, &deliveryCache](const DeliveryCache::Expected &value)
         mutable -> void
    {
        try {
            value.get()->handle(sink, resource, location);
        } catch (const roarchive::NoSuchFile&) {
            return sink.error(utility::makeError<NotFound>("Not found."));
        }
    });
}

void I3sd::generate_impl(const http::Request &request
                         , const http::ServerSink::pointer &sink)

{
    if (!ba::starts_with(request.path, paths::Rest)) {
        return sink->error(utility::makeError<NotFound>
                           ("Only paths under /rest are implemented."));
    }

    if (request.path == paths::RestInfo) {
        return sink->error(utility::makeError<InternalError>
                           ("TODO: Implement info json."));
    }

    if (ba::starts_with(request.path, paths::RestServices)) {
        // cut services prefix and go
        http::Request r(request);
        r.path = r.path.substr(paths::RestServices.size());
        return Daemon::generate_impl(r, sink);
    }

    // nothing
    return sink->error(utility::makeError<NotFound>("Not found."));
}

int main(int argc, char *argv[])
{
    return I3sd()(argc, argv);
}
