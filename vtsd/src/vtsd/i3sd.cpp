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

#include "jsoncpp/json.hpp"
#include "jsoncpp/io.hpp"

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

namespace mimes {
    const std::string ApplicationJson = "application/json; charset=UTF-8";
} // namespace mimes

LocationConfig defaultConfig()
{
    LocationConfig dc;

    // some file class defaults
    auto &fcs(dc.fileClassSettings);
    fcs.setMaxAge(FileClass::config, 60);
    fcs.setMaxAge(FileClass::data, 604800);

    return dc;
}

void addNonEmpty(Json::Value &value, const std::string &str)
{
    if (!str.empty()) { value = str; }
}

struct ServerInfo {
    double currentVersion;
    std::string fullVersion;

    std::string soapUrl;
    std::string secureSoapUrl;
    std::string owningSystemUrl;

    bool isTokenBasedSecurity;
    std::string tokenServicesUrl;
    std::time_t shortLivedTokenValidity;

    // built in configure
    std::string json;

    ServerInfo()
        : currentVersion(10.51), fullVersion("10.5.1")
        , isTokenBasedSecurity(false)
        , shortLivedTokenValidity(60)
    {}

    void configuration(po::options_description &config
                       , const std::string &prefix = "")
    {
        config.add_options()
            ((prefix + "currentVersion").c_str()
             , po::value(&currentVersion)
             ->default_value(currentVersion)->required()
             , "Server version (number).")
            ((prefix + "fullVersion").c_str()
             , po::value(&fullVersion)
             ->default_value(fullVersion)->required()
             , "Server full version (string).")

            ((prefix + "soapUrl").c_str()
             , po::value(&soapUrl)->default_value(soapUrl)->required()
             , "HTTP SOAP URL.")
            ((prefix + "secureSoapUrl").c_str()
             , po::value(&secureSoapUrl)
             ->default_value(secureSoapUrl)->required()
             , "HTTPS SOAP URL.")
            ((prefix + "owningSystemUrl").c_str()
             , po::value(&owningSystemUrl)
             ->default_value(owningSystemUrl)->required()
             , "URL of the owning system.")

            ((prefix + "isTokenBasedSecurity").c_str()
             , po::value(&isTokenBasedSecurity)
             ->default_value(isTokenBasedSecurity)->required()
             , "True if this server uses token-based security.")
            ((prefix + "tokenServicesUrl").c_str()
             , po::value(&tokenServicesUrl)
             ->default_value(tokenServicesUrl)->required()
             , "URL of Token service.")
            ((prefix + "shortLivedTokenValidity").c_str()
             , po::value(&shortLivedTokenValidity)
             ->default_value(shortLivedTokenValidity)->required()
             , "Validity of short lived tokens.")
            ;
    }

    void configure(const po::variables_map&, const std::string & = "")
    {
        Json::Value value(Json::objectValue);

        value["currentVersion"] = currentVersion;
        value["fullVersion"] = fullVersion;
        addNonEmpty(value["soapUrl"], soapUrl);
        addNonEmpty(value["secureSoapUrl"], secureSoapUrl);
        addNonEmpty(value["owningSystemUrl"], owningSystemUrl);

        auto &authInfo(value["authInfo"] = Json::objectValue);
        authInfo["isTokenBasedSecurity"] = isTokenBasedSecurity;
        if (isTokenBasedSecurity) {
            authInfo["tokenServicesUrl"] = tokenServicesUrl;
            authInfo["shortLivedTokenValidity"]
                = Json::UInt64(shortLivedTokenValidity);
        }

        std::ostringstream os;
        Json::write(os, value, false);
        json = os.str();
    }
};

} // namespace

class I3sd : public Daemon
{
public:
    I3sd()
        : Daemon("i3sd", BUILD_TARGET_VERSION, 3061, defaultConfig())
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

    ServerInfo serverInfo_;
};

void I3sd::configuration(po::options_description &cmdline
                         , po::options_description &config
                         , po::positional_options_description &pd)
{
    serverInfo_.configuration(config, "server-info.");

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
    Daemon::configureImpl(vars);

    serverInfo_.configure(vars, "server-info.");
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

struct SplitPath {
    bool scene;
    std::string dataset;
    std::string resource;

    SplitPath(const fs::path &filePath)
        : scene(false)
    {
        const auto sfilePath(filePath.string());
        if (sfilePath.empty()) { return; }

        auto sceneServer(sfilePath.find(paths::SceneServer));

        if (sceneServer == std::string::npos) {
            if (sfilePath.back() == '/') {
                dataset = sfilePath.substr(0, sfilePath.size() - 1);
                resource = '/';
                return;
            }

            dataset = sfilePath;
            return;
        }

        const auto afterSceneServer(sceneServer + paths::SceneServer.size());
        if ((afterSceneServer < sfilePath.size())
            && sfilePath[afterSceneServer] != '/')
        {
            dataset = sfilePath;
            return;
        }

        dataset = sfilePath.substr(0, sceneServer);
        resource = sfilePath.substr(sceneServer + 1);
        scene = true;
    }
};

void I3sd::handleDataset(DeliveryCache &deliveryCache
                         , const fs::path &filePath
                         , const http::Request &request
                         , Sink sink, const LocationConfig &location)
{
    SplitPath sp(filePath);

    if (!sp.scene) {
        // not a scene path
        if (!location.enableListing) {
            LOG(err1) << "Path " << filePath << " is unlistable.";
            return sink.error
                (utility::makeError<Forbidden>("Unbrowsable"));
        }

        boost::system::error_code ec;
        auto status(fs::status(sp.dataset, ec));

        if (ec) {
            // some error
            if (!fs::exists(status)) {
                LOG(err1) << "Path " << sp.dataset << " doesn't exist.";
                sink.error(utility::makeError<NotFound>
                           ("Path doesn't exist."));
                return;
            }

            sink.error(utility::makeError<InternalError>
                       ("Cannot stat parent path."));
            return;
        }

        if (fs::is_directory(status)) {
            if (sp.resource != "/") {
                return sink.redirect
                    (fs::path(sp.dataset).filename().string() + "/"
                     , utility::HttpCode::Found);
            }

            if (location.enableListing) {
                // directory and we have enabled browser -> directory listing
                sink.listing(sp.dataset);
                return;
            }

            // listing not enabled -> forbidden
            LOG(err1) << "Path " << filePath << " is unlistable.";
            return sink.error
                (utility::makeError<Forbidden>("Unlistable"));
        }

        // file -> handle as driver
    }

    deliveryCache.get
        (sp.dataset
         , [=, &deliveryCache](const DeliveryCache::Expected &value)
         mutable -> void
    {
        try {
            value.get()->handle
                (sink, { sp.resource, request.query }, location);

        } catch (const ListContent &lc) {
            if (location.enableListing) {
                // directory and we have enabled browser -> directory listing
                if (lc.nonFs) {
                    sink.listing(lc.listingBootstrap);
                } else {
                    sink.listing(sp.dataset, lc.listingBootstrap);
                }
                return;
            }
            LOG(err1) << "Path " << filePath << " is unlistable.";
            sink.error
                (utility::makeError<Forbidden>("Unbrowsable"));

        } catch (const RedirectToDir &lc) {
            sink.redirect
                (fs::path(sp.dataset).filename().string() + "/"
                 , utility::HttpCode::Found);

        } catch (const std::system_error &e) {
            LOG(err1) << e.what();
            if (e.code().category() == std::system_category()) {
                if (e.code().value() == ENOENT) {
                    return sink.error
                        (utility::makeError<NotFound>("No such file"));
                }
            }

        } catch (const roarchive::NotAnArchive&) {
            // cannot serve other files than archives
            return sink.error
                (utility::makeError<Forbidden>("Cannot serve as I3S data."));

        } catch (const roarchive::NoSuchFile&) {
            // no such file
            return sink.error(utility::makeError<NotFound>("Not found."));

        } catch (std::domain_error &e) {
            LOG(err1) << e.what();
            sink.error
                (utility::makeError<NotFound>("Domain error"));
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
        return sink->content
            (serverInfo_.json
             , Sink::FileInfo(mimes::ApplicationJson)
             .setFileClass(FileClass::config));
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
