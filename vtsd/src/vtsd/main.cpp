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

#include "vts-libs/storage/fstreams.hpp"
#include "vts-libs/storage/error.hpp"
#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts/options.hpp"
#include "vts-libs/vts/support.hpp"

#include "./error.hpp"
#include "./config.hpp"
#include "./delivery/cache.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace vr = vtslibs::registry;
namespace vts = vtslibs::vts;

class Daemon
    : public service::Service
    , public http::ContentGenerator
{
public:
    Daemon()
        : service::Service("vtsd", BUILD_TARGET_VERSION
                           , service::ENABLE_CONFIG_UNRECOGNIZED_OPTIONS
                           | service::ENABLE_UNRECOGNIZED_OPTIONS)
        , httpListen_(3060)
        , httpThreadCount_(boost::thread::hardware_concurrency())
        , coreThreadCount_(boost::thread::hardware_concurrency())
    {
        defaultConfig_.vars = vts::defaultSupportVars;

        openOptions_
            .ioRetries(1)
            .ioWait(60*1000);

        // some file class defaults
        auto &fcs(defaultConfig_.fileClassSettings);
        fcs.setMaxAge(FileClass::config, 60);
        fcs.setMaxAge(FileClass::support, 3600);
        fcs.setMaxAge(FileClass::registry, 3600);
        fcs.setMaxAge(FileClass::data, 604800);
        fcs.setMaxAge(FileClass::unknown, -1);
    }

private:
    virtual void generate_impl(const http::Request &request
                               , const http::ServerSink::pointer &sink);

    void handlePrefix(const LocationConfig &location
                     , const http::Request &request
                     , const http::ServerSink::pointer &sink);

    void handleRegex(const LocationConfig &location
                     , const LocationConfig::MatchResult &m
                     , const http::Request &request
                     , const http::ServerSink::pointer &sink);


    void handle(const fs::path &filePath, const http::Request &request
                , const http::ServerSink::pointer &sink
                , const LocationConfig &location);

    void handleDataset(const fs::path &filePath, const http::Request &request
                       , Sink sink, const LocationConfig &location);

    void handlePlain(const fs::path &filePath, const http::Request &request
                     , Sink sink, const LocationConfig &location);

    bool tryOpen(const fs::path &filePath);

    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    service::UnrecognizedParser::optional
    configure(const po::variables_map &vars
              , const service::UnrecognizedOptions &unrecognized);

    void configure(const po::variables_map &vars);

    std::vector<std::string> listHelps() const;

    bool help(std::ostream &out, const std::string &what) const;

    bool prePersonaSwitch();

    Service::Cleanup start();

    int run();

    void stat(std::ostream &os);

    void cleanup();

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

void Daemon::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    vr::registryConfiguration(config, vr::defaultPath());

    config.add_options()
        ("http.listen", po::value(&httpListen_)
         ->default_value(httpListen_)->required()
         , "TCP endpoint where to listen at.")
        ("http.threadCount", po::value(&httpThreadCount_)
         ->default_value(httpThreadCount_)->required()
         , "Number of server HTTP threads.")
        ("core.threadCount", po::value(&coreThreadCount_)
         ->default_value(coreThreadCount_)->required()
         , "Number of server core threads.")
        ;

    openOptions_.configuration(config, "open.");

    (void) cmdline;
    (void) pd;
}

service::UnrecognizedParser::optional
Daemon::configure(const po::variables_map &vars
                  , const service::UnrecognizedOptions &unrecognized)
{
    service::UnrecognizedParser parser
        ("Location configuration.");

    const std::string optPrefix("location");
    const auto optPrefixDotted(optPrefix + "<");
    const auto optPrefixDashed("--" + optPrefixDotted);

    std::set<std::string> seen;
    std::vector<std::string> locations;

    auto collect([&](const std::string &option, const std::string &prefix)
    {
        if (option.find(prefix) != 0) { return; }
        auto scolon(option.find('>', prefix.size()));
        if (scolon == std::string::npos) { return; }
        auto location(option.substr(prefix.size()
                                    , scolon - prefix.size()));
        if (seen.insert(location).second) {
            locations.push_back(location);
        }
    });

    for (const auto &option : unrecognized.cmdline) {
        collect(option, optPrefixDashed);
    }

    for (const auto &key : unrecognized.seenConfigKeys) {
        collect(key, optPrefixDotted);
    }

    locations_.reserve(locations.size());

    for (const auto &location : locations) {
        locations_.emplace_back(defaultConfig_, location);
        locations_.back().configuration
            (parser.options, "location<" + location + ">.");
    }

    return parser;

    (void) vars;
}

void Daemon::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    if (locations_.empty()) {
        LOGTHROW(err3, std::runtime_error)
            << "Missing location configuration. Please, provide at least one.";
    }

    for (auto &location : locations_) {
        location.configure(vars, "location<" + location.location + ">.");
    }

    openOptions_.configure(vars, "open.");

    // sort locations:
    {
        // grab prefix locations
        std::copy_if(locations_.begin(), locations_.end()
                     , std::back_inserter(prefixLocations_)
                     , [&](const LocationConfig &l) {
                         return l.match == LocationConfig::Match::prefix;
                     });

        // sort them in reverese order (i.e. longest first)
        std::sort(prefixLocations_.begin(), prefixLocations_.end()
                  , [&](const LocationConfig &l, const LocationConfig &r)
        {
            return l.location > r.location;
        });

        // then regex expressions in the order they were configured
        std::copy_if(locations_.begin(), locations_.end()
                     , std::back_inserter(regexLocations_)
                     , [&](const LocationConfig &l) {
                         return l.match == LocationConfig::Match::regex;
                     });
    }

    LOG(info3, log_)
        << std::boolalpha
        << "Config:"
        << "\n\thttp.listen = " << httpListen_
        << "\n\thttp.threadCount = " << httpThreadCount_
        << '\n' << utility::dump(openOptions_, "\topen.")
        << utility::LManip([&](std::ostream &os) {
                for (const auto &location : prefixLocations_) {
                    os << "\n\tlocation <" << location.location << ">:\n";
                    location.dump(os, "\t\t");
                }
            })
        << utility::LManip([&](std::ostream &os) {
                for (const auto &location : regexLocations_) {
                    os << "\n\tlocation <" << location.location << ">:\n";
                    location.dump(os, "\t\t");
                }
            })
        ;
    (void) vars;
}

std::vector<std::string> Daemon::listHelps() const
{
    return { "location" };
}

bool Daemon::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("vtsd daemon\n"
                "\n"
                );

        return true;
    }

    if (what == "location") {
        po::options_description po("per-location configuration");
        LocationConfig tmp(defaultConfig_);
        tmp.configuration(po, "location</path>.");
        out << po;

        return true;
    }

    return false;
}

bool Daemon::prePersonaSwitch()
{
    return false; // no need to keep saved persona
}

service::Service::Cleanup Daemon::start()
{
    auto guard(std::make_shared<Stopper>(*this));

    deliveryCache_ = boost::in_place(coreThreadCount_, openOptions_);

    http_ = boost::in_place();
    http_->serverHeader(utility::format
                        ("%s/%s", utility::buildsys::TargetName
                         , utility::buildsys::TargetVersion));
    http_->listen(httpListen_, std::ref(*this));
    http_->startServer(httpThreadCount_);

    return guard;
}

void Daemon::cleanup()
{
    // destroy delivery cache first
    deliveryCache_ = boost::none;
    http_ = boost::none;
}

void Daemon::stat(std::ostream &os)
{
    os << "TODO: report stat here";
}

int Daemon::run()
{
    try {
        while (Service::isRunning()) {
            ::usleep(500000);
        }
    } catch (AbandonAll) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

void sendListing(const fs::path &path, Sink &sink
                 , const Sink::Listing &bootstrap = Sink::Listing())
{
    http::ServerSink::Listing listing(bootstrap);

    for (fs::directory_iterator ipath(path), epath; ipath != epath; ++ipath) {
        const auto &entry(*ipath);
        listing.emplace_back(entry.path().filename().string()
                             , (is_directory(entry.status())
                                ? http::ServerSink::ListingItem::Type::dir
                                : http::ServerSink::ListingItem::Type::file));
    }

    sink.listing(listing);
}

bool Daemon::tryOpen(const fs::path &filePath)
{
    try {
        deliveryCache_->get(filePath.string());
    } catch (...) {
        // could not open dataset
        return false;
    }
    // dataset opened -> fine
    return true;
}

void Daemon::handle(const fs::path &filePath
                    , const http::Request &request
                    , const http::ServerSink::pointer &sink
                    , const LocationConfig &location)
{
    if (location.enableDataset) {
        return handleDataset
            (filePath, request, Sink(sink, location), location);
    }
    return handlePlain(filePath, request, Sink(sink, location), location);
}

void Daemon::handlePlain(const fs::path &filePath, const http::Request&
                         , Sink sink, const LocationConfig &location)
{
    try {
        // directory handling
        if (is_directory(status(filePath))) {
            auto file(filePath.filename());
            if (file != ".") {
                // directory redirect
                sink.redirect(file.string() + "/", utility::HttpCode::Found);
                return;
            }

            if (location.enableListing) {
                // directory and we have enabled browser -> directory listing
                sendListing(filePath, sink);
                return;
            }

            // listing not enabled -> forbidden
            LOG(err1) << "Path " << filePath << " is unlistable.";
        }

        // TODO: set proper content type

        sink.content(vs::fileIStream("application/octet-stream", filePath)
                     , FileClass::data);
    } catch (const vs::NoSuchFile &e) {
        LOG(err1) << e.what();
        sink.error(utility::makeError<NotFound>("No such file"));
    }
}

void Daemon::handleDataset(const fs::path &filePath, const http::Request&
                           , Sink sink, const LocationConfig &location)
{
    const auto parent(filePath.parent_path());
    const auto file(filePath.filename());

    deliveryCache_->get
        (parent.string()
         , [=, this](const DeliveryCache::Expected &value)
         mutable -> void
    {
        try {
            value.get()->handle(sink, file.string(), location);
        } catch (vs::NoSuchTileSet) {
            boost::system::error_code ec;
            auto status(fs::status(filePath, ec));
            if (ec) {
                // some error
                if (!fs::exists(status)) {
                    LOG(err1) << "Path " << filePath << " doesn't exist.";
                    sink.error(utility::makeError<NotFound>
                               ("Path doesn't exist."));
                    return;
                }

                sink.error(utility::makeError<InternalError>
                           ("Cannot stat parent path."));
                return;
            }

            // file exists
            if (fs::is_directory(status)) {
                if (file != ".") {
                    // directory redirect
                    sink.redirect
                        (file.string() + "/", utility::HttpCode::Found);
                    return;
                }

                if (location.enableListing) {
                    // directory and we have enabled browser -> directory
                    // listing
                    sendListing(parent, sink);
                    return;
                }

                // listing not enabled -> forbidden
                LOG(err1) << "Path " << filePath << " is unlistable.";
                return sink.error
                    (utility::makeError<Forbidden>("Unlistable"));
            }

            // not a directory, check if this is a file-based dataset
            deliveryCache_->get
                (filePath.string()
                 , [=,this](const DeliveryCache::Expected &value)
                 mutable -> void
            {
                if (value) {
                    LOG(info1) << "Non-directory dataset.";
                    // non-directory dataset -> treat as a directory -> redirect
                    return sink.redirect(file.string() + "/"
                                         , utility::HttpCode::Found);
                } else {
                    LOG(err1) << "No dataset found at " << filePath << ".";
                    return sink.error
                        (utility::makeError<NotFound>("No such dataset"));
                }
            });
        } catch (const ListContent &lc) {
            if (location.enableListing) {
                // directory and we have enabled browser -> directory listing
                sendListing(parent, sink, lc.listingBootstrap);
                return;
            }
            LOG(err1) << "Path " << filePath << " is unlistable.";
            sink.error
                (utility::makeError<Forbidden>("Unbrowsable"));

        } catch (const std::system_error &e) {
            LOG(err1) << e.what();
            if (e.code().category() == std::system_category()) {
                if (e.code().value() == ENOENT) {
                    return sink.error
                        (utility::makeError<NotFound>("No such file"));
                }
            }

        } catch (const vs::NoSuchFile &e) {
            LOG(err1) << e.what();
            sink.error
                (utility::makeError<NotFound>("No such file"));

        } catch (std::domain_error &e) {
            LOG(err1) << e.what();
            sink.error
                (utility::makeError<NotFound>("Domain error"));
        }
    });
}

void Daemon::handlePrefix(const LocationConfig &location
                          , const http::Request &request
                          , const http::ServerSink::pointer &sink)
{
    if (!location.root.empty()) {
        // use root
        const auto filePath(location.root / request.path);
        return handle(filePath, request, sink, location);
    }

    // apply alias and handle
    // cut from path
    auto path(request.path.substr(location.location.size()));

    // TODO: check for "./" and "../"!
    const fs::path filePath(location.alias.string() + path);
    handle(filePath, request, sink, location);
}

void Daemon::handleRegex(const LocationConfig &location
                         , const LocationConfig::MatchResult &m
                         , const http::Request &request
                         , const http::ServerSink::pointer &sink)
{
    // matched
    if (!location.root.empty()) {
        // use root
        const auto filePath(location.root / request.path);
        return handle(filePath, request, sink, location);
    }

    // TODO: check for "./" and "../"!
    const fs::path filePath(m.format(location.alias.string()
                                     , boost::format_no_copy));

    handle(filePath, request, sink, location);
}

void Daemon::generate_impl(const http::Request &request
                           , const http::ServerSink::pointer &sink)
{
    const LocationConfig *matchedLocation(nullptr);

    // try prefix locations
    for (const auto &location : prefixLocations_) {
        LOG(debug) << "matching: " << request.path
                   << " against prefix " << location.location;
        if (ba::starts_with(request.path, location.location)) {
            // remember and done
            matchedLocation = &location;
            break;
        }
    }

    // then try regex locations
    LocationConfig::MatchResult m;
    for (const auto &location : regexLocations_) {
        LOG(debug) << "matching: " << request.path
                   << " against regex " << location.location;
        if (boost::regex_search(request.path, m, *location.regex)) {
            // remember and done
            matchedLocation = &location;
            break;
        }
    }

    if (!matchedLocation) {
        throw NotFound("No matching location found.");
    }

    switch (matchedLocation->match) {
    case LocationConfig::Match::prefix:
        return handlePrefix(*matchedLocation, request, sink);

    case LocationConfig::Match::regex:
        return handleRegex(*matchedLocation, m, request, sink);
    }
}

int main(int argc, char *argv[])
{
    return Daemon()(argc, argv);
}
