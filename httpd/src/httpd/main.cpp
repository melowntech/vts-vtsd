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

#include "vts-libs/storage/error.hpp"
#include "vts-libs/registry/po.hpp"

#include "./error.hpp"
#include "./config.hpp"
#include "./delivery/cache.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace vr = vadstena::registry;

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
    {
        defaultConfig_.vars["VTS_BUILTIN_BROWSER_URL"]
            = "//cdn.melown.com/libs/melownjs/builtin/stable";

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

    void handle(const fs::path &filePath, const http::Request &request
                , const http::ServerSink::pointer &sink
                , const LocationConfig &location);

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

    LocationConfig defaultConfig_;
    LocationConfig::list locations_;

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
        ;

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

    std::set<std::string> locations;

    auto collect([&](const std::string &option, const std::string &prefix)
    {
        if (option.find(prefix) != 0) { return; }
        auto scolon(option.find('>', prefix.size()));
        if (scolon == std::string::npos) { return; }
        auto location(option.substr(prefix.size()
                                    , scolon - prefix.size()));
        locations.insert(location);
    });

    for (const auto &option : unrecognized.cmdline) {
        collect(option, optPrefixDashed);
    }

    for (const auto &config : unrecognized.config) {
        for (const auto &options : config) {
            collect(options.first, optPrefixDotted);
        }
    }

    locations_.reserve(locations.size());

    for (const auto &location : locations) {
        locations_.push_back(defaultConfig_);
        locations_.back().location = location;
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

    // sort locations in reverse
    std::sort(locations_.begin(), locations_.end()
              , [&](const LocationConfig &l, const LocationConfig &r)
    {
        return l.location > r.location;
    });

    LOG(info3, log_)
        << std::boolalpha
        << "Config:"
        << "\n\thttp.listen = " << httpListen_
        << "\n\thttp.threadCount = " << httpThreadCount_
        << utility::LManip([&](std::ostream &os) {
                for (const auto &location : locations_) {
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

    deliveryCache_ = boost::in_place();

    http_ = boost::in_place();
    http_->listen(httpListen_, std::ref(*this));
    http_->startServer(httpThreadCount_);

    return guard;
}

void Daemon::cleanup()
{
    // destroy, in reverse order
    http_ = boost::none;
    deliveryCache_ = boost::none;
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

void sendListing(const fs::path &path, const http::ServerSink::pointer &sink)
{
    http::ServerSink::Listing listing;

    for (fs::directory_iterator ipath(path), epath; ipath != epath; ++ipath) {
        const auto &entry(*ipath);
        listing.emplace_back(entry.path().filename().string()
                             , (is_directory(entry.status())
                                ? http::ServerSink::ListingItem::Type::dir
                                : http::ServerSink::ListingItem::Type::file));
    }

    sink->listing(listing);
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
                    , const http::Request&
                    , const http::ServerSink::pointer &sink
                    , const LocationConfig &location)
{
    auto parent(filePath.parent_path());
    auto file(filePath.filename());

    try {
        deliveryCache_->get(parent.string())
            ->handle(Sink(sink, location), file.string(), location);
    } catch (vs::NoSuchTileSet) {
        if (!exists(filePath)) {
            sink->error(utility::makeError<NotFound>("Path doesn't exist."));
            return;
        }

        if (is_directory(status(filePath))) {
            if (file != ".") {
                // directory redirect
                sink->seeOther(file.string() + "/");
                return;
            }

            if (location.enableListing) {
                // directory and we have enabled browser -> directory listing
                sendListing(parent, sink);
                return;
            }

            // browser not enabled -> forbidden
            return sink->error(utility::makeError<Forbidden>("Unbrowsable"));
        } else if (tryOpen(filePath)) {
            // non-directory dataset -> treat as a directory -> redirect
            return sink->seeOther(file.string() + "/");
        }

        // not found
        sink->error(utility::makeError<NotFound>("No such dataset"));
        return;
    } catch (NoBody) {
        if (location.enableListing) {
            // directory and we have enabled browser -> directory listing
            sendListing(parent, sink);
            return;
        }
        sink->error(utility::makeError<Forbidden>("Unbrowsable"));
    } catch (const std::system_error &e) {
        LOG(err1) << e.what();
        if (e.code().category() == std::system_category()) {
            if (e.code().value() == ENOENT) {
                return sink->error
                    (utility::makeError<NotFound>("No such file"));
            }
        }
    } catch (const vs::NoSuchFile &e) {
        LOG(err1) << e.what();
        sink->error(utility::makeError<NotFound>("No such file"));

    } catch (std::domain_error &e) {
        LOG(err1) << e.what();
        sink->error(utility::makeError<NotFound>("Domain error"));
    }
}

void Daemon::generate_impl(const http::Request &request
                           , const http::ServerSink::pointer &sink)
{
    for (const auto location : locations_) {
        if (ba::starts_with(request.path, location.location)) {
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
            return handle(filePath, request, sink, location);
        }
    }
    throw NotFound("No location handler found.");
}

int main(int argc, char *argv[])
{
    return Daemon()(argc, argv);
}
