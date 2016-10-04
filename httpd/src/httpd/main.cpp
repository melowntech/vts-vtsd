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
        : service::Service("vtshttpd", BUILD_TARGET_VERSION
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

    void handleDataset(const fs::path &filePath, const http::Request &request
                       , const http::ServerSink::pointer &sink
                       , const LocationConfig &location);

    void handlePlain(const fs::path &filePath, const http::Request &request
                     , const http::ServerSink::pointer &sink
                     , const LocationConfig &location);

    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    service::UnrecognizedParser::optional
    configure(const po::variables_map &vars
              , const service::UnrecognizedOptions &unrecognized);

    void configure(const po::variables_map &vars);

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

    fs::path root_;
    LocationConfig defaultConfig_;
    LocationConfig::list locations_;

    boost::optional<http::Http> http_;

    DeliveryCache deliveryCache_;
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
        ("http.root", po::value(&root_)->required()
         , "Data root.")
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
    const auto optPrefixDotted(optPrefix + ":");
    const auto optPrefixDashed("--" + optPrefixDotted);

    std::set<std::string> locations;

    auto collect([&](const std::string &option, const std::string &prefix)
    {
        if (option.find(prefix) != 0) { return; }
        auto scolon(option.find(':', prefix.size()));
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
            (parser.options, "location:" + location + ":.");
    }

    return parser;

    (void) vars;
}

void Daemon::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    if (std::find_if
        (locations_.begin(), locations_.end()
         , [&](const LocationConfig &l) { return l.location == "/"; })
        == locations_.end())
    {
        // inject default configuration to /
        locations_.push_back(defaultConfig_);
        locations_.back().location = "/";
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

bool Daemon::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("vtshttpd daemon\n"
                "\n"
                );

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

    http_ = boost::in_place();

    http_->listen(httpListen_, std::ref(*this));
    http_->startServer(httpThreadCount_);

    return guard;
}

void Daemon::cleanup()
{
    // TODO: stop machinery
    // destroy, in reverse order
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

void Daemon::handle(const fs::path &filePath
                    , const http::Request &request
                    , const http::ServerSink::pointer &sink
                    , const LocationConfig &location)
{
    LOG(info2) << "Path: " << request.uri << ", file path: " << filePath;

    if (location.enableDataset) {
        handleDataset(filePath, request, sink, location);
    } else {
        handlePlain(filePath, request, sink, location);
    }
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

void Daemon::handleDataset(const fs::path &filePath
                           , const http::Request &request
                           , const http::ServerSink::pointer &sink
                           , const LocationConfig &location)
{
    auto parent(filePath.parent_path());
    auto file(filePath.filename());

    LOG(info4) << "filePath: " << filePath;
    LOG(info4) << "parent: " << parent;
    LOG(info4) << "file: " << file;

    try {
        deliveryCache_.get(parent.string(), 0)
            ->handle(Sink(sink, location.fileClassSettings)
                     , file.string(), location);
    } catch (vs::NoSuchTileSet) {
        if (!exists(filePath)) {
            sink->error(utility::makeError<NotFound>("No such dataset"));
            return;
        }
        auto isDirectory(is_directory(status(filePath)));

        if (isDirectory) {
            if (file != ".") {
                // directory redirect
                sink->seeOther(request.uri + "/");
                return;
            }

            if (location.enableBrowser) {
                // directory and we have enabled browser -> directory listing
                sendListing(parent, sink);
                return;
            }
        }

        // not found
        sink->error(utility::makeError<NotFound>("No such dataset"));
        return;
    } catch (NoBody) {
        if (location.enableBrowser) {
            // directory and we have enabled browser -> directory listing
            sendListing(parent, sink);
            return;
        }
        sink->error(utility::makeError<NotFound>("No such dataset"));
    }
}

void Daemon::handlePlain(const fs::path &filePath
                         , const http::Request &request
                         , const http::ServerSink::pointer &sink
                         , const LocationConfig &location)
{
    throw InternalError("Not implemented yet.");
    (void) filePath;
    (void) request;
    (void) sink;
    (void) location;
}

void Daemon::generate_impl(const http::Request &request
                           , const http::ServerSink::pointer &sink)
{
    const auto filePath(boost::filesystem::path(root_) / request.uri);

    for (const auto location : locations_) {
        if (ba::starts_with(request.uri, location.location)) {
            handle(filePath, request, sink, location);
            return;
        }
    }
    throw NotFound("No location handler found.");
}

int main(int argc, char *argv[])
{
    return Daemon()(argc, argv);
}
