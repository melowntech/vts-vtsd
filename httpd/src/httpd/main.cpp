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
#include "utility/tcpendpoint-io.hpp"
#include "utility/buildsys.hpp"
#include "service/service.hpp"

#include "http/http.hpp"

#include "./error.hpp"
#include "./config.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

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

    LocationConfig defaultConfig_;

    boost::optional<http::Http> http_;
};

void Daemon::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    config.add_options()
        ("http.listen", po::value(&httpListen_)
         ->default_value(httpListen_)->required()
         , "TCP endpoint where to listen at.")
        ("http.threadCount", po::value(&httpThreadCount_)
         ->default_value(httpThreadCount_)->required()
         , "Number of server HTTP threads.")
        ;

    defaultConfig_.configuration(config, "default.", true);

    (void) cmdline;
    (void) pd;
}

const std::string RBPrefix("resource-backend");
const std::string RBPrefixDotted(RBPrefix + ".");

service::UnrecognizedParser::optional
Daemon::configure(const po::variables_map &vars
                  , const service::UnrecognizedOptions &unrecognized)
{
#if 0
    // configure resource backend
    const auto RBType(RBPrefixDotted + "type");
    if (!vars.count(RBType)) { return {}; }
    try {
        // fetch backend type
        resourceBackendConfig_.type = vars[RBType].as<std::string>();
        // and configure
        return ResourceBackend::configure
            (RBPrefixDotted, resourceBackendConfig_, unrecognized);
    } catch (const UnknownResourceBackend&) {
        throw po::validation_error
            (po::validation_error::invalid_option_value, RBType);
    }
#endif
    return {};
    (void) vars;
    (void) unrecognized;
}

void Daemon::configure(const po::variables_map &vars)
{
    (void) vars;
    LOG(info3, log_)
        << "Config:"
        << "\n\thttp.listen = " << httpListen_
        << "\n\thttp.threadCount = " << httpThreadCount_
        ;
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

void Daemon::generate_impl(const http::Request &request
                           , const http::ServerSink::pointer &sink)
{
    (void) request;
    (void) sink;
    throw InternalError("Not implemented yet");
}

int main(int argc, char *argv[])
{
    return Daemon()(argc, argv);
}
