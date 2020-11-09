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
#include <boost/algorithm/string/trim.hpp>
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

#include "error.hpp"
#include "config.hpp"
#include "delivery/cache.hpp"
#include "delivery/vts/driver.hpp"
#include "delivery/vts0/driver.hpp"

#include "daemon.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace vr = vtslibs::registry;
namespace vts = vtslibs::vts;

namespace {

LocationConfig defaultConfig()
{
    LocationConfig dc;
    dc.vars = vts::defaultSupportVars;

    // some file class defaults
    auto &fcs(dc.fileClassSettings);
    fcs.setMaxAge(FileClass::config, 60);
    fcs.setMaxAge(FileClass::support, 3600);
    fcs.setMaxAge(FileClass::registry, 3600);
    fcs.setMaxAge(FileClass::data, 604800);

    return dc;
}

} // namespace

class Vtsd : public Daemon
{
public:
    Vtsd()
        : Daemon("vtsd", BUILD_TARGET_VERSION, 3060, defaultConfig()
                 , Flags::needsHttpClient)
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
};

void Vtsd::configuration(po::options_description &cmdline
                         , po::options_description &config
                         , po::positional_options_description &pd)
{
    vr::registryConfiguration(config, vr::defaultPath());

    return Daemon::configurationImpl(cmdline, config, pd);
}

service::UnrecognizedParser::optional
Vtsd::configure(const po::variables_map &vars
                  , const service::UnrecognizedOptions &unrecognized)
{
    return Daemon::configureImpl(vars, unrecognized);
}

void Vtsd::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    return Daemon::configureImpl(vars);
}

std::vector<std::string> Vtsd::listHelps() const
{
    return Daemon::listHelpsImpl();
}

bool Vtsd::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("VTS delivery daemon\n"
                "\n"
                );

        return true;
    }

    return Daemon::helpImpl(out, what);
}

DeliveryCache::OpenDriver Vtsd::openDriver()
{
    return DeliveryCache::OpenDriver
        ([this](const std::string &path, const OpenOptions &openOptions
                , DeliveryCache &cache
                , const DeliveryCache::Callback &callback)
         -> DeliveryCache::Driver
         {
             LOG(info2) << "Opening driver for \"" << path << "\".";

             // try VTS
             try {
                 return openVts(path, openOptions, cache, callback
                                , proxiesConfigured());
             } catch (const vs::NoSuchTileSet&) {}

             // finally try VTS0
             return openVts0(path);
         });
}

boost::optional<std::string>
getProxy(const LocationConfig &location, const http::Request &request)
{
    if (!location.proxyHeader) { return boost::none; }

    if (const auto *header = request.getHeader(*location.proxyHeader)) {
        const auto proxy(ba::trim_copy(*header));
        if (location.allowedProxies.find(proxy)
            == location.allowedProxies.end())
        {
            LOG(warn2)
                << "Proxy <" << proxy << "> not allowed for location <"
                << request.path << ">.";
            return boost::none;
        }
        return proxy;
    }
    return boost::none;
}

namespace {

class CacheErrorHandler
    : public ErrorHandler
    , public std::enable_shared_from_this<CacheErrorHandler>
{
public:
    CacheErrorHandler(DeliveryCache &deliveryCache
                      , const fs::path &filePath
                      , Sink sink, const LocationConfig &location)
        : deliveryCache_(deliveryCache), filePath_(filePath)
        , sink_(sink), location_(location)
    {}

private:
    virtual void handle(const std::exception_ptr &exc);

    DeliveryCache &deliveryCache_;
    const fs::path filePath_;
    Sink sink_;
    const LocationConfig location_;
};

void CacheErrorHandler::handle(const std::exception_ptr &exc)
{
    const auto parent(filePath_.parent_path());
    const auto file(filePath_.filename());

    try {
        // rethrow current exception
        std::rethrow_exception(exc);
    } catch (const vs::NoSuchTileSet&) {
        boost::system::error_code ec;
        auto status(fs::status(filePath_, ec));
        if (ec) {
            // some error
            if (!fs::exists(status)) {
                LOG(err1) << "Path " << filePath_ << " doesn't exist.";
                sink_.error(utility::makeError<NotFound>
                            ("Path doesn't exist."));
                return;
            }

            sink_.error(utility::makeError<InternalError>
                        ("Cannot stat parent path."));
            return;
        }

        // file exists
        if (fs::is_directory(status)) {
            if (file != ".") {
                // directory redirect
                sink_.redirect
                    (file.string() + "/", utility::HttpCode::Found);
                return;
            }

            if (location_.enableListing) {
                // directory and we have enabled browser -> directory
                // listing
                sink_.listing(parent);
                return;
            }

            // listing not enabled -> forbidden
            LOG(err1) << "Path " << filePath_ << " is unlistable.";
            return sink_.error
                (utility::makeError<Forbidden>("Unlistable"));
        }

        // not a directory, check if this is a file-based dataset
        auto self(shared_from_this());
        deliveryCache_.get
            (filePath_.string()
             , location_.enableDataset.value_or(Format::native)
             , [self, this, file](const DeliveryCache::Expected &value)
             mutable -> void
        {
            if (value) {
                LOG(info1) << "Non-directory dataset.";
                // non-directory dataset -> treat as a directory -> redirect
                return sink_.redirect(file.string() + "/"
                                      , utility::HttpCode::Found);
            } else {
                LOG(err1) << "No dataset found at " << filePath_ << ".";
                return sink_.error
                    (utility::makeError<NotFound>("No such dataset"));
            }
        });
    } catch (const ListContent &lc) {
        if (location_.enableListing) {
            // directory and we have enabled browser -> directory listing
            sink_.listing(parent, lc.listingBootstrap);
            return;
        }
        LOG(err1) << "Path " << filePath_ << " is unlistable.";
        sink_.error
            (utility::makeError<Forbidden>("Unbrowsable"));

    } catch (const std::system_error &e) {
        LOG(err1) << e.what();
        if (e.code().category() == std::system_category()) {
            switch (e.code().value()) {
            case ENOENT: case ENOTDIR:
                return sink_.error
                    (utility::makeError<NotFound>("No such file"));
            }
        }

    } catch (const vs::NoSuchFile &e) {
        LOG(err1) << e.what();
        sink_.error
            (utility::makeError<NotFound>("No such file"));

    } catch (std::domain_error &e) {
        LOG(err1) << e.what();
        sink_.error
            (utility::makeError<NotFound>("Domain error"));

    } catch (const std::invalid_argument&) {
        // pass error
        sink_.error();
    } catch (const utility::HttpError&) {
        // pass error
        sink_.error();
    } catch (...) {
        // pass everything else to sink
        sink_.error();
    }
}

} // namespace

void Vtsd::handleDataset(DeliveryCache &deliveryCache
                         , const fs::path &filePath
                         , const http::Request &request
                         , Sink sink, const LocationConfig &location)
{
    deliveryCache.get
        (filePath.parent_path().string()
         , location.enableDataset.value_or(Format::native)
         , [=, &deliveryCache](const DeliveryCache::Expected &value)
         mutable -> void
    {
        auto errorHandler(std::make_shared<CacheErrorHandler>
                          (deliveryCache, filePath, sink, location));

        // handle error or return pointer to value
        if (auto driver = value.get(*errorHandler)) {
            driver->handle
                (sink
                 , Location(filePath.filename().string(), request.query
                            , getProxy(location, request))
                 , location
                 , errorHandler
                 );
        }
    });
}

int main(int argc, char *argv[])
{
    return Vtsd()(argc, argv);
}
