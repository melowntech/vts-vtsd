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
#include "./delivery/vts/driver.hpp"
#include "./delivery/vts0/driver.hpp"

#ifdef VTSLIBS_HAS_TILESTORAGE
#  include "./delivery/tileset/driver.hpp"
#endif

#include "./daemon.hpp"

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
    fcs.setMaxAge(FileClass::unknown, 1);

    return dc;
}

} // namespace

class Vtsd : public Daemon
{
public:
    Vtsd()
        : Daemon("vtsd", BUILD_TARGET_VERSION, 3060, defaultConfig())
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
        ([](const std::string &path, const OpenOptions &openOptions
            , DeliveryCache &cache, const DeliveryCache::Callback &callback)
         -> DeliveryCache::Driver
         {
             LOG(info2) << "Opening driver for \"" << path << "\".";

#ifdef VTSLIBS_HAS_TILESTORAGE
             // Legacy tilestorage support; tilestorage got an eviction notice
             // but still lingering here...

             // try VTS
             try {
                 return openVts(path, openOptions, cache, callback);
             } catch (vs::NoSuchTileSet) {}

             // try VTS0
             try { return openVts0(path); } catch (vs::NoSuchTileSet) {}

             // finaly, try old TS
             return openTileSet(path);

#else
             // try VTS
             try {
                 return openVts(path, openOptions, cache, callback);
             } catch (vs::NoSuchTileSet) {}

             // finally try VTS0
             return openVts0(path);
#endif

         });
}

void Vtsd::handleDataset(DeliveryCache &deliveryCache
                         , const fs::path &filePath
                         , const http::Request &request
                         , Sink sink, const LocationConfig &location)
{
    const auto parent(filePath.parent_path());
    const auto file(filePath.filename());

    deliveryCache.get
        (parent.string()
         , [=, &deliveryCache](const DeliveryCache::Expected &value)
         mutable -> void
    {
        try {
            value.get()->handle
                (sink, { file.string(), request.query }, location);
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
                    sink.listing(parent);
                    return;
                }

                // listing not enabled -> forbidden
                LOG(err1) << "Path " << filePath << " is unlistable.";
                return sink.error
                    (utility::makeError<Forbidden>("Unlistable"));
            }

            // not a directory, check if this is a file-based dataset
            deliveryCache.get
                (filePath.string()
                 , [=](const DeliveryCache::Expected &value)
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
                sink.listing(parent, lc.listingBootstrap);
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

        } catch (const utility::HttpError&) {
            // pass error
            sink.error();
        }
    });
}

int main(int argc, char *argv[])
{
    return Vtsd()(argc, argv);
}
