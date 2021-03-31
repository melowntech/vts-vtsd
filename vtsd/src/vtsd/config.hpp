/**
 * Copyright (c) 2017-2020 Melown Technologies SE
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

#ifndef vtsd_config_hpp_included_
#define vtsd_config_hpp_included_

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>

#include "utility/enum-io.hpp"

#include "vts-libs/storage/support.hpp"
#include "vts-libs/vts/options.hpp"

#include "http/request.hpp"

#include "fileclass.hpp"
#include "format.hpp"

namespace vs = vtslibs::storage;

struct LocationConfig {
    typedef std::vector<LocationConfig> list;
    typedef boost::match_results<std::string::const_iterator> MatchResult;

    enum class Match {
        prefix, regex
    };

    std::string location;
    Match match;
    boost::optional<Format> enableDataset = Format::native;

    bool enableBrowser;
    bool enableListing;
    vs::SupportFile::Vars vars;
    FileClassSettings fileClassSettings;
    boost::filesystem::path root;
    boost::filesystem::path alias;

    /** Valid only if match == Match::regex.
     */
    boost::optional<boost::regex> regex;

    /** Name of HTTP header that contains proxy name.
     */
    boost::optional<std::string> proxyHeader;

    /** List of allowed proxy names. No proxy is always supported.
     */
    std::set<std::string> allowedProxies;

    /** Extra HTTP headers to send with all responses.
     */
    http::Header::list httpHeaders;

    /** Config files (e.g. mapConfig.json, freelayer.json, ... for VTS) file
     *  class. Defaults to FileClass::ephemeral. Use wisely!
     */
    FileClass configClass;

    LocationConfig()
        : match(Match::prefix)
        , enableBrowser(false), enableListing(false)
        , configClass(FileClass::ephemeral)
    {}

    LocationConfig(const LocationConfig &config, const std::string &location)
    {
        *this = config;
        this->location = location;
    }

    void configuration(boost::program_options::options_description &od
                       , const std::string &prefix = "");

    void configure(const boost::program_options::variables_map &vars
                   , const std::string &prefix = "");

    std::ostream& dump(std::ostream &os, const std::string &prefix = "") const;

    bool native() const {
        return enableDataset && (*enableDataset == Format::native);
    }
};

UTILITY_GENERATE_ENUM_IO(LocationConfig::Match,
                         ((prefix))
                         ((regex))
                         )

#endif // vtsd_config_hpp_included_
