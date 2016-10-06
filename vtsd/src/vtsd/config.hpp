#ifndef httpd_config_hpp_included_
#define httpd_config_hpp_included_

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>

#include "utility/enum-io.hpp"

#include "vts-libs/storage/support.hpp"

#include "./fileclass.hpp"

namespace vs = vadstena::storage;

struct LocationConfig {
    typedef std::vector<LocationConfig> list;
    typedef boost::match_results<std::string::const_iterator> MatchResult;

    enum class Match {
        prefix, regex
    };

    std::string location;
    Match match;
    bool enableDataset;
    bool enableBrowser;
    bool enableListing;
    vs::SupportFile::Vars vars;
    FileClassSettings fileClassSettings;
    boost::filesystem::path root;
    boost::filesystem::path alias;

    /** Valid only if match == Match::regex.
     */
    boost::optional<boost::regex> regex;

    LocationConfig()
        : match(Match::prefix), enableDataset(true)
        , enableBrowser(false), enableListing(false)
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
};

UTILITY_GENERATE_ENUM_IO(LocationConfig::Match,
                         ((prefix))
                         ((regex))
                         )

#endif // httpd_config_hpp_included_
