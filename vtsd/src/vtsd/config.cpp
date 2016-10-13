#include "utility/format.hpp"

#include "./config.hpp"

namespace po = boost::program_options;

void LocationConfig::configuration(po::options_description &od
                                   , const std::string &prefix)
{
    od.add_options()
        ((prefix + "match").c_str()
         , po::value(&match)->default_value(match)
         , utility::format("Match type, one of {%s}."
                           , enumerationString(Match())).c_str())
        ((prefix + "dataset").c_str()
         , po::value(&enableDataset)->default_value(enableDataset)
         , "Enable dataset (tileset, storage, etc.) handling at "
         "this location.")
        ((prefix + "browser").c_str()
         , po::value(&enableBrowser)->default_value(enableBrowser)
         , "Enable built-in browser at this location.")
        ((prefix + "listing").c_str()
         , po::value(&enableListing)->default_value(enableListing)
         , "Enabled directory listing.")
        ((prefix + "root").c_str()
         , po::value(&root)
         , "Root directory (conflicts with alias).")
        ((prefix + "alias").c_str()
         , po::value(&alias)
         , "Alias this location to another directory (conflicts with root).")
        ((prefix + "vts.builtinBrowserUrl").c_str()
         , po::value(&vars["VTS_BUILTIN_BROWSER_URL"])
         , "URL of built in browser.")
        ;

    fileClassSettings.configuration(od, prefix + "max-age.");
}

void LocationConfig::configure(const po::variables_map &vars
                               , const std::string &prefix)
{
    const auto rootName(prefix + "root");
    const auto aliasName(prefix + "alias");
    const auto rootCount(vars.count(rootName));
    const auto aliasCount(vars.count(aliasName));

    if (rootCount == aliasCount) {
        if (rootCount) {
            // both defined
            throw po::validation_error
                (po::validation_error::multiple_values_not_allowed
                 , rootName + "," + aliasName);
        }

        // none defined
        throw po::validation_error
            (po::validation_error::at_least_one_value_required
             , rootName + "," + aliasName);
    }

    if (match == Match::regex) {
        regex = boost::in_place(location);
    }
}

std::ostream& LocationConfig::dump(std::ostream &os, const std::string &prefix)
    const
{
    os << prefix << "browser = " << enableBrowser << "\n"
       << prefix << "listing = " << enableListing << "\n"
        ;

    if (!root.empty()) {
        os << prefix << "root = " << root << "\n";
    } else {
        os << prefix << "root = none\n";
    }

    if (!alias.empty()) {
        os << prefix << "alias = " << alias << "\n";
    } else {
        os << prefix << "alias = none\n";
    }

    fileClassSettings.dump(os, prefix + "max-age.");

    for (const auto &var : vars) {
        os << prefix << "variable " << var.first << " = "
           << var.second << "\n";
    }
    return os;
}
