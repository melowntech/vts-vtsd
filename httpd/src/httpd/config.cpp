#include "./config.hpp"

namespace po = boost::program_options;

void LocationConfig::configuration(po::options_description &od
                                   , const std::string &prefix)
{
    od.add_options()
        ((prefix + "dataset").c_str()
         , po::value(&enableDataset)->default_value(enableDataset)
         , "Handle tile datasets at this location.")
        ((prefix + "browser").c_str()
         , po::value(&enableBrowser)->default_value(enableBrowser)
         , "Enable built-in browser at this location.")
        ((prefix + "vts.builtinBrowserUrl").c_str()
         , po::value(&vars["VTS_BUILTIN_BROWSER_URL"])
         , "URL of built in browser.")
        ;

    fileClassSettings.configuration(od, prefix + "max-age.");
}

void LocationConfig::dump(std::ostream &os, const std::string &prefix) const
{
    os << prefix << "dataset = " << enableDataset << "\n"
       << prefix << "browser = " << enableBrowser << "\n"
        ;
}
