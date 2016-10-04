#include "./config.hpp"

namespace po = boost::program_options;

void LocationConfig::configuration(po::options_description &od
                                   , const std::string &prefix
                                   , bool required)
{
    if (required) {
        od.add_options()
            ((prefix + "root").c_str()
             , po::value(&root)->required()
             , "Root directory.");
    } else {
        od.add_options()
            ((prefix + "root").c_str()
             , po::value(&root)->default_value(root)
             , "Root directory.");
    }

    od.add_options()
        ((prefix + "enableDataset").c_str()
         , po::value(&enableDataset)->default_value(enableDataset)
         , "Handle tile datasets at this location.")
        ((prefix + "enableBrowser").c_str()
         , po::value(&enableBrowser)->default_value(enableBrowser)
         , "Enable built-in browser at this location.")
        ((prefix + "vts.builtinBrowserUrl").c_str()
         , po::value(&vars["VTS_BUILTIN_BROWSER_URL"])
         , "URL of built in browser.")
        ;

    fileClassSettings.configuration(od, prefix + "max-age.");
}
