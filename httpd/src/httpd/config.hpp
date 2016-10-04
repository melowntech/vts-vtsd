#ifndef httpd_config_hpp_included_
#define httpd_config_hpp_included_

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>

#include "vts-libs/storage/support.hpp"

#include "./fileclass.hpp"

namespace vs = vadstena::storage;

struct LocationConfig {
    std::string location;
    boost::filesystem::path root;
    bool enableDataset;
    bool enableBrowser;
    vs::SupportFile::Vars vars;
    FileClassSettings fileClassSettings;

    LocationConfig()
        : enableDataset(true), enableBrowser(false)
    {}

    void configuration(boost::program_options::options_description &od
                       , const std::string &prefix = ""
                       , bool required = false);

};

#endif // httpd_config_hpp_included_
