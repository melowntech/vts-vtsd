#include <memory>
#include <fstream>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/gccversion.hpp"

#include "vts-libs/registry/po.hpp"

#include "vadstena-http.h"

#include "dbglog/dbglog.hpp"
#include "service/cmdline.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vr = vadstena::registry;
namespace va = vadstena;

std::shared_ptr<std::ofstream> getOut(const boost::optional<fs::path> &path)
{
    if (!path) { return nullptr; }
    auto f(std::make_shared<std::ofstream>());
    f->exceptions(std::ios::badbit | std::ios::failbit);
    f->open(path->string(), std::ios_base::out | std::ios_base::trunc);
    return f;
}

class Test : public service::Cmdline
{
public:
    Test()
        : Cmdline("test-httplib", BUILD_TARGET_VERSION)
        , flags_()
        , variables_(tileset_createVariables())
    {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    fs::path path_;
    boost::optional<fs::path> output_;

    int flags_;
    tileset_Variables *variables_;
};

void Test::configuration(po::options_description &cmdline
                         , po::options_description &config
                         , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("path", po::value(&path_)->required()
         , "Path to file.")
        ("output", po::value<fs::path>()
         , "Path to output file.")
        ("disable-browser", "Disables browser")
        ("enable-vts0-adapter", "Enables TS to VTS0 on the fly adapter.")
        ("var", po::value<std::vector<std::string>>()
         , "Variables in format key=value")
        ;
    vr::registryConfiguration(cmdline, vr::defaultPath());

    pd.add("path", 1);
    pd.add("output", 1);

    (void) config;
}

void Test::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    if (vars.count("output")) {
        output_ = vars["output"].as<fs::path>();
    }

    if (vars.count("disable-browser")) {
        flags_ |= TILESET_OPEN_DISABLE_BROWSER;
    }

    if (vars.count("enable-vts0-adapter")) {
        flags_ |= TILESET_OPEN_ENABLE_VTS0_ADAPTER;
    }

    if (vars.count("var")) {
        for (const auto &pair : vars["var"].as<std::vector<std::string>>()) {
            auto eq(pair.find('='));
            if (eq == std::string::npos) {
                tileset_addVariable(variables_
                                    , pair.c_str(), pair.size()
                                    , "", 0, true);
            } else {
                const auto key(pair.substr(0, eq));
                const auto value(pair.substr(eq + 1));
                tileset_addVariable(variables_
                                    , key.c_str(), key.size()
                                    , value.c_str(), value.size()
                                    , true);
            }
        }
    }
}

bool Test::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(test-vadstena-http test tool
)RAW";
    }
    return false;
}



std::ostream& operator<<(std::ostream &os, tileset_FileType type)
{
    switch (type) {
    case TILESET_FILETYPE_TILE: return os << "tile";
    case TILESET_FILETYPE_FILE: return os << "file";
    case TILESET_FILETYPE_BROWSER: return os << "browser";
    case TILESET_FILETYPE_HOT: return os << "hot";
    default: break;
    }
    return os << "unknown";
}

int Test::run()
{
    auto out(getOut(output_));

    auto *file(tileset_open2(path_.c_str(), flags_, variables_));
    if (!file) {
        LOG(fatal) << "Failed to open file " << path_ << ": "
                   << tileset_errorMessage();
        return EXIT_FAILURE;
    }
    tileset_Stat stat;
    if (-1 == tileset_stat(file, &stat)) {
        LOG(fatal) << "Failed to stat file " << path_ << ": "
                   << tileset_errorMessage();
        if (-1 == tileset_close(file)) {
            LOG(fatal) << "Failed to close file " << path_ << ": "
                   << tileset_errorMessage();
        }
        return EXIT_FAILURE;
    }

    LOG(info4) << "Size: " << stat.size << ".";
    LOG(info4) << "lastModified: " << stat.lastModified << ".";
    LOG(info4) << "contentType: " << stat.contentType << ".";
    LOG(info4) << "fileType: " << stat.fileType << ".";

    char buf[1024];
    while (auto bytes = tileset_read(file, buf, sizeof(buf))) {
        if (bytes == -1) {
            LOG(fatal) << "Failed to read from file " << path_ << ": "
                       << tileset_errorMessage();
            if (-1 == tileset_close(file)) {
                LOG(fatal) << "Failed to close file " << path_ << ": "
                           << tileset_errorMessage();
                return EXIT_FAILURE;
            }
        }
        LOG(info4) << "Read " << bytes << " bytes from file " << path_ << ".";
        if (out) { out->write(buf, bytes); }
    }

    if (-1 == tileset_close(file)) {
        LOG(fatal) << "Failed to close file " << path_ << ": "
                   << tileset_errorMessage();
        return EXIT_FAILURE;
    }

    if (out) { out->close(); }
    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    dbglog::set_mask("ALL");

    tileset_initialize(false);
    tileset_logMask("ALL", -1);
    tileset_logConsole(true);

    auto res(Test()(argc, argv));

    tileset_finish();

    return res;
}
