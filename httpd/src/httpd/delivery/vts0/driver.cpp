#include <mutex>

#include "vts-libs/storage/error.hpp"

#include "vts-libs/vts0/driver.hpp"
#include "vts-libs/vts0/support.hpp"
#include "vts-libs/vts0/tileop.hpp"
#include "vts-libs/vts0/driver/tilardriver.hpp"
#include "vts-libs/vts0/config.hpp"

#include "./driver.hpp"

namespace fs = boost::filesystem;
namespace vts0 = vadstena::vts0;
namespace vs = vadstena::storage;

namespace {

namespace constants {
    const std::string Config("mapConfig.json");
    const std::string Self(".");
    const std::string Index("index.html");
}

} // namespace

struct Vts0FileInfo : public FileInfo {
    // tileId, valid only when (type == Type::tileFile)
    vadstena::vts0::TileId tileId;

    Vts0FileInfo(const std::string &path, const LocationConfig &config);
};

Vts0FileInfo::Vts0FileInfo(const std::string &p, const LocationConfig &config)
    : FileInfo(p)
{
    if (vts0::fromFilename(tileId, tileFile, path.c_str())) {
        type = Type::tileFile;
        return;
    }

    if (constants::Config == path) {
        type = Type::file;
        file = vs::File::config;
        return;
    }

    if (!config.enableBrowser) {
        LOG(debug) << "Browser disabled, skipping browser files.";
        return;
    }

    LOG(debug) << "Browser enabled, checking browser files.";

    // translate "." -> index
    if (constants::Self == path) { path = constants::Index; }

    auto fsupport(vts0::supportFiles.find(path));
    if (fsupport != vts0::supportFiles.end()) {
        type = Type::support;
        support = &*fsupport;
    }
}

namespace {

class Vts0Driver : public DriverWrapper
{
public:
    Vts0Driver(const vts0::Driver::pointer &driver)
        : driver_(driver)
    {}

    virtual vs::Resources resources() const {
        return driver_->resources();
    }

    virtual bool externallyChanged() const {
        return driver_->externallyChanged();
    }

    virtual void handle(Sink sink, const std::string &path
                        , const LocationConfig &config);

private:
    vts0::Driver::pointer driver_;
    std::mutex mutex_;
};

void Vts0Driver::handle(Sink sink, const std::string &path
                        , const LocationConfig &config)
{
    Vts0FileInfo info(path, config);

    switch (info.type) {
    case FileInfo::Type::file: {
        std::unique_lock<std::mutex> guard(mutex_);
        return sink.content(driver_->input(info.file), FileClass::config);
    }

    case FileInfo::Type::tileFile:
    {
        std::unique_lock<std::mutex> guard(mutex_);
        return sink.content(driver_->input(info.tileId, info.tileFile)
                            , FileClass::data);
    }

    case FileInfo::Type::support:
        return sink.content(info.support->second);

    default: break;
    }

    // wtf?
    sink.error(utility::makeError<NotFound>("Unknown file."));
}

} // namespace

DriverWrapper::pointer openVts0(const std::string &path, int flags)
{
    (void) flags;
    return std::make_shared<Vts0Driver>
        (std::make_shared<vts0::TilarDriver>
         (path, vts0::OpenMode::readOnly));
}
