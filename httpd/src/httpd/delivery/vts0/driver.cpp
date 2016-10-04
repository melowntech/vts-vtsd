#include "vts-libs/storage/error.hpp"

#include "vts-libs/vts0/driver.hpp"
#include "vts-libs/vts0/support.hpp"
#include "vts-libs/vts0/tileop.hpp"
#include "vts-libs/vts0/driver/tilardriver.hpp"
#include "vts-libs/vts0/config.hpp"

#include "../vadstena-http.h"
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

Vts0FileInfo::Vts0FileInfo(const std::string &p, int flags)
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

    if (flags & TILESET_OPEN_DISABLE_BROWSER) {
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

    virtual void handle(const Sink &sink, const std::string &path
                        , int flags, const Variables::Wrapper &variables);
#if 0
    virtual std::unique_ptr<Handle>
    openFile(LockGuard::OptionalMutex &mutex, const std::string &path
             , int flags, const tileset_Variables::Wrapper &variables);
#endif

private:
    vts0::Driver::pointer driver_;
};

void Vts0Driver::handle(const Sink &sink, const std::string &path
                        , int flags, const Variables::Wrapper &variables)
{
    (void) sink;
    (void) path;
    (void) flags;
    (void) variables;
    throw InternalError("Not implemented yet");
}

#if 0
std::unique_ptr<Handle>
Vts0Driver::openFile(LockGuard::OptionalMutex &mutex, const std::string &path
                    , int flags, const tileset_Variables::Wrapper &variables)
{
    Vts0FileInfo info(path, flags);

    LockGuard guard(mutex);

    switch (info.type) {
    case FileInfo::Type::file:
        return std::unique_ptr<Handle>
            (new StreamHandle
             (driver_->input(info.file), TILESET_FILETYPE_FILE));

    case FileInfo::Type::tileFile:
        return std::unique_ptr<Handle>
            (new StreamHandle
             (driver_->input(info.tileId, info.tileFile)
              , TILESET_FILETYPE_TILE));

    case FileInfo::Type::support:
        return std::unique_ptr<Handle>
            (new BrowserFileHandle(info.support->second, variables));

    default: break;
    }

    LOGTHROW(err1, vs::NoSuchFile)
        << "Unknown file to open: \"" << info.path << "\".";
    throw; // shut up, compiler
}
#endif

} // namespace

DriverWrapper::pointer openVts0(const std::string &path, int flags)
{
    (void) flags;
    return std::make_shared<Vts0Driver>
        (std::make_shared<vts0::TilarDriver>
         (path, vts0::OpenMode::readOnly));
}
