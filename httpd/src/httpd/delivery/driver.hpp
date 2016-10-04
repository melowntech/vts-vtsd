#ifndef libvadstena_http_driver_hpp_included_
#define libvadstena_http_driver_hpp_included_

#include "vts-libs/storage/resources.hpp"
#include "vts-libs/storage/streams.hpp"
#include "vts-libs/storage/support.hpp"

#include "./vadstena-http.h"

#include "../sink.hpp"

#include "./lock.hpp"
#include "./variables.hpp"

namespace vs = vadstena::storage;

/** Can be thrown by handler to force redirect.
 */
struct NoBody {};

struct FileInfo {
    enum class Type {
        unknown, file, tileFile, support, definition, dirs
    };

    std::string path;

    // type of file
    Type type;

    // tileset file, valid only when (type == Type::file)
    vs::File file;

    // tileset tile file, valid only when (type == Type::tileFile)
    vs::TileFile tileFile;

    // support file, valid only when (type == Type::support)
    const vs::SupportFile::Files::value_type *support;

    FileInfo(const std::string &path)
        : path(path), type(), file(), tileFile(), support()
    {}
};

class DriverWrapper : boost::noncopyable {
public:
    typedef std::shared_ptr<DriverWrapper> pointer;

    DriverWrapper() {}
    virtual ~DriverWrapper() {}

    virtual vs::Resources resources() const = 0;
    virtual bool externallyChanged() const = 0;

    virtual void handle(const Sink &sink, const std::string &path
                        , int flags, const Variables::Wrapper &variables) = 0;

    /** Open driver for hot content is never cached.
     */
    virtual bool hotContent() const { return false; }
};

#endif // libvadstena_http_driver_hpp_included_
