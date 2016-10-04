#ifndef httpd_delivery_driver_hpp_included_
#define httpd_delivery_driver_hpp_included_

#include "vts-libs/storage/resources.hpp"
#include "vts-libs/storage/streams.hpp"
#include "vts-libs/storage/support.hpp"

#include "utility/raise.hpp"

#include "../sink.hpp"
#include "../config.hpp"

#include "../sink.hpp"

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

    virtual void handle(Sink sink, const std::string &path
                        , const LocationConfig &config) = 0;

    /** Open driver for hot content is never cached.
     */
    virtual bool hotContent() const { return false; }

    Sink::FileInfo fileinfo(const vs::FileStat &fileStat, FileClass fc);
    Sink::FileInfo fileinfo(const vs::FileStat &fileStat
                            , const boost::optional<long> &maxAge
                            = boost::none);
    Sink::FileInfo fileinfo(const vs::SupportFile &file, FileClass fc);
};

// inlines

inline Sink::FileInfo
DriverWrapper::fileinfo(const vs::FileStat &fileStat, FileClass fc)
{
    return Sink::FileInfo(fileStat.contentType, fileStat.lastModified)
        .setFileClass(fc);
}

inline Sink::FileInfo
DriverWrapper::fileinfo(const vs::FileStat &fileStat
                        , const boost::optional<long> &maxAge)
{
    return Sink::FileInfo(fileStat.contentType, fileStat.lastModified
                          , maxAge);
}

#endif // httpd_delivery_driver_hpp_included_
