#ifndef libvadstena_http_vts0_driver_hpp_included_
#define libvadstena_http_vts0_driver_hpp_included_

#include "vts-libs/vts0/basetypes.hpp"

#include "../driver.hpp"

struct Vts0FileInfo : public FileInfo {
    // tileId, valid only when (type == Type::tileFile)
    vadstena::vts0::TileId tileId;

    Vts0FileInfo(const std::string &path, int flags);
};

DriverWrapper::pointer openVts0(const std::string &path, int flags);

#endif // libvadstena_http_vts0_driver_hpp_included_
