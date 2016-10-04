#ifndef libvadstena_http_vts0_driver_hpp_included_
#define libvadstena_http_vts0_driver_hpp_included_

#include "vts-libs/vts0/basetypes.hpp"

#include "../driver.hpp"

DriverWrapper::pointer openVts0(const std::string &path, int flags);

#endif // libvadstena_http_vts0_driver_hpp_included_
