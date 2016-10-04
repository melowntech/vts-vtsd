#ifndef libvadstena_http_metatile_convert_hpp_included_
#define libvadstena_http_metatile_convert_hpp_included_

#include <map>

#include "vts-libs/tilestorage/driver.hpp"
#include "vts-libs/tilestorage/metatile.hpp"

#include "vts-libs/vts0/metatile.hpp"

namespace ts = vadstena::tilestorage;
namespace vts0 = vadstena::vts0;
namespace vs = vadstena::storage;

class MetaTree {
public:
    MetaTree(const ts::Driver::pointer &driver
             , const ts::Properties &properties);

    struct File {
        std::string data;
        vs::FileStat stat;

        File(const std::string &data, const vs::FileStat &stat)
            : data(data), stat(stat)
        {}
    };

    const File* file(const vts0::TileId &tileId) const;

private:
    typedef std::map<vts0::TileId, File> FileTree;
    FileTree tree_;
    vs::FileStat stat_;
};

#endif // libvadstena_http_metatile_convert_hpp_included_
