#include <stdexcept>
#include <sstream>

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"
#include "utility/streams.hpp"

#include "vts-libs/vts0/io.hpp"

#include "./metatile-convert.hpp"
#include "./ts2vts0.hpp"

MetaTree::MetaTree(const ts::Driver::pointer &driver
                   , const ts::Properties &properties)
{
    // get foat
    const auto foatId(asVts(properties, properties.foat));
    // foat starts at LOD=0 -> no need to do anything
    if (!foatId.lod) { return; }

    auto foatFile(driver->input(properties.foat, vs::TileFile::meta));
    stat_ = foatFile->stat();

    typedef std::map<vts0::TileId, vts0::MetaNode> Tree;
    Tree tree;

    ts::loadMetatile
        (*foatFile, properties.baseTileSize, properties.foat
         , [&tree, &properties]
         (const ts::TileId &tileId, const ts::MetaNode &node, std::uint8_t)
         {
             auto tid(asVts(properties, tileId));
             LOG(debug) << "Loaded metanode " << tid << ".";
             tree.insert(Tree::value_type(tid, asVts(node)));
         });

    const auto *foat([&]() -> const vts0::MetaNode*
    {
        auto ftree(tree.find(foatId));
        if (ftree == tree.end()) {
            LOGTHROW(err1, std::runtime_error)
                << "Foat " << foatId << " not in loaded data.";
        }
        return &ftree->second;
    }());

    std::function<void(const vts0::TileId&, const vts0::MetaNode&)>
        process = [&](const vts0::TileId &childId
                      , const vts0::MetaNode &child) -> void
    {
        // done?
        if (!childId.lod) { return; }

        auto nodeId(parent(childId));
        vts0::MetaNode node;
        node.invalidate();
        node.zmin = child.zmin;
        node.zmax = child.zmax;
        node.gsd = child.gsd;
        node.coarseness = child.coarseness;
        LOG(debug) << "Generating virtual metanode " << nodeId << ".";
        tree.insert(Tree::value_type(nodeId, node));
        process(nodeId, node);
    };

    process(foatId, *foat);

    struct Saver : vts0::MetaNodeSaver {
        Saver(const vts0::TileId &foatId, Tree &tree, FileTree &fileTree
              , const vs::FileStat &stat)
            : foatId(foatId), tree(tree), fileTree(fileTree), stat(stat)
        {}

        virtual void saveTile(const vts0::TileId &metaId
                              , const MetaTileSaver &saver)
            const UTILITY_OVERRIDE
        {
            // if foat is on the top of a metatile -> nothing to do
            if (metaId == foatId) { return; }
            std::ostringstream os;
            saver(os);
            fileTree.insert
                (FileTree::value_type(metaId, File(os.str(), stat)));
        }

        virtual const vts0::MetaNode* getNode(const vts0::TileId &tileId)
            const UTILITY_OVERRIDE
        {
            auto ftree(tree.find(tileId));
            if (ftree == tree.end()) { return nullptr; }
            return &ftree->second;
        }

        const vts0::TileId foatId;
        Tree &tree;
        FileTree &fileTree;
        const vs::FileStat stat;
    };

    vts0::saveMetatile({}, asVts(properties.metaLevels)
                      , Saver(foatId, tree, tree_, stat_));
}

const MetaTree::File* MetaTree::file(const vts0::TileId &tileId) const
{
    auto ftree(tree_.find(tileId));
    if (ftree == tree_.end()) { return nullptr; }
    return &ftree->second;
}
