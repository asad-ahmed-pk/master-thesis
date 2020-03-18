//
// MapDatabase.hpp
// Represents the 3D map database and exposes operations for querying the DB
//

#ifndef MASTER_THESIS_MAPDATABASE_HPP
#define MASTER_THESIS_MAPDATABASE_HPP

#include <unordered_map>
#include <memory>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "system/MapBlock.hpp"

namespace System
{
    // Typedefs for the RTree
    namespace RTree {
        typedef boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian> Point3D;
        typedef boost::geometry::model::box<Point3D> Box3D;
        typedef std::pair<Box3D, size_t> Value;
    }

    class MapDataBase
    {
    public:
        /// Create a default instance of the database
        MapDataBase();

        ~MapDataBase() = default;

        /// Add a shared reference to the map block to store in the database
        /// \param block The map block
        /// \return The ID of the newly inserted block
        size_t InsertBlock(std::shared_ptr<MapBlock> block);

        /// Get overlapping blocks that overlap the given block
        /// \param queryBlock The block being queried. Does not have to exist in the database.
        /// \param results Will be populated with a list of pointers to the blocks in the database
        void SelectOverlappingBlocks(const MapBlock& queryBlock, std::vector<std::shared_ptr<MapBlock>>& results);

        /// Get a full scene point cloud from all the current blocks in the database
        /// \param cloud Will be populated with all the points from all current blocks
        void GetFullPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const;

    private:
        boost::geometry::index::rtree<RTree::Value, boost::geometry::index::quadratic<16>> m_RTree;
        std::unordered_map<size_t, std::shared_ptr<MapBlock>> m_Blocks;
        size_t m_NextID { 0 };
    };
}

#endif //MASTER_THESIS_MAPDATABASE_HPP
