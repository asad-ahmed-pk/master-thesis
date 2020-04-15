//
// PointCloudListener.hpp
// Listens for changes to point cloud
//

#ifndef POINT_CLOUD_LISTENER_HPP
#define POINT_CLOUD_LISTENER_HPP

#include <tuple>
#include <queue>

#include <unordered_set>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Visualisation
{
class PointCloudListener
{
public:
    /// Default Constructor
    PointCloudListener() = default;
    
    virtual ~PointCloudListener() = default;
    
    /// Notify listener that the point cloud was added
    /// \param id The ID for this point cloud
    /// \param pointCloud The point cloud that was added
    void PointCloudWasAdded(size_t id, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud);
    
    /// Notify listener that a point cloud with ID was deleted
    /// \param id The ID of the point cloud that was deleted
    void PointCloudWasDeleted(size_t id);
    
    /// Get next pending cloud for addition
    /// \param id Will be set to the ID for the next point cloud
    /// \param cloud Will be set to point to a cloud pending addition
    /// \return Returns true if there is a cloud pending
    bool GetNextPendingCloudForAddition(size_t& id, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
    
    /// Get the ID of the cloud that is pending deletion
    /// \param id Will be set to the id of the cloud to be deleted
    /// \return True if there is a valid ID pending
    bool GetNextPendingCloudIDForDeletion(size_t& id);
    
private:
    std::queue<std::tuple<size_t, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>> m_CloudsPendingAddition;
    std::unordered_set<size_t> m_CloudsPendingDeletion;
    std::mutex m_AdditionMutex;
    std::mutex m_DeletionMutex;
};
}

#endif
