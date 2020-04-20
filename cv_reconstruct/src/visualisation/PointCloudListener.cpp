// PointCloudListener.cpp

#include "visualisation/PointCloudListener.hpp"

namespace Visualisation
{

// Point cloud added
void PointCloudListener::PointCloudWasAdded(size_t id, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud)
{
    // add to vector of tuples for addition
    m_AdditionMutex.lock();
    m_CloudsPendingAddition.push(std::make_tuple(id, pointCloud));
    m_AdditionMutex.unlock();
}

// Point cloud deleted
void PointCloudListener::PointCloudWasDeleted(size_t id)
{
    // add id to deletion list
    m_DeletionMutex.lock();
    m_CloudsPendingDeletion.insert(id);
    m_DeletionMutex.unlock();
}

// Point cloud updated
void PointCloudListener::PointCloudWasUpdated(size_t id)
{
    
}

// Get next pending cloud for addition
bool PointCloudListener::GetNextPendingCloudForAddition(size_t& id, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    if (m_AdditionMutex.try_lock())
    {
        if (!m_CloudsPendingAddition.empty())
        {
            id = std::get<0>(m_CloudsPendingAddition.front());
            cloud = std::get<1>(m_CloudsPendingAddition.front());
            m_CloudsPendingAddition.pop();
            
            m_AdditionMutex.unlock();
            return true;
        }
        else {
            m_AdditionMutex.unlock();
        }
    }
    
    return false;
}

// Get next pending cloud for deletion
bool PointCloudListener::GetNextPendingCloudIDForDeletion(size_t& id)
{
    if (m_DeletionMutex.try_lock())
    {
        if (!m_CloudsPendingDeletion.empty())
        {
            id = *m_CloudsPendingDeletion.begin();
            m_CloudsPendingDeletion.erase(m_CloudsPendingDeletion.begin());
            
            m_DeletionMutex.unlock();
            
            return true;
        }
        else {
            m_DeletionMutex.unlock();
        }
    }
    
    return false;
}

}
