//
// Visualiser.hpp
// Visualises point clouds in real time on the main thread
//

#include <thread>
#include <chrono>

#include "visualisation/Visualiser.hpp"

#define POINT_CLOUD_ID "scene_cloud"

namespace Visualisation
{
    // Constructor
    Visualiser::Visualiser(std::shared_ptr<System::MapDataBase> mapDataBase, std::shared_ptr<System::KeyFrameDatabase> kfDataBase) : PointCloudListener(), m_MapDataBase(mapDataBase), m_KeyFrameDataBase(kfDataBase)
    {
        // get access to the point cloud being stored in the database
        m_PointCloud = m_MapDataBase->GetPointCloud();
        
        // set as listener of DB
        m_MapDataBase->RegisterAsListener(this);
    }

    // Run main loop
    void Visualiser::Run()
    {
        // wait for point cloud to have some points
        while (m_PointCloud->empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // wait until queue has a point cloud if viewer has not been init
        while (m_Viewer == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Update();
        }
        
        // update viewer
        Update();

        // viewer main loop
        while (!m_Viewer->wasStopped())
        {
            // update the internal viewer
            Update();
            m_Viewer->spinOnce(100);
        }
    }
    
    // Main update loop
    void Visualiser::Update()
    {
        size_t id;
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
        
        // update current point cloud poses
        for (auto id : m_PointCloudsInViewer)
        {
            Eigen::Affine3f affine;
            affine.matrix() = m_KeyFrameDataBase->SelectKeyFrame(id)->GetTrackedPose();
            m_Viewer->updatePointCloudPose(std::to_string(id), affine);
        }
        
        // Get clouds pending for addition and add to the viewer
        while (GetNextPendingCloudForAddition(id, cloud))
        {
            if (!m_FirstCloudAdded) {
                InitInternalViewer(id, cloud);
            }
            else {
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
                m_Viewer->addPointCloud(cloud, rgb, std::to_string(id));
                m_Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::to_string(id));
            }
            
            m_PointCloudsInViewer.insert(id);
        }
        
        // Get clouds pending deletion and remove from viewer
        while (GetNextPendingCloudIDForDeletion(id)) {
            m_Viewer->removePointCloud(std::to_string(id));
        }
    }

    // Create internal viewer
    void Visualiser::InitInternalViewer(size_t id, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
    {
        m_Viewer = pcl::visualization::PCLVisualizer::Ptr { new pcl::visualization::PCLVisualizer ("Scene Viewer") };
        m_Viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        m_Viewer->addPointCloud<pcl::PointXYZRGB>(m_PointCloud, rgb, std::to_string(id));
        m_Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::to_string(id));
        m_Viewer->addCoordinateSystem(1.0);
        m_Viewer->initCameraParameters();
        
        m_FirstCloudAdded = true;
    }
}
