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
    Visualiser::Visualiser(std::shared_ptr<System::MapDataBase> mapDataBase) : m_MapDataBase(mapDataBase) {
        // get access to the point cloud being stored in the database
        m_PointCloud = m_MapDataBase->GetPointCloud();
    }

    // Run main loop
    void Visualiser::Run()
    {
        // wait for point cloud to have some points
        while (m_PointCloud->empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // init internal viewer with point cloud
        InitInternalViewer();

        // viewer main loop
        while (!m_Viewer->wasStopped())
        {
            m_Viewer->spinOnce(100);
            
            // update the internal viewer
            m_Viewer->updatePointCloud(m_PointCloud, POINT_CLOUD_ID);
        }
    }

    // Create internal viewer
    void Visualiser::InitInternalViewer()
    {
        m_Viewer = pcl::visualization::PCLVisualizer::Ptr { new pcl::visualization::PCLVisualizer ("Scene Viewer") };
        m_Viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_PointCloud);
        m_Viewer->addPointCloud<pcl::PointXYZRGB>(m_PointCloud, rgb, POINT_CLOUD_ID);
        m_Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, POINT_CLOUD_ID);
        m_Viewer->addCoordinateSystem(1.0);
        m_Viewer->initCameraParameters();
    }
}
