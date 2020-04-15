//
// Visualiser.hpp
// Visualises point clouds in real time on the main thread
//

#ifndef VISUALISER_HPP
#define VISUALISER_HPP

#include <memory>
#include <unordered_map>

#include <pcl/visualization/pcl_visualizer.h>

#include "visualisation/PointCloudListener.hpp"
#include "system/MapDataBase.hpp"

namespace Visualisation {
class Visualiser : public PointCloudListener
{
public:
    /// Create instance of visualiser using the given map database
    /// \param mapDataBase a shared pointer to the map database from which the map will be displayed
    Visualiser(std::shared_ptr<System::MapDataBase> mapDataBase);
    
    ~Visualiser() = default;
    
    /// Run the visualiser. Blocks and returns when user quits.
    void Run();
    
private:
    void InitInternalViewer(size_t id, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    void Update();
    
private:
    bool m_FirstCloudAdded { false };
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_PointCloud;
    std::unordered_map<size_t, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>> m_RGBHandlers;
    pcl::visualization::PCLVisualizer::Ptr m_Viewer;
    std::shared_ptr<System::MapDataBase> m_MapDataBase;
};
}

#endif
