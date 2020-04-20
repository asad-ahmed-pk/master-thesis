//
// Visualiser.hpp
// Visualises point clouds in real time on the main thread
//

#ifndef VISUALISER_HPP
#define VISUALISER_HPP

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <pcl/visualization/pcl_visualizer.h>

#include "visualisation/PointCloudListener.hpp"
#include "system/MapDataBase.hpp"
#include "system/KeyFrameDatabase.hpp"

namespace Visualisation {
class Visualiser : public PointCloudListener
{
public:
    /// Create instance of visualiser using the given map database
    /// \param mapDataBase a shared pointer to the map database from which the map will be displayed
    Visualiser(std::shared_ptr<System::MapDataBase> mapDataBase, std::shared_ptr<System::KeyFrameDatabase> kfDataBase);
    
    ~Visualiser() = default;
    
    /// Run the visualiser. Blocks and returns when user quits.
    void Run();
    
private:
    void InitInternalViewer(size_t id, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    void Update();
    
private:
    bool m_FirstCloudAdded { false };
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_PointCloud;
    std::unordered_set<size_t> m_PointCloudsInViewer;
    pcl::visualization::PCLVisualizer::Ptr m_Viewer;
    std::shared_ptr<System::MapDataBase> m_MapDataBase;
    std::shared_ptr<System::KeyFrameDatabase> m_KeyFrameDataBase;
};
}

#endif
