//
// Visualiser.hpp
// Visualises point clouds in real time on the main thread
//

#ifndef VISUALISER_HPP
#define VISUALISER_HPP

#include <memory>

#include <pcl/visualization/pcl_visualizer.h>

#include "system/MapDataBase.hpp"

namespace Visualisation {
class Visualiser
{
public:
    /// Create instance of visualiser using the given map database
    /// \param mapDataBase a shared pointer to the map database from which the map will be displayed
    Visualiser(std::shared_ptr<System::MapDataBase> mapDataBase);
    
    ~Visualiser() = default;
    
    /// Run the visualiser. Blocks and returns when user quits.
    void Run();
    
private:
    void InitInternalViewer();
    
private:
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_PointCloud;
    pcl::visualization::PCLVisualizer::Ptr m_Viewer;
    std::shared_ptr<System::MapDataBase> m_MapDataBase;
};
}

#endif
