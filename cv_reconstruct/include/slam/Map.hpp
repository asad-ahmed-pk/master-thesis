//
// Map.hpp
// Acts as an interface to the SLAM map
//

#ifndef MASTER_THESIS_MAP_HPP
#define MASTER_THESIS_MAP_HPP

#include <unordered_map>

#include <eigen3/Eigen/Eigen>

namespace SLAM
{
    class Map
    {
    public:
        Map();
        ~Map();

    private:
        std::unordered_map<int, Eigen::Vector3i> m_RGBMap;
    };
}

#endif //MASTER_THESIS_MAP_HPP
