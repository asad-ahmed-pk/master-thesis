//
// PipelineResult.hpp
// Contains the processing result of running a frame through the pipeline process
//

#ifndef MASTER_THESIS_PIPELINERESULT_HPP
#define MASTER_THESIS_PIPELINERESULT_HPP

#include <memory>

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Pipeline
{
    struct PipelineResult
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudLocalized { new pcl::PointCloud<pcl::PointXYZRGB>() };
        cv::Mat DisparityImage;
    };
}

#endif //MASTER_THESIS_PIPELINERESULT_HPP
