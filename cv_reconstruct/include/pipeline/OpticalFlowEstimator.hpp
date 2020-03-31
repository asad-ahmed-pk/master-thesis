//
// OpticalFlowEstimator.hpp
// Estimates pixel movement using Optical Flow
//

#ifndef OPTICAL_FLOW_ESTIMATOR_HPP
#define OPTICAL_FLOW_ESTIMATOR_HPP

#include <vector>

#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>

namespace Features
{
    class OpticalFlowEstimator
    {
    public:
        /// Create default instance for flow estimation
        OpticalFlowEstimator();
        
        ~OpticalFlowEstimator() = default;
        
        /// Compute pixel correspondences from dense optical flow from image 1 to 2
        /// \param image1 The first image moving towards the second image
        /// \param image2 The second image
        /// \param points1 The computed points of the first image
        /// \param points2 The computed points of the second image (flow from first image)
        /// \param mask1 An optional mask for image 1
        /// \param mask2 An optional mask for image 2
        void EstimateCorrespondingPixels(const cv::Mat& image1, const cv::Mat& image2, std::vector<cv::KeyPoint>& points1, std::vector<cv::KeyPoint>& points2, cv::InputArray mask1 = cv::noArray(), cv::InputArray mask2 = cv::noArray());
        
        /// Compute pixel correspondences from dense optical flow from images in the list
        /// \param images A list of images to compute motion flowing from the first to the second and so on...
        /// \param trackedPoints Will be populated with the common pixels that were seen in ALL images
        /// \param An optional mask to apply on the first image before tracking the flow of pixels
        void EstimateCorrespondingPixels(const std::vector<cv::Mat>& images, std::vector<std::vector<cv::KeyPoint>>& trackedPoints, cv::InputArray mask = cv::noArray());
        
    private:
        cv::Ptr<cv::FarnebackOpticalFlow> m_FarnebackOF;
    };
}

#endif
