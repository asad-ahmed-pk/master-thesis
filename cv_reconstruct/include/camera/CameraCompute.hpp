//
// CameraCompute.hpp
// Computation functions for computing camera parameters and matrices
//

#ifndef CV_RECONSTRUCT_CAMERACOMPUTE_HPP
#define CV_RECONSTRUCT_CAMERACOMPUTE_HPP

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/core.hpp>

#include "camera/StereoCameraSettings.hpp"

namespace Camera
{
    class CameraCompute
    {
    public:
        /// Construct compute module with given stereo camera setup
        /// \param settings The stereo camera setup that will be used to run computations
        CameraCompute(StereoCameraSettings settings);

        /// Compute the fundamental matrix with the given left and right image
        /// \param leftImage The left stereo image
        /// \param rightImage The right stereo image
        /// \return The computed fundamental matrix
        Eigen::Matrix3f FundamentalMatrix(const cv::Mat &leftImage, const cv::Mat &rightImage);

    private:
        StereoCameraSettings m_StereoSettings;
    };
}

#endif //CV_RECONSTRUCT_CAMERACOMPUTE_HPP
