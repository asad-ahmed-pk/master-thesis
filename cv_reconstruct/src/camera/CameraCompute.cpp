//
// CameraCompute.cpp
// Computation functions for computing camera parameters and matrices
//

#include "camera/CameraCompute.hpp"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace Camera
{
    const float NN_MATCH_RATIO { 0.8f };

    CameraCompute::CameraCompute(Settings::StereoCameraSettings settings) : m_StereoSettings(std::move(settings))
    {

    }

    // Compute F matrix
    Eigen::Matrix3f CameraCompute::FundamentalMatrix(const cv::Mat &leftImage, const cv::Mat &rightImage, const Eigen::Matrix3f& RL, const Eigen::Matrix3f& RR)
    {
        // get matching points in left and right images
        std::vector<cv::Point2f> pointsLeft, pointsRight;
        ComputeMatchingFeatures(leftImage, rightImage, pointsLeft, pointsRight);

        // calculate fundamental matrix
        cv::Mat F_CV = cv::findFundamentalMat(pointsLeft, pointsRight);

        // convert F to Eigen
        Eigen::Matrix3f  F;
        cv::cv2eigen(F_CV, F);

        return std::move(F);
    }

    // Compute E matrix
    Eigen::Matrix3f CameraCompute::EssentialMatrix(const cv::Mat &leftImage, const cv::Mat &rightImage, const Eigen::Matrix3f &KL, const Eigen::Matrix3f &KR)
    {
        // get matching points in left and right images
        std::vector<cv::Point2f> pointsLeft, pointsRight;
        ComputeMatchingFeatures(leftImage, rightImage, pointsLeft, pointsRight);

        // convert to cv matrix
        cv::Mat KL_CV, KR_CV;
        cv::eigen2cv(KL, KL_CV);
        cv::eigen2cv(KR, KR_CV);

        // calculate essential matrix
        cv::Mat E_CV = cv::findEssentialMat(pointsLeft, pointsRight, KL_CV, cv::RANSAC, 0.95);

        // convert E to Eigen
        Eigen::Matrix3f  E;
        cv::cv2eigen(E_CV, E);

        return std::move(E);
    }

    // Find matching points in stereo images
    void CameraCompute::ComputeMatchingFeatures(const cv::Mat& leftImage, const cv::Mat& rightImage,
                                                std::vector<cv::Point2f>& pointsLeft, std::vector<cv::Point2f>& pointsRight)
    {
        // use ORB feature descriptor
        cv::Ptr<cv::ORB> orb = cv::ORB::create();

        std::vector<cv::KeyPoint> keypointsLeft;
        std::vector<cv::KeyPoint> keypointsRight;

        cv::Mat descLeft; cv::Mat descRight;

        // detect keypoints
        orb->detectAndCompute(leftImage, cv::noArray(), keypointsLeft, descLeft);
        orb->detectAndCompute(leftImage, cv::noArray(), keypointsRight, descRight);

        // form correspondences based on 2 nearest neighbours in matches
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> matches;

        // match and set matches lists
        matcher.knnMatch(descLeft, descRight, matches, 2);

        for (size_t i = 0; i < matches.size(); i++)
        {
            cv::DMatch firstMatch = matches[i][0];

            float distance1 = matches[i][0].distance;
            float distance2 = matches[i][1].distance;

            if (distance1 < NN_MATCH_RATIO * distance2) {
                pointsLeft.push_back(keypointsLeft[firstMatch.queryIdx].pt);
                pointsRight.push_back(keypointsRight[firstMatch.trainIdx].pt);
            }
        }
    }
}