//
// FrameFeatureExtractor.hpp
// Extracts 2D features from stereo frames
//

#ifndef MASTER_THESIS_FRAMEFEATUREEXTRACTOR_HPP
#define MASTER_THESIS_FRAMEFEATUREEXTRACTOR_HPP

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

#include "StereoFrame.hpp"

namespace Pipeline
{
    class FrameFeatureExtractor
    {
    public:
        /// Create a default instance of the feature extractor
        FrameFeatureExtractor();

        ~FrameFeatureExtractor() = default;

        /// Compute the correspondences between the 2 images
        /// \param image1 The first image
        /// \param image2 The second image
        /// \param keypoints1 The keypoints detected in the 1st image
        /// \param keypoints2 The keypoints detected in the 2nd image
        /// \param matches Will be populated with matches
        void ComputeCorrespondences(const cv::Mat& image1, const cv::Mat& image2, std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2, std::vector<cv::DMatch>& matches) const;

        /// Get 2D image correspondences by matching keypoints in the given image with pre-computed descriptors
        /// \param descriptors Descriptors to match against
        /// \param image The image to match against the descriptors
        /// \param computedKeypoints Keypoints that will be computed for the image
        /// \param computedDescriptors Descriptors that will be computed for the image
        /// \param matches Will be populated with matches
        void ComputeMatchesWithImage(const cv::Mat& descriptors, const cv::Mat& image, std::vector<cv::KeyPoint>& computedKeypoints, cv::Mat& computedDescriptors, std::vector<cv::DMatch>& matches) const;

        /// Compute features from the given frame
        /// \param image The image to compute features from
        /// \param computedKeypoints Will be populated with computed keypoints in the left cam image
        /// \param computedDescriptors Will be populated with computed descriptors in the left cam image
        void ComputeFeaturesFromImage(const cv::Mat& image, std::vector<cv::KeyPoint>& computedKeypoints, cv::Mat& computedDescriptors) const;

    private:
        void FilterForGoodMatches(const std::vector<std::vector<cv::DMatch>>& matches, std::vector<cv::DMatch>& goodMatches) const;
    private:
        cv::Ptr<cv::Feature2D> m_FeatureExtractor;
        cv::Ptr<cv::BFMatcher> m_BFMatcher;
    };
}

#endif //MASTER_THESIS_FRAMEFEATUREEXTRACTOR_HPP
