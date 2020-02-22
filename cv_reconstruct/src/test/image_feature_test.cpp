//
// image_feature_test.cpp
// Test image feature extraction
//

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pipeline/FrameFeatureExtractor.hpp"

int main(int argc, char** argv)
{
    // read in 2 consecutive images
    cv::Mat image1 = cv::imread("0l.png", cv::IMREAD_COLOR);
    cv::Mat image2 = cv::imread("1l.png", cv::IMREAD_COLOR);

    // extract features
    Pipeline::FrameFeatureExtractor featureExtractor;

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    std::vector<cv::DMatch> matches;

    featureExtractor.ComputeCorrespondences(image1, image2, keypoints1, keypoints2, matches);
    std::cout << "\nFound " << matches.size() << " matches";

    // draw matches
    cv::Mat matchesImage;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matchesImage, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::imshow("Matches", matchesImage);
    cv::waitKey(0);
    cv::imwrite("matches.png", matchesImage);

    std::cout << std::endl;
    return 0;
}

