// feature_match_test.cpp
// Match features using Optical Flow

#include <string>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <random>
#include <iostream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pipeline/OpticalFlowEstimator.hpp"

#define NUM_KEYFRAMES 5
#define SAMPLE_SIZE 20
#define IMAGE_FILE_PREFIX "keyframe_"

void RunOpticalFlowOnNImages(const std::string& prefix, const std::vector<cv::Mat>& images);

int main(int argc, char** argv)
{
    // seed random number generator
    std::srand(std::time(nullptr));
    
    // read in all 5 keyframes
    std::vector<cv::Mat> images;
    for (int i = 0; i < NUM_KEYFRAMES; i++) {
        images.emplace_back(cv::imread(IMAGE_FILE_PREFIX + std::to_string(i) + ".png"));
    }
    
    // prepare vector of first 2 for test 1
    std::vector<cv::Mat> first2 { images[0], images[1] };
    RunOpticalFlowOnNImages("two_images", first2);
    
    // run on N images
    RunOpticalFlowOnNImages("n_images", images);
    
    std::cout << std::endl;
    return 0;
}

void RunOpticalFlowOnNImages(const std::string& prefix, const std::vector<cv::Mat>& images)
{
    // prepare optical flow data
    std::vector<std::vector<cv::KeyPoint>> keypoints;
    Features::OpticalFlowEstimator opticalFlow;
    
    // run optical flow on N images
    std::cout << "\nRunning Optical Flow";
    opticalFlow.EstimateCorrespondingPixelsv2(images, keypoints);
    std::cout << "\nDone" << std::endl;
    
    // generate random indices
    std::vector<size_t> sampleIndices;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        sampleIndices.push_back(std::rand() % keypoints[0].size());
    }
    
    // draw keypoints for each image
    for (int i = 0; i < images.size(); i++)
    {
        // get sampled keypoints for this image
        std::vector<cv::KeyPoint> sampledKeyPoints;
        for (int j = 0; j < sampleIndices.size(); j++) {
            sampledKeyPoints.push_back(keypoints[i][sampleIndices[j]]);
        }
        
        // save keypoints to disk
        cv::Mat output;
        cv::drawKeypoints(images[i], sampledKeyPoints, output);
        cv::imwrite(prefix + "_keypoints_" + std::to_string(i) + ".png", output);
    }
}
