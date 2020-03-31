//
// OpticalFlowEstimator.cpp
// Estimates pixel movement using Optical Flow
//

#include "pipeline/OpticalFlowEstimator.hpp"

#define NUM_LEVELS 10
#define PYR_SCALE 0.5
#define FAST_PYR false
#define WIN_SIZE 29
#define NUM_ITERS 10
#define POLY_N 7
#define POLY_SIGMA 1.1

namespace Features
{
    // Constructor
    OpticalFlowEstimator::OpticalFlowEstimator() {
        m_FarnebackOF = cv::FarnebackOpticalFlow::create(NUM_LEVELS, PYR_SCALE, FAST_PYR, WIN_SIZE, NUM_ITERS, POLY_N, POLY_SIGMA);
    }

    // Flow from image1 to image2
    void OpticalFlowEstimator::EstimateCorrespondingPixels(const cv::Mat& image1, const cv::Mat& image2, std::vector<cv::KeyPoint>& points1, std::vector<cv::KeyPoint>& points2, cv::InputArray mask1, cv::InputArray mask2)
    {
        // convert to greyscale
        cv::Mat prev; cv::Mat next;
        cv::cvtColor(image1, prev, cv::COLOR_BGR2GRAY);
        cv::cvtColor(image2, next, cv::COLOR_BGR2GRAY);
        
        // setup masks if provided
        bool isMasked1 = (&mask1 != &cv::noArray());
        bool isMasked2 = (&mask2 != &cv::noArray());
        cv::Mat maskImage1; cv::Mat maskImage2;
        
        if (isMasked1) {
            maskImage1 = mask1.getMat();
        }
        if (isMasked2) {
            maskImage2 = mask2.getMat();
        }
        
        // compute flow
        cv::Mat flow;
        m_FarnebackOF->calc(prev, next, flow);
        
        // split into x and y components
        std::vector<cv::Mat> components;
        cv::split(flow, components);
        
        float dx; float dy;
        float x1; float y1;
        for (int y0 = 0; y0 < flow.rows; y0++)
        {
            for (int x0 = 0; x0 < flow.cols; x0++)
            {
                // if not in mask then skip
                if (isMasked1)
                {
                    int value = static_cast<int>(maskImage1.at<unsigned char>(y0, x0));
                    if (value == 0) {
                        continue;
                    }
                }
                
                // amount moved from prev to next
                dx = components[0].at<float>(y0, x0);
                dy = components[1].at<float>(y0, x0);
                
                // new pixel positions in image 2
                x1 = x0 + dx;
                y1 = y0 + dy;
                
                // skip point - has gone out of image
                if (x1 < 0.0f || x1 >= flow.cols) {
                    continue;
                }
                else if (y1 < 0.0f || y1 >= flow.rows) {
                    continue;
                }
                
                // check mask in image 2 after flow
                if (isMasked2)
                {
                    int value = static_cast<int>(maskImage2.at<unsigned char>(y1, x1));
                    if (value == 0) {
                        continue;
                    }
                }
                
                // pixel was tracked in both images
                points1.push_back(cv::KeyPoint(x0, y0, 8.0));
                points2.push_back(cv::KeyPoint(x1, y1, 8.0));
            }
        }
    }

    // Multiple image version of flow estimation
    void OpticalFlowEstimator::EstimateCorrespondingPixels(const std::vector<cv::Mat>& images, std::vector<std::vector<cv::KeyPoint>>& trackedPoints, cv::InputArray mask)
    {
        // determine if mask is being applied
        bool isMasked = (&mask != &cv::noArray());
        cv::Mat maskImage;
        if (isMasked) {
            maskImage = mask.getMat();
        }
        
        // need at least 2 images
        if (images.size() < 2) {
            return;
        }
        
        // allocate space for tracked points
        for (size_t i = 0; i < images.size(); i++) {
            trackedPoints.push_back(std::vector<cv::KeyPoint>());
        }
        
        // estimate flow from first image to second image
        EstimateCorrespondingPixels(images[0], images[1], trackedPoints[0], trackedPoints[1]);
        
        // setup second image as greyscale and estimate flow from second image and onwards
        cv::Mat prevImage; cv::Mat nextImage; cv::Mat flow;
        cv::cvtColor(images[1], prevImage, cv::COLOR_BGR2GRAY);
        
        // only 2 images were provided - done here
        if (images.size() == 2) {
            return;
        }
        
        // vector of iterators to remove points that go off in any of the images (want common points)
        std::vector<std::vector<cv::KeyPoint>::iterator> iters {
            trackedPoints[1].begin()
        };
        
        // image bounds
        int width = images[0].cols; int height = images[0].rows;
        
        // begin flow estimation on remaining n - 1 images
        std::vector<cv::Mat> components; float x0, x1, y0, y1;
        for (size_t i = 2; i < images.size(); i++)
        {
            // next image as grey scale
            cv::cvtColor(images[i], nextImage, cv::COLOR_BGR2GRAY);
            
            // dense optical flow
            m_FarnebackOF->calc(prevImage, nextImage, flow);
            cv::split(flow, components);
            
            // iterate using first image points as basis
            auto iter = trackedPoints[0].begin();
            while (iter != trackedPoints[0].end())
            {
                // previous image pixel positions
                x0 = iters.back()->pt.x;
                y0 = iters.back()->pt.y;
                
                // new pixel positions in this image
                x1 = x0 + components[0].at<float>(iter->pt.y, iter->pt.x);
                y1 = y0 + components[1].at<float>(iter->pt.y, iter->pt.x);
                
                // check if out of bounds and remove in all previous images
                if ((x1 < 0.0 || x1 >= width) || (y1 < 0.0 || y1 >= height))
                {
                    // remove in first image
                    iter = trackedPoints[0].erase(iter);
                    
                    // remove in all other images
                    for (size_t j = 0; j < iters.size(); j++) {
                        iters[j] = trackedPoints[j + 1].erase(iters[j]);
                    }
                }
                else
                {
                    // all good insert this point for this image
                    trackedPoints[i].push_back(cv::KeyPoint(x1, y1, 10.0));
                    iter++;
                }
            }
            
            // add iter for this image since done processing
            iters.push_back(trackedPoints[i].begin());
            
            // reset the other iters for next image processing
            for (size_t j = 0; j < iters.size(); j++) {
                iters[j] = trackedPoints[j + 1].begin();
            }
        }
    }
}
