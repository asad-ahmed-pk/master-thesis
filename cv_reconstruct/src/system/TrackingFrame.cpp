//
// TrackingFrame.cpp
// Represents a frame used for tracking
//

#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>

#include "system/TrackingFrame.hpp"

#include <opencv2/highgui/highgui.hpp>

namespace System
{
    // Constructor
    TrackingFrame::TrackingFrame(const cv::Mat& cameraImage, const cv::Mat& disparity, std::shared_ptr<Reconstruct::Reconstruct3D> reconstructor, const GPS& gps) : m_3DReconstructor(reconstructor), m_GPSLocation(gps)
    {
        cameraImage.copyTo(m_CameraImage);
        disparity.copyTo(m_Disparity);
        SetupFrame();
    }

    // Setup the frame with all required features
    void TrackingFrame::SetupFrame()
    {
        // prune the disparity and set the mask
        m_Mask = cv::Mat(m_CameraImage.rows, m_CameraImage.cols, CV_8U);
        PruneDisparityImage(m_Disparity, m_Mask);
        
        // convert disparity to float range
        m_Disparity.convertTo(m_Disparity, CV_32F, 1.0 / 16.0, 0.0);
        
        // further update mask to only be valid where disparity is not zero
        for (int row = 0; row < m_Mask.rows; row++)
        {
            for (int col = 0; col < m_Mask.cols; col++)
            {
                if (m_Disparity.at<float>(row, col) <= 0.0f) {
                    m_Mask.at<unsigned char>(row, col) = 0;
                }
            }
        }
    }

    // Prune disparity image
    void TrackingFrame::PruneDisparityImage(cv::Mat& disparity, cv::Mat& mask) const
    {
        // compute std dev of disparity and threshold it
        cv::Mat disp; cv::Mat mean; std::vector<double> std;
        cv::normalize(disparity, disp, 0, 255, cv::NORM_MINMAX, CV_8U);

        cv::meanStdDev(disp, mean, std);
        cv::threshold(disp, mask, std[0] * 1.3, 255, cv::THRESH_BINARY);

        // apply mask to disparity and prune
        for (int row = 0; row < mask.rows; row++)
        {
            for (int col = 0; col < mask.cols; col++)
            {
                int value = static_cast<int>(mask.at<unsigned char>(row, col));
                if (value == 0) {
                    disparity.at<short>(row, col) = 0;
                }
            }
        }
    }

    float TrackingFrame::DistanceFrom(const TrackingFrame& other) {
        return m_GPSLocation.DistanceBetweenOtherGPS(other.m_GPSLocation);
    }

    // Dense point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr TrackingFrame::GetDensePointCloud() const {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZRGB>(m_3DReconstructor->Triangulate3D(m_Disparity, m_CameraImage, m_Mask)) };
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZRGB>(m_3DReconstructor->GeneratePointCloud(m_Disparity, m_CameraImage)) };
        return cloud;
    }

    // Getters
    size_t TrackingFrame::GetID() const {
        return m_ID;
    }

    cv::Mat TrackingFrame::GetCameraImageMask() const {
        return m_Mask;
    }

    cv::Mat TrackingFrame::GetCameraImage() const {
        return m_CameraImage;
    }

    cv::Mat TrackingFrame::GetDisparity() const {
        return m_Disparity;
    }

    // Set tracked pose
    void TrackingFrame::SetTrackedPose(const Eigen::Matrix4f& pose) {
        m_EstimatedPose = pose;
    }

    Eigen::Matrix4f TrackingFrame::GetTrackedPose() const {
        return m_EstimatedPose;
    }

    // Set id
    void TrackingFrame::SetID(size_t id) {
        m_ID = id;
    }

    // < operator
    bool TrackingFrame::operator<(const TrackingFrame& other) const {
        return m_ID < other.m_ID;
    }
    
    // == operator
    bool TrackingFrame::operator==(const TrackingFrame& other) const {
        return m_ID == other.m_ID;
    }
}
