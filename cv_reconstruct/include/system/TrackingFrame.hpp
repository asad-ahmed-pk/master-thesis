//
// TrackingFrame.hpp
// Represents a frame used for tracking
// Holds information on colour space, disparity, and 2D features
//

#ifndef MASTER_THESIS_TRACKINGFRAME_HPP
#define MASTER_THESIS_TRACKINGFRAME_HPP

#include <memory>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

#include "reconstruct/Reconstruct3D.hpp"
#include "pipeline/FrameFeatureExtractor.hpp"

namespace System
{
    struct GPS
    {
    public:
        float Latitude;
        float Longitude;
        float Altitude;
        
        float DistanceBetweenOtherGPS(const GPS& other) {
            float distance = DistanceInKmBetweenEarthCoordinates(Latitude, Longitude, other.Latitude, other.Longitude);
            return (distance * 1000);
        }
        
    private:
        float DegreesToRadians(float degrees) {
          return (degrees * M_PI / 180);
        }

        float DistanceInKmBetweenEarthCoordinates(float lat1, float lon1, float lat2, float lon2)
        {
          static float earthRadiusKm = 6371;

          float dLat = DegreesToRadians(lat2 - lat1);
          float dLon = DegreesToRadians(lon2 - lon1);

          float lat1R = DegreesToRadians(lat1);
          float lat2R = DegreesToRadians(lat2);

          float a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1R) * cos(lat2R);
          float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
            
          return earthRadiusKm * c;
        }
    };

    class TrackingFrame
    {
    public:
        /// Construct an instance from the given stereo frame
        /// \param leftImage The camera image (RGB)
        /// \param disparity The disparity image used for depth estimation
        /// \param featureExtractor Shared ptr to a 2D feature extractor
        /// \param reconstructor Shared ptr to a set-up 3D reconstructor
        TrackingFrame(const cv::Mat& cameraImage, const cv::Mat& disparity, std::shared_ptr<Reconstruct::Reconstruct3D> reconstructor, const GPS& gps);

        ~TrackingFrame() = default;
        
        size_t GetID() const;
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetDensePointCloud() const;
        
        cv::Mat GetCameraImageMask() const;
        
        cv::Mat GetCameraImage() const;
        
        cv::Mat GetDisparity() const;
        
        float DistanceFrom(const TrackingFrame& other);
        
        void SetTrackedPose(const Eigen::Matrix4f& pose);
        
        Eigen::Matrix4f GetTrackedPose() const;
        
        void SetID(size_t id);
        
        bool operator<(const TrackingFrame& other) const;
        
        bool operator==(const TrackingFrame& other) const;

    private:
        void PruneDisparityImage(cv::Mat& disparity, cv::Mat& mask) const;
        void SetupFrame();

    private:
        std::shared_ptr<Reconstruct::Reconstruct3D> m_3DReconstructor;

    private:
        size_t m_ID { 0 };
        cv::Mat m_CameraImage;
        cv::Mat m_Disparity;
        cv::Mat m_Mask;
        GPS m_GPSLocation;
        
    private:
        Eigen::Matrix4f m_EstimatedPose = Eigen::Matrix4f::Identity();
    };
}

#endif //MASTER_THESIS_TRACKINGFRAME_HPP
