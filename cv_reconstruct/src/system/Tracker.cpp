//
// Tracker.cpp
// Tracks frames and estimates transforms between frames
//
#define CERES_FOUND 1

#include <eigen3/Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/sfm/reconstruct.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/common/geometry.h>
#include <pcl/io/pcd_io.h>

#include "system/TrackingFrame.hpp"
#include "system/Tracker.hpp"

#include <opencv2/highgui.hpp>

#define MIN_CORRESPONDENCES_NEEDED 20
#define MIN_DISTANCE_FOR_NEW_KEYFRAME 5.0
#define IMAGE_COUNT_REQUIRED_FOR_SFM 3

namespace System
{
    // Constructor
    Tracker::Tracker(std::shared_ptr<Pipeline::FrameFeatureExtractor> featureExtractor,
                     std::shared_ptr<Reconstruct::Reconstruct3D> reconstructor,
                     std::shared_ptr<MappingSystem> mappingSystem,
                     std::shared_ptr<KeyFrameDatabase> keyFrameDB) : m_FeatureExtractor(std::move(featureExtractor)), m_3DReconstructor(reconstructor), m_MappingSystem(mappingSystem), m_KeyFrameDatabase(keyFrameDB)
    {
        // setup optimsation graph with camera params
        float fx, fy, cx, cy;
        m_3DReconstructor->GetCameraParameters(fx, fy, cx, cy);
        m_OptimisationGraph = std::make_unique<OptimisationGraph>(fx, fy, cx, cy);
        
        // optical flow estimation
        m_OpticalFlowEstimator = std::make_unique<Features::OpticalFlowEstimator>();
    }

    // Track this frame and update estimated position and rotation of the camera
    void Tracker::TrackFrame(std::shared_ptr<TrackingFrame> frame)
    {
        // if no keyframes add this as the first keyframe
        if (m_KeyFrameDatabase->IsEmpty())
        {
            //size_t keyFrameID = m_KeyFrameDatabase->InsertKeyFrame(frame);
            //int poseVertexID = m_OptimisationGraph->AddDefaultCameraPoseVertex(true);
            //m_KeyFramePoseVertexIDs[keyFrameID] = poseVertexID;
            //m_MappingSystem->AddKeyFrame(frame);
            
            // add to image paths for SfM
            m_KeyFrameDatabase->InsertKeyFrame(frame);
            m_TrackedKeyFrames.push_back(frame);
            m_KeyFrameImagePaths.push_back(m_KeyFrameDatabase->GetKeyFrameImagePath(frame->GetID()));
        }
        else {
            // track against last keyframe
            std::shared_ptr<TrackingFrame> recentKeyFrame = m_KeyFrameDatabase->SelectMostRecentKeyFrame();
            return TrackFrame(frame, recentKeyFrame);
        }
    }
    
    // Track between previous frame and this new frame
    void Tracker::TrackFrame(std::shared_ptr<TrackingFrame> currentFrame, std::shared_ptr<TrackingFrame> recentKeyFrame)
    {
        // gps distance for keyframe tracking check
        float distance = currentFrame->DistanceFrom(*recentKeyFrame);
        //std::cout << "\nGPS distance to previous keyframe: " << distance;
        if (distance < MIN_DISTANCE_FOR_NEW_KEYFRAME) {
            return;
        }
        
        // insert into keyframe database as passed certain distance
        m_KeyFrameDatabase->InsertKeyFrame(currentFrame);
        
        // now tracking this keyframe
        m_TrackedKeyFrames.push_back(currentFrame);
        m_KeyFrameImagePaths.push_back(m_KeyFrameDatabase->GetKeyFrameImagePath(currentFrame->GetID()));
        
        // need at least 3 images for SfM
        if (m_KeyFrameImagePaths.size() < IMAGE_COUNT_REQUIRED_FOR_SFM) {
            return;
        }
        
        // SfM for pose estimation
        
        // intrinsics matrix
        float fx, fy, cx, cy;
        m_3DReconstructor->GetCameraParameters(fx, fy, cx, cy);
        cv::Matx33d K = cv::Matx33d( fx, 0, cx, 0, fy, cy, 0, 0, 1);
        
        std::vector<cv::Mat> Rs_est, ts_est, points3d_estimated;
        cv::sfm::reconstruct(m_KeyFrameImagePaths, Rs_est, ts_est, K, points3d_estimated, true);
        
        /*
        std::cout << "\n----------------------------\n" << std::endl;
        std::cout << "Reconstruction: " << std::endl;
        std::cout << "============================" << std::endl;
        std::cout << "Estimated 3D points: " << points3d_estimated.size() << std::endl;
        std::cout << "Estimated cameras: " << Rs_est.size() << std::endl;
        std::cout << "Refined intrinsics: " << std::endl << K << std::endl << std::endl;
        std::cout << "3D Visualization: " << std::endl;
        std::cout << "============================" << std::endl;
        */
        
        // get points
        if (currentFrame->GetID() == 15)
        {
            std::vector<cv::Vec3f> point_cloud_est;
            for (int i = 0; i < points3d_estimated.size(); ++i)
              point_cloud_est.push_back(cv::Vec3f(points3d_estimated[i]));
            
            // save as PCD
            pcl::PointCloud<pcl::PointXYZ> pcd;
            for (auto cvp : point_cloud_est) {
                pcl::PointXYZ p(cvp(0), cvp(1), cvp(2));
                pcd.push_back(p);
            }
            pcl::io::savePCDFileBinary("opencv_sfm_output.pcd", pcd);
        }
        
        
        // keyframes to send to mapper
        std::vector<std::shared_ptr<TrackingFrame>> processedKeyFrames;
        
        // update all poses
        for (size_t i = 0; i < m_TrackedKeyFrames.size(); i++) {
            auto kf = m_TrackedKeyFrames[i];
            kf->SetTrackedPose(PoseFromCVRT(Rs_est[i], ts_est[i]));
        }
        
        // if first time doing this - set all 3 poses
        if (m_TrackedKeyFrames.size() == IMAGE_COUNT_REQUIRED_FOR_SFM) {
            for (size_t i = 0; i < IMAGE_COUNT_REQUIRED_FOR_SFM; i++) {
                //m_TrackedKeyFrames[i]->SetTrackedPose(PoseFromCVRT(Rs_est[i], ts_est[i]));
                processedKeyFrames.push_back(m_TrackedKeyFrames[i]);
            }
        }
        else
        {
            // just set last pose, as first few were already estimated
            //m_TrackedKeyFrames.back()->SetTrackedPose(PoseFromCVRT(Rs_est.back(), ts_est.back()));
            processedKeyFrames.push_back(m_TrackedKeyFrames.back());
        }
        
        // send tracked keyframes to mapper for mapping
        m_MappingSystem->AddKeyFrames(processedKeyFrames);
    }

    // Create eigen 4x4 homogenous transformation matrix from R, and t
    Eigen::Matrix4f Tracker::PoseFromCVRT(const cv::Mat& R, const cv::Mat t) const
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                T(i, j) = R.at<double>(i, j);
            }
            
            T(i, 3) = t.at<double>(i, 0);
        }
        
        return T;
    }

    // Get current tracked pose
    Eigen::Matrix4d Tracker::GetPose() const {
        return m_CurrentPose;
    }
}
