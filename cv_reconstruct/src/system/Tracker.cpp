//
// Tracker.cpp
// Tracks frames and estimates transforms between frames
//

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>
#include <pcl/common/geometry.h>

#include "system/TrackingFrame.hpp"
#include "system/Tracker.hpp"

#include <opencv2/highgui.hpp>

#define MIN_CORRESPONDENCES_NEEDED 20
#define MIN_DISTANCE_FOR_NEW_KEYFRAME 0.5

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
            size_t keyFrameID = m_KeyFrameDatabase->InsertKeyFrame(frame);
            int poseVertexID = m_OptimisationGraph->AddDefaultCameraPoseVertex(true);
            m_KeyFramePoseVertexIDs[keyFrameID] = poseVertexID;
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
        // find correspondences between frames
        std::vector<cv::Mat> images { recentKeyFrame->GetCameraImage(), currentFrame->GetCameraImage() };
        std::vector<std::vector<cv::KeyPoint>> keyFrameKeyPoints;
        m_OpticalFlowEstimator->EstimateCorrespondingPixelsv2(images, keyFrameKeyPoints, recentKeyFrame->GetCameraImageMask());
        
        // keyframe pose is already in graph - add (temporarily) the current frame pose to graph
        int currentCameraID = m_OptimisationGraph->AddDefaultCameraPoseVertex(false);
        
        // add common 3D keypoints to graph (points seen by keyframe camera and this frame's camera)
        std::vector<pcl::PointXYZRGB> triangulatedPoints;
        m_3DReconstructor->TriangulatePoints(recentKeyFrame->GetDisparity(), recentKeyFrame->GetCameraImage(), keyFrameKeyPoints[0], triangulatedPoints);
        
        // get last recent keyframe camera ID
        int keyFrameCameraID = m_KeyFramePoseVertexIDs[recentKeyFrame->GetID()];
        
        // add 3D points for these 2 cameras looking at these common 3D points
        std::vector<int> cameras { keyFrameCameraID, currentCameraID };
        m_OptimisationGraph->AddCamerasLookingAtPoints(cameras, triangulatedPoints, keyFrameKeyPoints, true);
        
        // solve for poses
        m_OptimisationGraph->Optimise();
        
        // get pose of current frame vs recent keyframe
        Eigen::Isometry3d currentFramePose = m_OptimisationGraph->GetCameraPose(currentCameraID);
        Eigen::Isometry3d keyFramePose = m_OptimisationGraph->GetCameraPose(keyFrameCameraID);
        
        // update current tracked pose
        m_CurrentPose = currentFramePose.matrix();
        
        // get position difference from last keyframe
        Eigen::Vector3d t1 = currentFramePose.translationExt();
        Eigen::Vector3d t0 = keyFramePose.translationExt();
        float distanceFromKeyFrame = (t1 - t0).norm();
        
        std::cout << "\nDistance from last keyframe: " << distanceFromKeyFrame << std::endl;
        
        // if distance or rotation is above threshold - insert as new keyframe
        // TODO: add rotation check here too
        bool isNowKeyFrame = (distanceFromKeyFrame >= MIN_DISTANCE_FOR_NEW_KEYFRAME);
        
        // exceeded distance threshold - insert new keyframe
        if (isNowKeyFrame)
        {
            // add as keyframe and store optimisation graph id
            size_t id = m_KeyFrameDatabase->InsertKeyFrame(currentFrame);
            m_KeyFramePoseVertexIDs[id] = currentCameraID;
            
            // this camera pose stays - make it fixed if only 2 keyframes exist
            m_OptimisationGraph->SetCameraPoseFixed(currentCameraID, m_KeyFrameDatabase->GetCount() <= 2);
            
            std::cout << "\nAdded keyframe" << std::endl;
            
            // get optimised 3D points
            std::vector<pcl::PointXYZRGB> optimisedPoints;
            m_OptimisationGraph->GetPointsObservedByCamera(currentCameraID, optimisedPoints);
            
            // prepare cloud
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            for (size_t i = 0; i < optimisedPoints.size(); i++) {
                cloud.push_back(optimisedPoints[i]);
            }
            
            // send to mapping system
            std::vector<std::shared_ptr<TrackingFrame>> keyFramesForPoints { recentKeyFrame, currentFrame };
            m_MappingSystem->AddPointsForKeyFrames(cloud, keyFramesForPoints);
        }
        else {
             m_OptimisationGraph->RemoveCameraPoseVertex(currentCameraID);
        }
        
        // delete all temp vertices
        m_OptimisationGraph->RemoveTempEdges();
    }

    // Get current tracked pose
    Eigen::Matrix4d Tracker::GetPose() const {
        return m_CurrentPose;
    }
}
