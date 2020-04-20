//
// MappingSystem.cpp
// 3D mapping system that handles the development and refinement of the 3D map
//

#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <set>

#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/highgui/highgui.hpp>

#include "system/MappingSystem.hpp"
#include "pipeline/FrameFeatureExtractor.hpp"

#define MIN_DISTANCE_FOR_NEW_MAP_BLOCK 20.0

#define NUM_BLOCKS_FOR_LOCAL_OPTIMISATION 3

namespace System
{
    // Constructor
    MappingSystem::MappingSystem(std::shared_ptr<Reconstruct::Reconstruct3D> reconstructor, std::shared_ptr<KeyFrameDatabase> keyFrameDB) : m_3DReconstructor(reconstructor), m_KeyFrameDataBase(keyFrameDB)
    {
        // map database
        m_MapDataBase = std::make_shared<MapDataBase>();
        
        // setup optimsation graph with camera params
        float fx, fy, cx, cy;
        m_3DReconstructor->GetCameraParameters(fx, fy, cx, cy);
        m_OptimisationGraph = std::make_unique<OptimisationGraph>(fx, fy, cx, cy);
        
        // setup optical flow estimator
        m_OpticalFlowEstimator = std::make_unique<Features::OpticalFlowEstimator>();
    }

    // Start optimisation thread
    void MappingSystem::StartOptimisationThread()
    {
        // TODO: launch background thread for map refinement
    }

    // Add a keyframe to keyframe list
    void MappingSystem::AddKeyFrame(std::shared_ptr<TrackingFrame> keyFrame)
    {
        // POINT CLOUD ALIGNMENT METHOD
        /*
        std::vector<std::shared_ptr<TrackingFrame>> frames { keyFrame };
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyFramePointCloud = keyFrame->GetDensePointCloud();
        
        // save to disk
        pcl::io::savePCDFileBinary("keyframe_" + std::to_string(keyFrame->GetID()) + ".pcd", *keyFramePointCloud);
        cv::imwrite("keyframe_disparity_" + std::to_string(keyFrame->GetID()) + ".png", keyFrame->GetDisparity());
        
        // reduce noise
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (keyFramePointCloud);
        sor.setMeanK(300);
        sor.setStddevMulThresh(0.3);
        //sor.filter (*keyFramePointCloud);
        
        // downsample and filter using voxel grid
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setInputCloud (keyFramePointCloud);
        voxel_grid.setLeafSize (0.2, 0.2, 0.2);
        //voxel_grid.filter(*keyFramePointCloud);
        
        // more noise removal based on radiius search
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
        outrem.setInputCloud(keyFramePointCloud);
        outrem.setRadiusSearch(1.0);
        outrem.setMinNeighborsInRadius(200);
        //outrem.filter (*keyFramePointCloud);
        
        pcl::io::savePCDFileBinary("keyframe_" + std::to_string(keyFrame->GetID()) + "_filtered.pcd", *keyFramePointCloud);
        
        // first block - no alignment
        if (m_TargetBlock == nullptr) {
            std::shared_ptr<MapBlock> block = std::make_shared<MapBlock>(frames, *keyFramePointCloud);
            m_MapDataBase->InsertBlock(block);
            m_TargetBlock = block;
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZRGB> };
            
            //std::cout << "\ndx: " << deltaX << " " << "dz: " << deltaZ;
            
            // apply last transform to new point cloud
            Eigen::Matrix4f T = m_TransformLast;
            //T(2, 3) = T(2, 3) + (2 * deltaZ);
            //T(0, 3) = T(0, 3) + (2 * deltaX);
            pcl::transformPointCloud(*keyFramePointCloud, *keyFramePointCloud, T);
            
            // ICP
            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            
            // settings
            // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
            icp.setMaxCorrespondenceDistance (5);
            // Set the maximum number of iterations (criterion 1)
            icp.setMaximumIterations (50);
            // Set the transformation epsilon (criterion 2)
            icp.setTransformationEpsilon (1e-8);
            // Set the euclidean distance difference epsilon (criterion 3)
            icp.setEuclideanFitnessEpsilon (1);
            
            // alignment
            icp.setInputSource(keyFramePointCloud);
            icp.setInputTarget(m_TargetBlock->GetPoints());
            icp.align(*cloud);
            
            // update delta x and z
            float x0 = m_TransformLast(0, 3);
            float z0 = m_TransformLast(2, 3);
            
            //m_TransformLast *= icp.getFinalTransformation();
            m_TransformLast = icp.getFinalTransformation() * m_TransformLast;
            std::cout << "\nEstimated Transform: \n" << m_TransformLast;
            
            float x1 = m_TransformLast(0, 3);
            float z1 = m_TransformLast(2, 3);
            
            deltaX = x1 - x0;
            deltaZ = z1 - z0;
            
            // save as new block
            std::shared_ptr<MapBlock> block = std::make_shared<MapBlock>(frames, *cloud);
            m_MapDataBase->InsertBlock(block);
            
            m_TargetBlock = block;
        }
        */
        
         // OPTICAL FLOW METHOD
        /*
        // main keyframe store
        m_KeyFrames.push_back(keyFrame);
        
        // optimisation list
        m_KeyFramesForOptimisation.push_back(keyFrame);
        if (m_KeyFramesForOptimisation.size() >= NUM_BLOCKS_FOR_LOCAL_OPTIMISATION)
        {
            // local optimisation for these 3 blocks
            
            // create local optimisation graph
            float fx, fy, cx, cy;
            m_3DReconstructor->GetCameraParameters(fx, fy, cx, cy);
            OptimisationGraph graph(fx, fy, cx, cy);
            
            // images from all keyframes
            std::vector<cv::Mat> images;
            
            // camera vertex IDs
            std::vector<int> cameraGraphIDs;
            
            // add these N keyframes to graph
            for (size_t i = 0; i < m_KeyFramesForOptimisation.size(); i++)
            {
                std::shared_ptr<TrackingFrame> kf = m_KeyFramesForOptimisation[i];
                
                // first 2 poses are fixed - set to estimated pose if there is one
                if (i < 2)
                {
                    if (m_KeyFramePoses.find(kf->GetID()) != m_KeyFramePoses.end()) {
                        int id = graph.AddDefaultCameraPoseVertex(true, m_KeyFramePoses[kf->GetID()]);
                        cameraGraphIDs.push_back(id);
                    }
                    else {
                        int id = graph.AddDefaultCameraPoseVertex(true);
                        cameraGraphIDs.push_back(id);
                    }
                }
                else {
                    // add remaining poses - not fixed these need to be estimated
                    int id = graph.AddDefaultCameraPoseVertex(false);
                    cameraGraphIDs.push_back(id);
                }
                
                // add image to image list
                images.push_back(kf->GetCameraImage());
            }
            
            // prepare required data for triangulation
            cv::Mat disparity = m_KeyFramesForOptimisation[0]->GetDisparity();
            cv::Mat mask = m_KeyFramesForOptimisation[0]->GetCameraImageMask();
            cv::Mat cameraImage = m_KeyFramesForOptimisation[0]->GetCameraImage();
            
            // get common tracked pixels in these N images
            std::vector<std::vector<cv::KeyPoint>> pixels;
            m_OpticalFlowEstimator->EstimateCorrespondingPixels(images, pixels, mask);
            
            // get observed 3D points
            std::vector<pcl::PointXYZRGB> points;
            m_3DReconstructor->TriangulatePoints(disparity, cameraImage, pixels[0], points);
            
            // add observations to graph
            graph.AddCamerasLookingAtPoints(cameraGraphIDs, points, pixels, false);
            
            // optimise
            graph.Optimise();
            
            // get optimised points from graph
            std::vector<pcl::PointXYZRGB> optimisedPoints;
            graph.GetPointsObservedByCamera(cameraGraphIDs[0], optimisedPoints);
            
            // create map block and insert into map database
            
            // prepare point cloud
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            for (size_t i = 0; i < optimisedPoints.size(); i++) {
                cloud.push_back(optimisedPoints[i]);
            }
            
            // insert block into database
            std::shared_ptr<MapBlock> block = std::make_shared<MapBlock>(m_KeyFramesForOptimisation, cloud);
            m_MapDataBase->InsertBlock(block);
            
            // store optimised poses for keyframes
            for (size_t i = 0; i < m_KeyFramesForOptimisation.size(); i++) {
                m_KeyFramePoses[m_KeyFramesForOptimisation[i]->GetID()] = graph.GetCameraVertexPose(cameraGraphIDs[i]);
            }
            
            // clear all keyframes for optimisation and store last one
            // last 2 keyframes will be fixed and stored next round
            std::shared_ptr<TrackingFrame> lastKF = m_KeyFramesForOptimisation[m_KeyFramesForOptimisation.size()-1];
            std::shared_ptr<TrackingFrame> lastKF2 = m_KeyFramesForOptimisation[m_KeyFramesForOptimisation.size()-2];
            m_KeyFramesForOptimisation.clear();
            m_KeyFramesForOptimisation.push_back(lastKF2);
            m_KeyFramesForOptimisation.push_back(lastKF);
            
            // save BA block to disk
            static int num = 1;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr map { new pcl::PointCloud<pcl::PointXYZRGB>() };
            pcl::io::savePCDFileBinary("BA_BLOCK_" + std::to_string(num) + ".pcd", cloud);
            num++;
            
            std::cout << "\nSaved to disk" << std::endl;
        }
        */
    }

    // New version - Please work :(
    void MappingSystem::AddKeyFrames(const std::vector<std::shared_ptr<TrackingFrame>>& keyFrames)
    {
        std::vector<std::shared_ptr<TrackingFrame>> frames;
        for (auto keyFrame : keyFrames)
        {
            frames.push_back(keyFrame);
            
            // skip keyframes in sets of 5
            if (keyFrame->GetID() > 0 && (keyFrame->GetID() % 5) != 0) {
                return;
            }
            
            /*
            if (m_LastKeyFrameAddedToMap != nullptr)
            {
                float distance = keyFrame->DistanceFrom(*m_LastKeyFrameAddedToMap);
                if (distance < MIN_DISTANCE_FOR_NEW_MAP_BLOCK) {
                    continue;
                }
            }
            */
            
            // pose
            Eigen::Affine3f affine;
            affine.matrix() = keyFrame->GetTrackedPose();
            
            // rotation
            Eigen::Matrix3f rotation = affine.rotation();
            Eigen::Vector3f euler = rotation.eulerAngles(0, 1, 2);
            
            float roll = euler(0) * (180.0 / M_PI);
            float pitch = euler(1) * (180.0 / M_PI);
            float yaw = euler(2) * (180.0 / M_PI);
            
            // print pose
            std::cout << "\n||------------------------------------------------------------------------------------||";
            std::cout << "\n\nKey Frame #" << keyFrame->GetID() << std::endl;
            std::cout << "\nPosition: " << affine.translation().transpose();
            std::cout << "\nRotation (Roll, Pitch, Yaw): " << "(" << roll << ", " << pitch << ", " << yaw << ")";
            std::cout << "\n\n||------------------------------------------------------------------------------------||" << std::endl;
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyFramePointCloud = keyFrame->GetDensePointCloud();
            
            // downsample and filter that shit some moaaar
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
            voxel_grid.setInputCloud (keyFramePointCloud);
            voxel_grid.setLeafSize (0.2, 0.2, 0.2);
            //voxel_grid.filter(*keyFramePointCloud);
            
            // more noise removal, get that shit out of here
            pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
            outrem.setInputCloud(keyFramePointCloud);
            outrem.setRadiusSearch(1.0);
            outrem.setMinNeighborsInRadius(300);
            //outrem.filter (*keyFramePointCloud);
            
            // scale down cloud
            float S = 2.0;
            Eigen::Matrix4f ST = Eigen::Matrix4f::Identity();
            ST (0,0) = ST (0,0) * S;
            ST (1,1) = ST (1,1) * S;
            ST (2,2) = ST (2,2) * S;
            pcl::transformPointCloud(*keyFramePointCloud, *keyFramePointCloud, ST);
            
            // move cloud to estimated pose
            pcl::transformPointCloud(*keyFramePointCloud, *keyFramePointCloud, keyFrame->GetTrackedPose());
            
            // icp if previous point cloud is available
            if (m_TargetBlock != nullptr)
            {
                // ICP
                //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZRGB> };
                pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
                
                // settings
                // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
                icp.setMaxCorrespondenceDistance (5);
                // Set the maximum number of iterations (criterion 1)
                icp.setMaximumIterations (50);
                // Set the transformation epsilon (criterion 2)
                icp.setTransformationEpsilon (1e-8);
                // Set the euclidean distance difference epsilon (criterion 3)
                icp.setEuclideanFitnessEpsilon (1);
                
                // alignment
                icp.setInputSource(keyFramePointCloud);
                icp.setInputTarget(m_TargetBlock->GetPoints());
                //icp.align(*keyFramePointCloud);
            }
            
            // insert block into map database
            std::shared_ptr<MapBlock> block = std::make_shared<MapBlock>(frames, *keyFramePointCloud);
            m_MapDataBase->InsertBlock(block);
            m_LastKeyFrameAddedToMap = keyFrame;
            std::cout << "\nKeyFrame " << "#" << keyFrame->GetID() << " inserted into map";
            
            // debug: sleep for 1 second. Then remove previous point clouds
            //std::this_thread::sleep_for(std::chrono::seconds(2));
            for (size_t i = 1; i < keyFrame->GetID(); i++) {
                //m_MapDataBase->DeleteBlock(i);
            }
            
            frames.clear();
            
            m_TargetBlock = block;
        }
    }

    // Add common point cloud seen from keyframes
    void MappingSystem::AddPointsForKeyFrames(const pcl::PointCloud<pcl::PointXYZRGB>& points, const std::vector<std::shared_ptr<TrackingFrame>>& keyFrames)
    {
        // create map block from points and insert into database
        std::shared_ptr<MapBlock> block = std::make_shared<MapBlock>(keyFrames, points);
        m_MapDataBase->InsertBlock(block);
        
        // push onto unpotimised block list
        m_UnoptimisedBlocks.push_back(block);
        
        // check if local BA is needed
        if (m_UnoptimisedBlocks.size() >= NUM_BLOCKS_FOR_LOCAL_OPTIMISATION) {
            LocalOptimisation();
        }
        
        std::cout << "\nMapping system added 3D points" << std::endl;
    }

    // Get map
    void MappingSystem::GetMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const {
        m_MapDataBase->GetFullPointCloud(cloud);
    }

    // Get map database
    std::shared_ptr<MapDataBase> MappingSystem::GetMapDataBase() const {
        return m_MapDataBase;
    }

    // Local optimisation
    void MappingSystem::LocalOptimisation()
    {
        std::cout << "\nPerforming local optimisation. " << m_UnoptimisedBlocks.size() << " blocks will be optimised" << std::endl;
        
        // extract all keyframes in the unoptimised blocks
        std::set<TrackingFrame> keyFrames;
        
        // add all poses to graph if not already added
        for (std::shared_ptr<MapBlock> block : m_UnoptimisedBlocks)
        {
            for (std::shared_ptr<TrackingFrame> frame : block->GetKeyFrames())
            {
                // if not already added to graph - add it
                if (m_CameraGraphIDs.find(frame->GetID()) == m_CameraGraphIDs.end()) {
                    int cameraVertexID = m_OptimisationGraph->AddDefaultCameraPoseVertex(frame->GetID() < 2);
                    m_CameraGraphIDs[frame->GetID()] = cameraVertexID;
                }
                keyFrames.insert(*frame);
            }
        }
        
        // get common pixels between these poses
        std::vector<int> cameras; std::vector<cv::Mat> images;
        std::vector<std::vector<cv::KeyPoint>> projectedPoints;
        std::vector<pcl::PointXYZRGB> points3D;
        
        for (const TrackingFrame& frame : keyFrames) {
            cameras.push_back(m_CameraGraphIDs[frame.GetID()]);
            images.push_back(frame.GetCameraImage());
        }
        m_OpticalFlowEstimator->EstimateCorrespondingPixels(images, projectedPoints, keyFrames.begin()->GetCameraImageMask());
        
        // project first keyframe's points to 3D (all other keyframes can see this)
        m_3DReconstructor->TriangulatePoints(keyFrames.begin()->GetDisparity(), images[0], projectedPoints[0], points3D);
        
        // add this set of cameras and points to graph for local optimisation
        m_OptimisationGraph->AddCamerasLookingAtPoints(cameras, points3D, projectedPoints, false);
        m_OptimisationGraph->Optimise();
        
        std::cout << "\nWindowed BA complete" << std::endl;
        
        // get 3D points back and merge blocks in map
        // get optimised 3D points
        std::vector<pcl::PointXYZRGB> optimisedPoints;
        m_OptimisationGraph->GetPointsObservedByCamera(cameras[0], optimisedPoints);
        
        // prepare point cloud
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (size_t i = 0; i < optimisedPoints.size(); i++) {
            cloud.push_back(optimisedPoints[i]);
        }
        
        // merge the set of blocks
        m_MapDataBase->MergeBlocks(m_UnoptimisedBlocks, cloud);
        m_UnoptimisedBlocks.clear();
        
        std::cout << "\nLocal BA updated map database" << std::endl;
        
        // debug: save to disk
        /*
        static int num = 1;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map { new pcl::PointCloud<pcl::PointXYZRGB>() };
        pcl::io::savePCDFileBinary("BA_BLOCK_" + std::to_string(num) + ".pcd", cloud);
        num++;
        
        std::cout << "\nSaved to disk" << std::endl;
        */
    }

    // Perform full BA
    void MappingSystem::FullBA()
    {
        // perform full optimisation
        m_OptimisationGraph->Optimise(10);
        
        // get all points and save point cloud
        std::vector<pcl::PointXYZRGB> points;
        std::vector<Eigen::Isometry3d> poses;
        
        m_OptimisationGraph->GetAllPointsAndPoses(points, poses);
        
        // prepare point cloud
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (size_t i = 0; i < points.size(); i++) {
            cloud.push_back(points[i]);
        }
        
        // save to disk
        pcl::io::savePCDFileBinary("full_optimised_cloud.pcd", cloud);
    }
}
