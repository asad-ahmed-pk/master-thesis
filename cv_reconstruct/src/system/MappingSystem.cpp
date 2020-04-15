//
// MappingSystem.cpp
// 3D mapping system that handles the development and refinement of the 3D map
//

#include <memory>
#include <set>

#include "system/MappingSystem.hpp"

#define MIN_OVERLAP_RATIO_NEDDED_FOR_INSERT 0.2f
#define MAX_OVERLAP_RATIO_NEDDED_FOR_INSERT 0.6f

#define NUM_BLOCKS_FOR_LOCAL_OPTIMISATION 5

#include <pcl/io/pcd_io.h>

namespace System
{
    // Constructor
    MappingSystem::MappingSystem(std::shared_ptr<Reconstruct::Reconstruct3D> reconstructor) : m_3DReconstructor(reconstructor)
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
        // perform full 20 optimisations
        m_OptimisationGraph->Optimise(20);
        
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
