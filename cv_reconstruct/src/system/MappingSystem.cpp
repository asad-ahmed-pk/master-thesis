//
// MappingSystem.cpp
// 3D mapping system that handles the development and refinement of the 3D map
//

#include <memory>
#include <set>

#include "system/MappingSystem.hpp"

#define MIN_OVERLAP_RATIO_NEDDED_FOR_INSERT 0.2f
#define MAX_OVERLAP_RATIO_NEDDED_FOR_INSERT 0.6f

#define NUM_BLOCKS_FOR_LOCAL_OPTIMISATION 3

#include <pcl/io/pcd_io.h>

namespace System
{
    // Constructor
    MappingSystem::MappingSystem(std::shared_ptr<Reconstruct::Reconstruct3D> reconstructor) : m_3DReconstructor(reconstructor)
    {
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
        m_MapDataBase.InsertBlock(block);
        
        // check if local optimisation is needed
        m_UnoptimisedBlocks.push_back(block);
        if (m_UnoptimisedBlocks.size() >= NUM_BLOCKS_FOR_LOCAL_OPTIMISATION) {
            LocalOptimisation();
        }
        
        std::cout << "\nMapping system added 3D points" << std::endl;
    }

    // Get map
    void MappingSystem::GetMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const {
        m_MapDataBase.GetFullPointCloud(cloud);
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
                    m_OptimisationGraph->AddDefaultCameraPoseVertex(frame->GetID() < 2);
                }
                keyFrames.insert(*frame);
            }
        }
        
        // get common pixels between these poses
        std::vector<int> cameras; std::vector<cv::Mat> images;
        std::vector<std::vector<cv::KeyPoint>> projectedPoints;
        std::vector<pcl::PointXYZRGB> points3D;
        
        for (const TrackingFrame& frame : keyFrames) {
            cameras.push_back(frame.GetID());
            images.push_back(frame.GetCameraImage());
        }
        m_OpticalFlowEstimator->EstimateCorrespondingPixels(images, projectedPoints);
        
        // project first keyframe's points to 3D (all other keyframes can see this)
        m_3DReconstructor->TriangulatePoints(keyFrames.begin()->GetDisparity(), images[0], projectedPoints[0], points3D);
        
        // add this set of cameras and points to graph for local optimisation
        m_OptimisationGraph->AddCamerasLookingAtPoints(cameras, points3D, projectedPoints, false);
        m_OptimisationGraph->Optimise();
        
        std::cout << "\nWindowed BA complete" << std::endl;
        
        // get 3D points back and merge blocks in map
        // get optimised 3D points
        std::vector<pcl::PointXYZ> optimisedPoints;
        m_OptimisationGraph->GetPointsObservedByCamera(cameras[0], optimisedPoints);
        
        // add colour information to points
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cv::Vec3b color;
        cv::Mat cameraImage = images[0];
        for (size_t i = 0; i < optimisedPoints.size(); i++)
        {
            color = cameraImage.at<cv::Vec3b>(projectedPoints[0][i].pt.y, projectedPoints[0][i].pt.x);
            pcl::PointXYZRGB p(color[2], color[1], color[0]);
            
            p.x = optimisedPoints[i].x;
            p.y = optimisedPoints[i].y;
            p.z = optimisedPoints[i].z;
            
            cloud.push_back(p);
        }
        
        // merge the set of blocks
        m_MapDataBase.MergeBlocks(m_UnoptimisedBlocks, cloud);
        m_UnoptimisedBlocks.clear();
        
        std::cout << "\nLocal BA updated map database" << std::endl;
        
        // debug: save to disk
        static int num = 1;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map { new pcl::PointCloud<pcl::PointXYZRGB>() };
        pcl::io::savePCDFileBinary("BA_BLOCK_" + std::to_string(num) + ".pcd", cloud);
        num++;
        
        std::cout << "\nSaved to disk" << std::endl;
    }
}
