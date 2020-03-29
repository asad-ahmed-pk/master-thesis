//
// Tracker.hpp
// Tracks frames and estimates transforms between frames
//

#ifndef MASTER_THESIS_TRACKER_HPP
#define MASTER_THESIS_TRACKER_HPP

#include <memory>
#include <unordered_map>

#include <eigen3/Eigen/Eigen>

#include "pipeline/FrameFeatureExtractor.hpp"
#include "reconstruct/Reconstruct3D.hpp"
#include "system/TrackingFrame.hpp"
#include "system/OptimisationGraph.hpp"
#include "system/MappingSystem.hpp"
#include "system/KeyFrameDatabase.hpp"

namespace System
{
    class Tracker
    {
    public:
        Tracker(std::shared_ptr<Pipeline::FrameFeatureExtractor> featureExtractor,
                std::shared_ptr<Reconstruct::Reconstruct3D> reconstructor,
                std::shared_ptr<MappingSystem> mappingSystem,
                std::shared_ptr<KeyFrameDatabase> keyFrameDB);

        ~Tracker() = default;

        /// Track the given frame
        /// \param frame The frame to be tracked
        void TrackFrame(std::shared_ptr<TrackingFrame> frame);
        
        /// Get the pose matrix of the current tracked frame
        /// \return The 4x4 pose matrix
        Eigen::Matrix4d GetPose() const;
        
    private:
        void TrackFrame(std::shared_ptr<TrackingFrame> currentFrame, std::shared_ptr<TrackingFrame> recentKeyFrame);

    private:
        std::shared_ptr<Pipeline::FrameFeatureExtractor> m_FeatureExtractor;
        std::shared_ptr<Reconstruct::Reconstruct3D> m_3DReconstructor;
        std::shared_ptr<MappingSystem> m_MappingSystem;
        std::shared_ptr<KeyFrameDatabase> m_KeyFrameDatabase;
        
    private:
        std::unordered_map<size_t, int> m_KeyFramePoseVertexIDs;
        std::unordered_map<size_t, std::tuple<int, int>> m_KeyFrame3DPointVertexIDs;
        std::vector<std::shared_ptr<TrackingFrame>> m_KeyFrames;
        
    private:
        Eigen::Matrix4d m_CurrentPose = Eigen::Matrix4d::Identity();
        std::unique_ptr<OptimisationGraph> m_OptimisationGraph;
    };
}

#endif //MASTER_THESIS_TRACKER_HPP
