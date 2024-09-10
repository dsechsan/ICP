//
// Created by Sandeep Chintada on 9/9/24.
//

#ifndef POINTCLOUDS_ICPVISUALIZATION_H
#define POINTCLOUDS_ICPVISUALIZATION_H
#include "open3d/Open3D.h"
#include <Eigen/Dense>
#include <memory>
#include "ICPAlgorithm.h"

class ICPVisualization{
    using pcd_type = std::shared_ptr<open3d::geometry::PointCloud>;
private:
    std::string save_path;
    std::string file_prefix;
    pcd_type current_ptr;
    pcd_type source_ptr;
    pcd_type target_ptr;

public:
    std::unique_ptr<open3d::visualization::Visualizer> visualizer;
    ICPVisualization(const std::string& window_name, int width, int height);
    ~ICPVisualization();
    void close();
    void setPointClouds(const pcd_type &source, const pcd_type &target);
    void updateCurrentPointCloud(const pcd_type &current);
    void saveFrame(int iteration);
//    void registerAnimationCallback(ICPAlgorithm& icp);
    void run(ICPAlgorithm& icp);


    void setSavePath(std::string& save_path_);
    void setFilePrefix(std::string& file_prefix_);

};



#endif //POINTCLOUDS_ICPVISUALIZATION_H
