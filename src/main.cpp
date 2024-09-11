//
// Created by Sandeep Chintada on 9/9/24.
//
#include "ICPAlgorithm.h"
#include "ICPVisualization.h"
#include <memory>
#include "open3d/Open3D.h"
#include "utils.h"

/**
 * @brief The main function for running the ICP (Iterative Closest Point) visualization and alignment.
 *
 * This function initializes the source and target point clouds, sets up the ICP algorithm,
 * and runs the visualization with the specified settings. The target can either be a transformed
 * version of the source for testing (debug mode) or a separate point cloud file.
 * If only pose is needed, icp.align() should get you the answer.
 *
 * @return int Returns 0 upon successful execution.
 */
int main(){
    bool debug = true;

    std::string sourcepath = "/Users/dsechs/Library/CloudStorage/OneDrive-UCSanDiego/Desktop/Cpp/pointclouds/data/bunny.pcd";
    open3d::geometry::PointCloud source, target;
    open3d::io::ReadPointCloud(sourcepath, source);
    auto source_ptr = std::make_shared<open3d::geometry::PointCloud>(source);
    pcd_type target_ptr;
    setColors(source_ptr,Color::NAVY);

    if(!debug) {
        std::string targetpath = "/Users/dsechs/Downloads/kitchen/Rf4.pcd";
        open3d::io::ReadPointCloud(targetpath, target);
        target_ptr = std::make_shared<open3d::geometry::PointCloud>(target);
    }else{
    // Transform the source_ptr by a known R,T for testing
        Eigen::Matrix4d test_pose = Eigen::Matrix4d::Identity();
        Eigen::Vector3d axis = {0, 0, 1};
        double angle = M_PI/6.0;

        auto rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        auto trans = Eigen::Vector3d(0, 0.5, 0.3);

        Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
        target_pose.block<3, 3>(0, 0) = rot;
        target_pose.block<3, 1>(0, 3) = trans;

        target = source;
        target_ptr = std::make_shared<open3d::geometry::PointCloud>(target);
        setColors(target_ptr,Color::RED);
        transformPointCloud(target_ptr,target_pose);
    }


    Eigen::Matrix4d pose_init = Eigen::Matrix4d::Identity();
    ICPAlgorithm icp(source_ptr,target_ptr);
    icp.setPose(pose_init);
    icp.setMaxIterations(100);

    std::string file_prefix = "frame_";
    std::string save_path = "/Users/dsechs/Library/CloudStorage/OneDrive-UCSanDiego/Desktop/Cpp/pointclouds/frames/";
    ICPVisualization vis("ICP Visualization", 1600,1200);
    vis.setSavePath(save_path);
    vis.setFilePrefix(file_prefix);
    vis.setPointClouds(source_ptr,target_ptr);

    vis.visualizer->UpdateRender();
    vis.run(icp);
    std::cout << icp.getPose() << "\n";

    return 0;


}