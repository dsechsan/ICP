//
// Created by Sandeep Chintada on 9/9/24.
//
#include "ICPAlgorithm.h"
#include "ICPVisualization.h"
#include <memory>
#include "open3d/Open3D.h"
#include "utils.h"

int main(){
//    bool debug = false;
    std::string sourcepath = "/Users/dsechs/Library/CloudStorage/OneDrive-UCSanDiego/Desktop/Cpp/pointclouds/bunny.pcd";
    open3d::geometry::PointCloud source, target;
    open3d::io::ReadPointCloud(sourcepath, source);
    std::string targetpath = "/Users/dsechs/Downloads/kitchen/Rf4.pcd";
//    open3d::io::ReadPointCloud(targetpath, target);
    Eigen::Matrix4d test_pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d axis = {1,0,0};
    double angle = 0;

    auto rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    auto trans = Eigen::Vector3d(0, 1, 0);

    Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
    target_pose.block<3, 3>(0, 0) = rot;
    target_pose.block<3, 1>(0, 3) = trans;

    target = source;

    auto source_ptr = std::make_shared<open3d::geometry::PointCloud>(source);
    setColors(source_ptr,Color::YELLOW);
    auto target_ptr = std::make_shared<open3d::geometry::PointCloud>(target);
    setColors(target_ptr,Color::RED);
    transformPointCloud(target_ptr,target_pose);


    Eigen::Matrix4d pose_init = Eigen::Matrix4d::Identity();
    ICPAlgorithm icp(source_ptr,target_ptr);
    icp.setPose(pose_init);
    icp.setMaxIterations(50);
//
//    icp.align();
//    std::cout << icp.getPose() << "\n";
    std::string file_prefix = "frame_";
    std::string save_path = "/Users/dsechs/Library/CloudStorage/OneDrive-UCSanDiego/Desktop/Cpp/pointclouds/frames/";
    ICPVisualization vis("ICP Visualization", 1200,800);
    vis.setSavePath(save_path);
    vis.setFilePrefix(file_prefix);
    vis.setPointClouds(source_ptr,target_ptr);

    vis.visualizer->UpdateRender();
    vis.run(icp);

    std::cout << icp.getPose() << "\n";

    return 0;


}