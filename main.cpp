//
// Created by Sandeep Chintada on 9/9/24.
//
#include "ICP.h"
#include <memory>
#include "open3d/Open3D.h"

int main(int argc, char* argv[]){
    bool debug = false;
    std::string sourcepath = "/Users/dsechs/Downloads/kitchen/Rf3.pcd";
    open3d::geometry::PointCloud source, target, current, neighbours;
    open3d::io::ReadPointCloud(sourcepath, source);
    std::string targetpath = "/Users/dsechs/Downloads/kitchen/Rf4.pcd";
    open3d::io::ReadPointCloud(targetpath, target);

    if (debug){target = source;}

    current = source;
    neighbours = target;

    auto source_ptr = std::make_shared<open3d::geometry::PointCloud>(source);
    auto target_ptr = std::make_shared<open3d::geometry::PointCloud>(target);
    auto current_ptr = std::make_shared<open3d::geometry::PointCloud>(current);
    auto neighbours_ptr = std::make_shared<open3d::geometry::PointCloud>(neighbours);


    auto yellow = Eigen::Vector3d(1, 1, 0);
    ICPAlgorithm::setColors(source_ptr, yellow);

    auto blue = Eigen::Vector3d(0, 0, 1);
    ICPAlgorithm::setColors(current_ptr, blue);

    auto red = Eigen::Vector3d(1, 0, 0);
    ICPAlgorithm::setColors(target_ptr, red);

    if (debug) {
        auto angle = 0.5;
        auto axis = Eigen::Vector3d(0, 0, 1);
        auto rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        auto trans = Eigen::Vector3d(0, 0, 0);

        Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
        target_pose.block<3, 3>(0, 0) = rot;
        target_pose.block<3, 1>(0, 3) = trans;

        ICPAlgorithm::transformPointCloud(target_ptr, target_pose);
    }

    Eigen::Matrix4d pose_initial_guess = Eigen::Matrix4d::Identity();
    double last_rmse = 0;
    int iteration = 1;

    ICPAlgorithm transformationEstimation(source_ptr, target_ptr);
    std::string file_prefix = "frame_";
    std::string save_path = "/Users/dsechs/Library/CloudStorage/OneDrive-UCSanDiego/Desktop/Cpp/pointclouds/frames/";
    transformationEstimation.setFilePrefix(file_prefix);
    transformationEstimation.setSavePath(save_path);

    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("ICP Animation", 1600, 1200);
    visualizer.AddGeometry(current_ptr);
    visualizer.AddGeometry(source_ptr);
    visualizer.AddGeometry(target_ptr);
    visualizer.GetRenderOption().background_color_ = Eigen::Vector3d(0.8,0.8,0.8);

    std::string initial_filename = save_path + "frame_0.png";
    visualizer.PollEvents();
    visualizer.UpdateRender();
    visualizer.CaptureScreenImage(initial_filename);
    std::cout << "Saved initial frame: " << initial_filename << std::endl;

    visualizer.RegisterAnimationCallback([&](open3d::visualization::Visualizer* vis) {
        return transformationEstimation.updateICP(vis,current_ptr, neighbours_ptr,
                                                  pose_initial_guess,last_rmse,iteration);
    });

    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
    std::cout << pose_initial_guess << "\n";

    return 0;


}