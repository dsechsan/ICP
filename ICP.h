//
// Created by Sandeep Chintada on 9/4/24.
//

#ifndef POINTCLOUDS_ICP_H
#define POINTCLOUDS_ICP_H

#include <Eigen/Dense>
#include "open3d/Open3D.h"
#include <memory>
#include <utility>

class ICPAlgorithm {
public:
    using pcd_type = std::shared_ptr<open3d::geometry::PointCloud>;

    ICPAlgorithm() : source_ptr(nullptr), target_ptr(nullptr){};
    ICPAlgorithm(pcd_type source, pcd_type target): source_ptr(std::move(source)), target_ptr(std::move(target)){};

    ICPAlgorithm(const ICPAlgorithm& aTrans) = delete;
    ICPAlgorithm& operator=(const ICPAlgorithm& aTrans) = delete;
    ~ICPAlgorithm() = default;

    void setSource(const pcd_type &source){
        source_ptr = source;
    }

    void setTarget(const pcd_type &target){
        target_ptr = target;
    }

    void setMaxIterations(const int &val){
        max_iterations = val;
    }

    void setSavePath(const std::string& savepath){
        save_path_ = savepath;
    }
    void setFilePrefix(const std::string& fileprefix){
        file_prefix_ = fileprefix;
    }

    static Eigen::Vector3d mean(const pcd_type &pcd);
    static void setColors(const pcd_type &pcd, Eigen::Vector3d &color);
    static void transformPointCloud(const pcd_type &pcd, Eigen::Matrix4d &Pose);
    bool updateICP(open3d::visualization::Visualizer* visualizer,
                   const pcd_type &current, const pcd_type &neighbours,
                   Eigen::Matrix4d &Pose, double &last_rmse, int& iteration);


private:
    pcd_type source_ptr;
    pcd_type target_ptr;
    int max_iterations = 50;
    std::string save_path_;
    std::string file_prefix_;

    double computeRMSE(const pcd_type &current);
    void computeCorrespondences(const pcd_type &current, const pcd_type &neighbours);
    void registerPoints(const pcd_type &neighbours, Eigen::Matrix4d &Pose);
    void updateCurrent(const pcd_type &current, const Eigen::Matrix4d &Pose);

};

#endif //POINTCLOUDS_ICP_H
