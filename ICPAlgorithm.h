//
// Created by Sandeep Chintada on 9/4/24.
//

#ifndef POINTCLOUDS_ICPALGORITHM_H
#define POINTCLOUDS_ICPALGORITHM_H

#include <Eigen/Dense>
#include "open3d/Open3D.h"
#include <memory>
#include <utility>

class ICPAlgorithm {
public:
    using pcd_type = std::shared_ptr<open3d::geometry::PointCloud>;

    ICPAlgorithm() : source_ptr(nullptr), target_ptr(nullptr), max_iterations(50){};
    ICPAlgorithm(pcd_type source, pcd_type target): source_ptr(std::move(source)), target_ptr(std::move(target)){};

    ICPAlgorithm(const ICPAlgorithm& aTrans) = delete;
    ICPAlgorithm& operator=(const ICPAlgorithm& aTrans) = delete;
    ~ICPAlgorithm() = default;

    void setSource(const pcd_type &source);
    void setTarget(const pcd_type &target);
    void setMaxIterations(const int &val);
    void setPose(Eigen::Matrix4d& init_pose);
    Eigen::Matrix4d getPose();

    using UpdateCallback = std::function<void(pcd_type&, int&)>;
    void align(UpdateCallback updateCallback = nullptr);

//    bool updateICP(open3d::visualization::Visualizer* visualizer,
//                   const pcd_type &current, const pcd_type &neighbours,
//                   Eigen::Matrix4d &Pose, double &last_rmse, int& iteration);


private:
    pcd_type source_ptr;
    pcd_type target_ptr;
    int max_iterations;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

    double computeRMSE(const pcd_type &current);
    void computeCorrespondences(const pcd_type &current, const pcd_type &neighbours);
    void registerPoints(const pcd_type &neighbours);
    void updateCurrent(const pcd_type &current);

};

#endif //POINTCLOUDS_ICPALGORITHM_H
