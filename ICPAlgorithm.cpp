//
// Created by Sandeep Chintada on 9/8/24.
//

#include <iostream>
#include <Eigen/Dense>
#include "open3d/Open3D.h"
#include "ICPAlgorithm.h"
#include "utils.h"

#include <memory>
#include <cmath>
#include <thread>


double ICPAlgorithm::computeRMSE(const ICPAlgorithm::pcd_type &current_ptr) {
    double norm = 0;
    for(size_t i=0; i<current_ptr->points_.size(); i++) {
        Eigen::Vector3d vec = current_ptr->points_[i] - target_ptr->points_[i];
        norm += vec.squaredNorm();
    }
    return std::sqrt(norm/static_cast<double>(source_ptr->points_.size()));
}


void ICPAlgorithm::computeCorrespondences(const ICPAlgorithm::pcd_type &current_ptr,
                                          const ICPAlgorithm::pcd_type &neighbours_ptr) {
    open3d::geometry::KDTreeFlann kdtree(*target_ptr);
    neighbours_ptr->points_.clear();
    for (const auto &point: current_ptr->points_) {
        std::vector<int> indices(1);
        std::vector<double> distances(1);
        kdtree.SearchKNN(point, 1, indices, distances);
        neighbours_ptr->points_.push_back(target_ptr->points_[indices[0]]);
    }

}

void ICPAlgorithm::registerPoints(const ICPAlgorithm::pcd_type &neighbours_ptr) {
    auto source_mean_vec = mean(source_ptr);
    auto target_mean_vec = mean(neighbours_ptr);

    Eigen::MatrixXd source_mean(3,source_ptr->points_.size());
    Eigen::MatrixXd target_mean(3,source_ptr->points_.size());

    for (long i = 0; i<source_ptr->points_.size(); i++){
        source_mean.col(i) = source_ptr->points_[i] - source_mean_vec;
        target_mean.col(i) = neighbours_ptr->points_[i] - target_mean_vec;
    }

    auto W = target_mean * source_mean.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto U = svd.matrixU();
    auto V = svd.matrixV();

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    I(2,2) = (U*V.transpose()).determinant();
    Eigen::Matrix3d R = U*I*V.transpose();
    Eigen::Vector3d T = target_mean_vec - R*source_mean_vec;

    pose.block<3,3>(0,0) = R;
    pose.block<3,1>(0,3) = T;
}

void ICPAlgorithm::updateCurrent(const ICPAlgorithm::pcd_type &current_ptr) {
    current_ptr->points_.clear();
    current_ptr->points_.reserve(source_ptr->points_.size());
    auto rot = pose.block<3,3>(0,0);
    auto trans = pose.block<3,1>(0,3);
    for(const auto &point : source_ptr->points_){
        auto transformed_point = rot*point + trans;
        current_ptr->points_.emplace_back(transformed_point);
    }
}

// Main ICP method. If callback is provided, visualizer window will open
void ICPAlgorithm::align(ICPAlgorithm::UpdateCallback callback){
    auto current_ptr = std::make_shared<open3d::geometry::PointCloud>(*source_ptr);
    setColors(current_ptr, Color::BLUE);

    auto neighbours_ptr = std::make_shared<open3d::geometry::PointCloud>(*target_ptr);
    int iteration = 0;
    double last_rmse = 0;

    while (iteration < max_iterations){
        computeCorrespondences(current_ptr, neighbours_ptr);
        registerPoints(neighbours_ptr);
        updateCurrent(current_ptr);
        double rmse = computeRMSE(current_ptr);

        if (callback){
            callback(current_ptr,iteration);
        }
        if (std::abs(rmse - last_rmse) < 1e-7){
            break;
        }
        last_rmse = rmse;
        iteration++;
    }
}

//Setters and Getters
void ICPAlgorithm::setSource(const pcd_type &source) {
    source_ptr = source;
}

void ICPAlgorithm::setTarget(const pcd_type &target) {
    target_ptr = target;
}

void ICPAlgorithm::setMaxIterations(const int &val) {
    max_iterations = val;
}

void ICPAlgorithm::setPose(Eigen::Matrix4d &init_pose) {
    pose = init_pose;
}

Eigen::Matrix4d ICPAlgorithm::getPose() {
    return pose;
}










