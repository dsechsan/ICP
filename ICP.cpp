//
// Created by Sandeep Chintada on 9/8/24.
//

#include <iostream>
#include <Eigen/Dense>
#include "open3d/Open3D.h"
#include "ICP.h"

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

Eigen::Vector3d ICPAlgorithm::mean(const ICPAlgorithm::pcd_type &pcd) {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for(const auto &vec : pcd->points_){
        sum += vec;
    }
    sum /= static_cast<double>(pcd->points_.size());
    return sum;
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

void ICPAlgorithm::registerPoints(const ICPAlgorithm::pcd_type &neighbours_ptr,
                                  Eigen::Matrix4d &Pose) {
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

    Pose.block<3,3>(0,0) = R;
    Pose.block<3,1>(0,3) = T;
}

void ICPAlgorithm::updateCurrent(const ICPAlgorithm::pcd_type &current_ptr,
                                 const Eigen::Matrix4d &Pose) {
    current_ptr->points_.clear();
    current_ptr->points_.reserve(source_ptr->points_.size());
    auto rot = Pose.block<3,3>(0,0);
    auto trans = Pose.block<3,1>(0,3);
    for(const auto &point : source_ptr->points_){
        auto transformed_point = rot*point + trans;
        current_ptr->points_.emplace_back(transformed_point);
    }
}

bool ICPAlgorithm::updateICP(open3d::visualization::Visualizer* visualizer, const ICPAlgorithm::pcd_type &current_ptr,
                             const ICPAlgorithm::pcd_type &neighbours_ptr,
                             Eigen::Matrix4d &Pose, double &last_rmse, int &iteration) {

    if (iteration >= max_iterations) {
        std::cout << "Reached maximum iterations: " << max_iterations << std::endl;
        visualizer->Close();
        return false;
    }

    computeCorrespondences(current_ptr, neighbours_ptr);
    registerPoints(neighbours_ptr, Pose);
    updateCurrent(current_ptr,Pose);

    visualizer->UpdateGeometry(current_ptr);
    std::string filename = save_path_+ file_prefix_ + std::to_string(iteration) + ".png";
    visualizer->CaptureScreenImage(filename);
    std::cout << "Saved " << filename << std::endl;

    double rmse = computeRMSE(current_ptr);
    std::cout << "Iteration " << iteration++ << " RMSE: " << rmse << std::endl;

    if (std::abs(rmse - last_rmse) < 1e-8) {
        std::cout << "Converged with RMSE difference: " << std::abs(rmse - last_rmse) << std::endl;
        visualizer->Close();
        return false;
    }
    last_rmse = rmse;

    return true;
}

void ICPAlgorithm::setColors(const ICPAlgorithm::pcd_type &pcd, Eigen::Vector3d &color) {
    pcd->colors_.clear();
    pcd->colors_.resize(pcd->points_.size(),color);
}

void ICPAlgorithm::transformPointCloud(const ICPAlgorithm::pcd_type &pcd, Eigen::Matrix4d &Pose) {
    pcd->Transform(Pose);
}






