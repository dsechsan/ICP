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

/**
 * @brief Computes the RMSE between the current and target point clouds.
 * @param current_ptr The current point cloud shared pointer.
 * @return double The computed RMSE value.
 */

double ICPAlgorithm::computeRMSE(const ICPAlgorithm::pcd_type &current_ptr) {
    double norm = 0;
    for(size_t i=0; i<current_ptr->points_.size(); i++) {
        Eigen::Vector3d vec = current_ptr->points_[i] - target_ptr->points_[i];
        norm += vec.squaredNorm();
    }
    return std::sqrt(norm/static_cast<double>(source_ptr->points_.size()));
}

/**
* @brief Computes the correspondences between the current and target point clouds using a KD-Tree.
* @param current_ptr The current point cloud shared pointer.
* @param neighbours_ptr The neighbors point cloud shared pointer to store the correspondences.
*/

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
/**
 * @brief Registers the source and the target by using the computed correspondences as
 * ground truth. Uses the Kabsch Algorithm.
 * @param neighbours_ptr The neighbors point cloud shared pointer to store the correspondences.
 */
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

/**
 * @brief Updates the current point cloud by transforming the source point cloud using the estimated pose.
 * @param current_ptr The current point cloud shared pointer to be updated with the transformed points.
 */

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

/**
 * @brief Main ICP alignment method that iteratively updates the current point cloud to align with the target.
 *
 * This function performs the ICP algorithm, computing correspondences, registering points, and updating
 * the current point cloud. If a callback is provided, it will be called at each iteration to update the visualization.
 *
 * @param callback An optional callback function to update the visualizer with the current point cloud at each iteration.
 */

void ICPAlgorithm::align(ICPAlgorithm::UpdateCallback callback){
    auto current_ptr = std::make_shared<open3d::geometry::PointCloud>(*source_ptr);
    setColors(current_ptr, Color::YELLOW);

    auto neighbours_ptr = std::make_shared<open3d::geometry::PointCloud>(*target_ptr);
    int iteration = 1;
    double last_rmse = 0;

    while (iteration < max_iterations){
        if (callback){
            callback(current_ptr,iteration);
        }
        computeCorrespondences(current_ptr, neighbours_ptr);
        registerPoints(neighbours_ptr);
        updateCurrent(current_ptr);
        double rmse = computeRMSE(current_ptr);
        if (std::abs(rmse - last_rmse) < 1e-7){
            break;
        }
        last_rmse = rmse;
        iteration++;
    }
}

//Setters and Getters for ICP Algorithm

/**
 * @brief Sets the source point cloud for the ICP algorithm.
 * @param source The source point cloud shared pointer.
 */

void ICPAlgorithm::setSource(const pcd_type &source) {
    source_ptr = source;
}
/**
 * @brief Sets the target point cloud for the ICP algorithm.
 * @param target The target point cloud shared pointer.
 */

void ICPAlgorithm::setTarget(const pcd_type &target) {
    target_ptr = target;
}

/**
 * @brief Sets the maximum number of iterations for the ICP alignment.
 * @param val The maximum number of iterations.
 */
void ICPAlgorithm::setMaxIterations(const int &val) {
    max_iterations = val;
}

/**
 * @brief Sets the initial pose matrix for the ICP alignment.
 * @param init_pose The initial pose matrix.
 */
void ICPAlgorithm::setPose(Eigen::Matrix4d &init_pose) {
    pose = init_pose;
}

/**
 * @brief Retrieves the current pose matrix estimated by the ICP alignment.
 * @return Eigen::Matrix4d The current pose matrix.
 */

Eigen::Matrix4d ICPAlgorithm::getPose() {
    return pose;
}










