//
// Created by Sandeep Chintada on 9/9/24.
//

#ifndef POINTCLOUDS_UTILS_H
#define POINTCLOUDS_UTILS_H
#include <Eigen/Dense>
#include "open3d/Open3D.h"
#include <iostream>
#include <memory>

using pcd_type = std::shared_ptr<open3d::geometry::PointCloud>;
enum class Color{
    RED,
    GREEN,
    BLUE,
    YELLOW,
    DARK_RED,
    DARK_GREEN,
    DARK_BLUE,
    NAVY

};
Eigen::Vector3d mean(const pcd_type &pcd);
void setColors(const pcd_type &pcd, Color color);
void transformPointCloud(const pcd_type &pcd, Eigen::Matrix4d &Pose);

Eigen::Vector3d getColorVector(Color color);
#endif //POINTCLOUDS_UTILS_H
