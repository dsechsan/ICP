//
// Created by Sandeep Chintada on 9/9/24.
//
#include "utils.h"

Eigen::Vector3d mean(const pcd_type &pcd){
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for(const auto &vec : pcd->points_){
        sum += vec;
    }
    sum /= static_cast<double>(pcd->points_.size());
    return sum;
}

void setColors(const pcd_type &pcd, Color color){
    pcd->colors_.clear();
    pcd->colors_.resize(pcd->points_.size(), getColorVector(color));
}

void transformPointCloud(const pcd_type &pcd, Eigen::Matrix4d &Pose){
    pcd->Transform(Pose);
};

Eigen::Vector3d getColorVector(Color color){
    switch(color){
        case Color::RED:
            return {1,0,0};
        case Color::BLUE:
            return {0,0,1};
        case Color::GREEN:
            return {0,1,0};
        case Color::YELLOW:
            return {1,1,0};
    }
}