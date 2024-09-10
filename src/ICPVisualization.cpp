//
// Created by Sandeep Chintada on 9/9/24.
//
#include "ICPVisualization.h"

ICPVisualization::ICPVisualization(const std::string &window_name, int width, int height) : visualizer(std::make_unique<open3d::visualization::Visualizer>()) {
    visualizer->CreateVisualizerWindow(window_name, width, height);
    visualizer->GetRenderOption().background_color_ = Eigen::Vector3d(0.8,0.8,0.8);
}

ICPVisualization::~ICPVisualization() {
    visualizer->DestroyVisualizerWindow();
}

void ICPVisualization::setPointClouds(const ICPVisualization::pcd_type &source,
                                      const ICPVisualization::pcd_type &target) {
    source_ptr = source;
    target_ptr = target;
    visualizer->AddGeometry(source);
    visualizer->AddGeometry(target);


}
void ICPVisualization::updateCurrentPointCloud(const ICPVisualization::pcd_type &current) {
    if(!current_ptr){
        current_ptr = current;
        visualizer->AddGeometry(current_ptr);
    }
    current_ptr = current;
    visualizer->UpdateGeometry(current_ptr);
}

void ICPVisualization::saveFrame(int iteration) {
    std::string filename = save_path+ file_prefix + std::to_string(iteration) + ".png";
    visualizer->CaptureScreenImage(filename);
    std::cout << "Saved Frame: " << filename << "\n";
}

void ICPVisualization::run(ICPAlgorithm &icp) {
    icp.align([&](const pcd_type& current, int iteration){
        updateCurrentPointCloud(current);
        visualizer->PollEvents();
        visualizer->UpdateRender();
        saveFrame(iteration);
    });
    visualizer->Run();
//    close();
}

void ICPVisualization::setSavePath(std::string &save_path_) {
    save_path = save_path_;
}

void ICPVisualization::setFilePrefix(std::string &file_prefix_) {
    file_prefix = file_prefix_;
}



