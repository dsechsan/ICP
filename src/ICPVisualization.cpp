//
// Created by Sandeep Chintada on 9/9/24.
//
#include "ICPVisualization.h"

/**
 * @brief Constructs the ICPVisualization object and initializes the visualizer window with grey background.
 * @param window_name The name of the visualizer window.
 * @param width The width of the visualizer window in pixels.
 * @param height The height of the visualizer window in pixels.
 */

ICPVisualization::ICPVisualization(const std::string &window_name, int width, int height) :
    visualizer(std::make_unique<open3d::visualization::Visualizer>()) {
    visualizer->CreateVisualizerWindow(window_name, width, height);
    visualizer->GetRenderOption().background_color_ = Eigen::Vector3d(0.9,0.96,1);
}

/**
 * @brief Destroys the visualizer window upon object destruction.
 */
ICPVisualization::~ICPVisualization() {
    visualizer->DestroyVisualizerWindow();
}

/**
 * @brief Sets the source and target point clouds for visualization and adds them to the visualizer.
 * @param source The source point cloud shared pointer.
 * @param target The target point cloud shared pointer.
 */

void ICPVisualization::setPointClouds(const ICPVisualization::pcd_type &source,
                                      const ICPVisualization::pcd_type &target) {
    source_ptr = source;
    target_ptr = target;
    visualizer->AddGeometry(source);
    visualizer->AddGeometry(target);
}

/**
 * @brief Updates the current point cloud in the visualizer with the latest alignment results.
 * @param current The current point cloud to be updated in the visualizer.
 */

void ICPVisualization::updateCurrentPointCloud(const ICPVisualization::pcd_type &current) {
    if(!current_ptr){
        current_ptr = current;
        visualizer->AddGeometry(current_ptr);
    }
    current_ptr = current;
    visualizer->UpdateGeometry(current_ptr);
}

/**
 * @brief Saves the current frame of the visualizer as an image file.
 * @param iteration The current iteration number, used to name the saved image file.
 */

void ICPVisualization::saveFrame(int iteration) {
    std::string filename = save_path+ file_prefix + std::to_string(iteration) + ".png";
    visualizer->CaptureScreenImage(filename);
    std::cout << "Saved Frame: " << filename << "\n";
}

/**
 * @brief Runs the ICP algorithm and updates the visualizer during each iteration of the alignment.
 * @param icp The ICPAlgorithm object that handles the point cloud alignment.
 */
void ICPVisualization::run(ICPAlgorithm &icp) {
    icp.align([&](const pcd_type& current, int iteration){
        updateCurrentPointCloud(current);
        visualizer->PollEvents();
        visualizer->UpdateRender();
        saveFrame(iteration);
    });
    visualizer->Run();
}

/**
 * @brief Sets the path where frames will be saved.
 * @param save_path_ The path where the frames should be saved.
 */
void ICPVisualization::setSavePath(std::string &save_path_) {
    save_path = save_path_;
}
/**
 * @brief Sets the prefix for saved frame filenames.
 * @param file_prefix_ The prefix to use for saved frame filenames.
 */
void ICPVisualization::setFilePrefix(std::string &file_prefix_) {
    file_prefix = file_prefix_;
}



