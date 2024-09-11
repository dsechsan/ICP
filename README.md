# ICP Point Cloud Alignment with Open3D and Eigen

## Overview

This project implements the classic Iterative Closest Point (ICP) Algorithm (point-to-point) from scratch in C++ using the Eigen library for matrix operations
and Open3D for handling point clouds and visualization.

## Getting Started

### Prerequisites

* CMake 3.24 or higher 
* Open3D [C++ pre-compiled binary](https://www.open3d.org/docs/latest/getting_started.html) - A library for 3D data processing and visualization
* Eigen - C++ library for Linear Algebra, used for efficient matrix computations

### Project Structure

* main.cpp : Sets up the pointclouds and runs the main visualization loop.
* ICPAlgorithm.cpp : Implementation of ICP algorithm.
* ICPVisualization.cpp : Handles the rendering and visualization using Open3D.
* include/ : Header files

### Limitations of ICP

The ICP algorithm, particularly the point-to-point variant, is highly sensitive to the initial guess. 
If the initial alignment between the source and target point clouds is not close enough, the algorithm may fail to converge. It performs well when:

* Rotation is limited to one axis and is less than 90°.
* Translation is mainly along one axis.

However, the algorithm struggles when:

* Rotation exceeds 90° or involves multiple axes.
*  initial guess is significantly inaccurate.

### Examples

<img src="https://github.com/dsechsan/ICP/blob/a04801b078473514e1eca94ac2062b86da6d35a5/gifs/output-30z.gif" width="49%" alt="-30° around z-axis" style="margin-right: 10px;"> 
<img src="https://github.com/dsechsan/ICP/blob/a04801b078473514e1eca94ac2062b86da6d35a5/gifs/output60.gif" width="49%" alt="60° around z-axis">
<img src="https://github.com/dsechsan/ICP/blob/a04801b078473514e1eca94ac2062b86da6d35a5/gifs/outputyt1.gif" width="49%" alt="1m along y axis">
<img src="https://github.com/dsechsan/ICP/blob/a04801b078473514e1eca94ac2062b86da6d35a5/gifs/outputytzr30.gif" width="49%" alt="1m along y and 30° around z">
<img src="https://github.com/dsechsan/ICP/blob/a04801b078473514e1eca94ac2062b86da6d35a5/gifs/outputyztzr30.gif" width="49%" alt="0.5m along y, 0.3m around z and 30° around z-axis">
<img src="https://github.com/dsechsan/ICP/blob/a04801b078473514e1eca94ac2062b86da6d35a5/gifs/output120x.gif" width="49%" alt="120° around z-axis">


### Future Work

* **Real-time ICP**
* **Alternative alignment methods** Explore other point cloud alignment techniques like Generalized ICP. 
* **Better visualization**
* **Optimization** for large point clouds


