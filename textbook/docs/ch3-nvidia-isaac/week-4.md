---
sidebar_position: 4
---

# Week 4: Isaac ROS

**NVIDIA Isaac ROS** is a collection of hardware-accelerated packages that enable ROS developers to build high-performance robotics applications. It bridges the gap between the powerful NVIDIA Isaac SDKs and the familiar ROS 2 ecosystem.

## The Best of Both Worlds

Isaac ROS allows you to continue using the ROS 2 tools and conventions you are familiar with, while transparently adding GPU acceleration for key robotics tasks like perception, navigation, and manipulation.

These are not just simple wrappers around NVIDIA libraries. Isaac ROS packages are *ROS 2 nodes* that have been optimized to take full advantage of the underlying NVIDIA hardware, including GPUs and Jetson devices.

## Key Components of Isaac ROS

Isaac ROS is comprised of several categories of packages:

-   **isaac_ros_common:** Core utilities and data types.
-   **isaac_ros_image_pipeline:** GPU-accelerated versions of common image processing tasks like rectification, resizing, and color space conversion.
-   **isaac_ros_apriltag:** High-performance AprilTag detection.
-   **isaac_ros_visual_slam:** A GPU-accelerated Visual SLAM solution for real-time localization and mapping.
-   **isaac_ros_nvblox:** Real-time 3D reconstruction for navigation and obstacle avoidance.
-   **isaac_ros_dnn_inference:** A framework for running deep neural network (DNN) models for tasks like object detection and segmentation.

## Example: Using Isaac ROS for Visual SLAM

A typical ROS 2 Visual SLAM pipeline involves several processing steps on the input camera images. With Isaac ROS, many of these steps can be offloaded to the GPU, resulting in a significant performance improvement.

**Traditional ROS 2 Pipeline (CPU-based):**

```
Camera Node -> Rectification Node (CPU) -> Visual SLAM Node (CPU)
```

**Isaac ROS Pipeline (GPU-accelerated):**

```
Camera Node -> Rectification Node (GPU) -> Visual SLAM Node (GPU)
```

By keeping the data on the GPU as it flows from one processing stage to the next (a concept known as "zero-copy"), Isaac ROS avoids costly memory transfers between the CPU and GPU, leading to much higher throughput and lower latency.

![Isaac ROS GEMs](https://developer.nvidia.com/blog/wp-content/uploads/2022/03/isaac-ros-gems-1.png)

This concludes our chapter on the AI-Robot Brain. You now have an understanding of how to leverage the NVIDIA Isaac platform to build and train intelligent robots, and how to integrate that power into the ROS 2 ecosystem.