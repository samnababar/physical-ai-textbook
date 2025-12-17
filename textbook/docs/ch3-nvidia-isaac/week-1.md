---
sidebar_position: 1
---

# Week 1: Introduction to NVIDIA Isaac

Welcome to Chapter 3! Now that we have a grasp of ROS 2 and simulation, it's time to build the "brain" of our robot. For this, we turn to the **NVIDIA Isaac** platform, a powerful toolkit for developing AI-powered robots.

## What is NVIDIA Isaac?

NVIDIA Isaac is an end-to-end platform for the development, simulation, and deployment of AI-enabled robots. It provides a comprehensive set of tools and SDKs that leverage NVIDIA's expertise in GPU-accelerated computing, computer vision, and AI.

The Isaac platform is composed of several key components, which we will explore in this chapter:

-   **Isaac Sim:** A photorealistic, physically accurate virtual environment for developing, testing, and training AI-based robots.
-   **Isaac Gym:** A high-performance reinforcement learning framework for training policies in parallel on a single GPU.
-   **Isaac ROS:** A collection of hardware-accelerated packages and GEMs (GPU-accelerated ROS packages) that give ROS developers a significant performance boost.

## Why NVIDIA Isaac?

-   **Performance:** By leveraging NVIDIA GPUs, the Isaac platform provides unparalleled performance for simulation and AI inference.
-   **Realism:** Isaac Sim offers photorealistic rendering and accurate physics, which is crucial for training vision-based AI models (a concept known as *Sim2Real*).
-   **Ecosystem:** The platform is tightly integrated with other NVIDIA technologies like DeepStream for video analytics and TAO for transfer learning.
-   **ROS-Compatible:** Isaac ROS allows developers to seamlessly integrate GPU acceleration into their existing ROS 2 workflows.

![NVIDIA Isaac Platform](https://developer.nvidia.com/sites/default/files/akamai/isaac/isaac-ros-banner.jpg)