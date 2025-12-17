---
sidebar_position: 1
---

# Week 1: Introduction to VLAs

Welcome to the final chapter of our textbook! We've built a nervous system (ROS 2), created a virtual world (Gazebo/Unity), and given our robot a powerful brain (NVIDIA Isaac). Now, it's time to connect the robot's perception to its actions using **Vision-Language-Action (VLA)** models.

## What are VLAs?

VLAs, also known as Vision-Language-Action models, are a type of multimodal AI model that can understand and respond to instructions given in natural language, grounded in the visual context of the environment. In simpler terms, you can tell a VLA-powered robot what to do in plain English, and it will figure out the necessary actions based on what it "sees."

## The Three Modalities

As the name suggests, VLAs operate on three modalities:

1.  **Vision:** The robot's perception of the world, typically through a camera.
2.  **Language:** The natural language command given by the user (e.g., "pick up the red block").
3.  **Action:** The sequence of motor commands the robot needs to execute to accomplish the task (e.g., joint angles, end-effector poses).

## Why are VLAs a Game-Changer for Robotics?

For decades, programming robots has been a tedious and expert-driven task. Every new task required a complex, hand-crafted program. VLAs promise a future where robots can be instructed with the same ease as a human.

This has profound implications for:

-   **Manufacturing:** Quickly re-tasking robots on an assembly line.
-   **Logistics:** Instructing robots to sort and pack a wide variety of items.
-   **Healthcare:** Assisting patients with everyday tasks.
-   **Home Robotics:** A truly helpful robot assistant in our homes.

![A conceptual diagram of a VLA](https://www.tri.global/sites/default/files/styles/card_image/public/2023-03/Large-Scale-Behavior-Cloning.jpg?itok=8G2t5p2q)

In this chapter, we will explore the architectures of VLAs, how they are trained, and how they can be deployed on a real robot.