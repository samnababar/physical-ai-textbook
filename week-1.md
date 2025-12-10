---
sidebar_position: 1
---

# Week 1: Introduction to ROS 2

Welcome to the first week of our journey into the Robotic Nervous System! This week, we'll be diving into the fundamentals of ROS 2, the open-source middleware that has become the standard for robotics development.

## What is ROS?

The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

While it's called an "Operating System," it's more accurate to think of it as "middleware" or a "meta-operating system" that runs on top of a host OS like Linux, macOS, or Windows.

## Why ROS 2?

ROS 2 is a complete redesign of the original ROS framework, built to address the needs of modern robotics applications, including:

- **Real-time control:** Support for deterministic, low-latency communication.
- **Multi-robot systems:** Native support for communication between multiple robots.
- **Production environments:** Enhanced security, robustness, and lifecycle management.
- **Cross-platform support:** Officially supports Linux, macOS, and Windows.

## Core Concepts

We will cover the following core concepts this week:

- **Nodes:** The fundamental processing units in ROS 2.
- **Topics:** The communication channels that nodes use to exchange data.
- **Messages:** The data structures that are sent over topics.
- **Workspace:** The directory where you develop your ROS 2 applications.

## Example: Your First ROS 2 Node

Here is a simple "Hello, World!" example of a ROS 2 node written in Python.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('hello_world_node')
        self.get_logger().info('Hello, World!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

![ROS 2 Logo](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/_static/ros2-logo-background-white.png)