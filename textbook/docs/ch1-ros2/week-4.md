---
sidebar_position: 4
---

# Week 4: Launch Files and Parameters

As our robotics systems grow in complexity, running each node manually becomes tedious. Launch files allow us to start and configure a collection of nodes at once. Parameters allow us to configure nodes without recompiling our code.

## Launch Files

ROS 2 launch files are Python scripts that allow you to start multiple nodes and configure them. They are essential for managing complex applications.

### Example Launch File

Here is an example of a launch file that starts two nodes:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_examples',
            executable='simple_publisher',
            name='publisher1'
        ),
        Node(
            package='ros2_examples',
            executable='simple_subscriber',
            name='subscriber1'
        ),
    ])
```

## Parameters

Parameters are configurable values that you can set for a node. They are used to change the behavior of a node without changing the code. Parameters can be set from a launch file or from the command line.

### Accessing Parameters in a Node

```python
from rclpy.node import Node

class MyParamNode(Node):
    def __init__(self):
        super().__init__('my_param_node')
        self.declare_parameter('my_parameter', 'default_value')
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'My parameter is: {my_param}')
```

This concludes our introduction to the Robotic Nervous System with ROS 2. You should now have a solid foundation to build upon as we move to the next chapters.