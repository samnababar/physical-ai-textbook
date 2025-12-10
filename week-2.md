---
sidebar_position: 2
---

# Week 2: ROS 2 Nodes and Topics

This week, we'll explore two of the most fundamental concepts in ROS 2: Nodes and Topics. They are the building blocks of any ROS 2 application.

## Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. In essence, a node in ROS is a process that performs computation. A well-designed ROS system is comprised of many nodes, each responsible for a single, well-defined task (e.g., one node for controlling wheel motors, one node for controlling a laser range-finder, etc.).

## Topics

Topics are named buses over which nodes exchange messages. Topics have anonymous publish/subscribe semantics, which decouples the production of information from its consumption. In general, nodes do not know who they are communicating with. Instead, nodes that are interested in data *subscribe* to the relevant topic; and nodes that generate data *publish* to the relevant topic.

### Publisher/Subscriber Example

Here is a Python example of a simple publisher and subscriber.

#### Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

# main function and entry point...
```

#### Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

# main function and entry point...
```

![ROS 2 Computation Graph](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-CLI-Tools/Understanding-ROS-2-Topics/images/topics_pub_sub.png)