---
sidebar_position: 3
---

# Week 3: ROS 2 Services and Actions

While topics are great for continuous data streams, sometimes we need a request/reply interaction. This is where Services and Actions come in.

## Services

Services are a type of request/response communication in ROS 2. One node acts as a *service server*, providing a service, and another node acts as a *service client*, calling the service. This is a synchronous communication, meaning the client will wait until the server has responded.

### Service/Client Example

Here's an example of a service that adds two integers.

#### Service Server

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class SimpleService(Node):
    def __init__(self):
        super().__init__('simple_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response

# main function and entry point...
```

#### Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class SimpleClient(Node):
    def __init__(self):
        super().__init__('simple_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

# ... sending request logic
```

## Actions

Actions are similar to services, but they are designed for long-running tasks. They provide feedback during execution and are cancellable. An action consists of a goal, feedback, and a result.

![ROS 2 Actions](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-CLI-Tools/Understanding-ROS-2-Actions/images/actions_conceptual.png)

```