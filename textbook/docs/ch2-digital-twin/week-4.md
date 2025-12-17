---
sidebar_position: 4
---

# Week 4: ROS 2 and Unity Integration

The real power of using Unity for robotics simulation comes from its ability to connect with ROS 2. This allows us to leverage the vast ecosystem of ROS 2 tools and packages while taking advantage of Unity's advanced rendering capabilities.

## Connecting ROS 2 and Unity

The primary method for connecting ROS 2 and Unity is through the **Unity Robotics Hub**. Specifically, we use the `ROS-TCP-Connector` package to establish a communication bridge between the ROS 2 network and the Unity simulation.

The `ROS-TCP-Connector` consists of two parts:

1.  A Unity package that runs inside the Unity Editor.
2.  A ROS 2 package that runs on the ROS side.

These two components communicate over a TCP socket, relaying ROS 2 messages back and forth.

## Bidirectional Communication

This setup allows for bidirectional communication:

-   **ROS to Unity:** Send commands from a ROS 2 node to control a robot in Unity (e.g., joint velocities, target poses).
-   **Unity to ROS:** Publish sensor data from the Unity simulation to a ROS 2 topic (e.g., camera images, LiDAR scans).

### Example: Controlling a cube in Unity from ROS 2

This example shows how you could publish a `Twist` message in ROS 2 to control the movement of a cube in a Unity scene.

**ROS 2 Publisher (Python):**

```python
# Simplified example
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class UnityController(Node):
    def __init__(self):
        super().__init__('unity_controller')
        self.publisher = self.create_publisher(Twist, 'unity_control', 10)
        # ... logic to publish twist messages
```

**Unity Subscriber (C#):**

```csharp
// Simplified example
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class CubeController : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>("unity_control", MoveCube);
    }

    void MoveCube(TwistMsg twist)
    {
        // Logic to apply the twist message to the cube's transform
        transform.Translate((float)twist.linear.x, (float)twist.linear.y, (float)twist.linear.z);
    }
}
```

This powerful combination allows us to build sophisticated digital twins for developing and testing the next generation of intelligent robots.