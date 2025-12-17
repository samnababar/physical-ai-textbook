---
sidebar_position: 3
---

# Week 3: Unity Simulation

Unity is a powerful and versatile game engine that has gained traction in the robotics community for its high-fidelity graphics, ease of use, and extensive asset store. It is particularly well-suited for training and testing computer vision and reinforcement learning models.

## Why Unity for Robotics?

- **Photorealistic Rendering:** Create highly realistic environments for training vision-based AI.
- **C# Scripting:** A modern, object-oriented programming language for creating complex simulation logic.
- **Asset Store:** A vast marketplace of 3D models, environments, and tools.
- **Unity Robotics Hub:** A set of official packages for connecting Unity simulations with ROS.

## Unity Robotics Hub

The Unity Robotics Hub provides several key packages:

- **ROS-TCP-Connector:** Enables communication between Unity and ROS using a TCP-based protocol.
- **URDF-Importer:** A tool for importing URDF (Unified Robot Description Format) files into Unity.
- **Unity Catalog:** A collection of pre-made robotic assets and environments.

### Example C# script for rotating a cube

```csharp
using UnityEngine;

public class Rotator : MonoBehaviour
{
    // Update is called once per frame
    void Update()
    {
        transform.Rotate(new Vector3(15, 30, 45) * Time.deltaTime);
    }
}
```

![Unity Editor](https://unity.com/sites/default/files/styles/16_9_s_scale/public/2020-09/unity-editor-2020-2.jpeg?itok=jRTGj4m7)