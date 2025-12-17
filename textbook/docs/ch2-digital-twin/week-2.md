---
sidebar_position: 2
---

# Week 2: Gazebo Simulation

Gazebo is a mature, open-source robotics simulator. It is the go-to choice for many roboticists, especially those working with ROS. Gazebo provides a rich set of features for simulating robots, sensors, and environments.

## Key Features of Gazebo

- **Physics Engines:** Supports multiple physics engines, including ODE, Bullet, Simbody, and DART.
- **Sensor Simulation:** A wide range of sensors are supported, including cameras, LiDAR, IMUs, and GPS.
- **ROS Integration:** Seamless integration with ROS and ROS 2.
- **Graphical Interface:** A user-friendly GUI for visualizing and interacting with the simulation.
- **Plugins:** Extend Gazebo's functionality with custom plugins.

## Building a World in Gazebo

Gazebo worlds are described using SDF (Simulation Description Format), which is an XML-based format. You can create complex environments with buildings, furniture, and other objects.

### Example SDF for a simple box

```xml
<?xml version='1.0'?>
<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    <model name='box'>
        <pose>0 0 0.5 0 0 0</pose>
        <link name='link'>
            <collision name='collision'>
                <geometry>
                    <box>
                        <size>1 1 1</size>
                    </box>
                </geometry>
            </collision>
            <visual name='visual'>
                <geometry>
                    <box>
                        <size>1 1 1</size>
                    </box>
                </geometry>
            </visual>
        </link>
    </model>
  </world>
</sdf>
```

![Gazebo World](https://classic.gazebosim.org/assets/images/screenshots/gazebo_7_4_screenshot_1-29437299.png)