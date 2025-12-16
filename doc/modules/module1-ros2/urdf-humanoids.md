---
id: urdf-humanoids
title: URDF for Humanoid Robot Models
sidebar_label: URDF & Humanoids
---

# URDF for Humanoid Robot Models

In this section, we delve into the Unified Robot Description Format (URDF), a critical XML-based file format used in ROS 2 to describe the physical properties of a robot. URDF defines the robot's kinematics (how its parts are connected) and dynamics (mass, inertia), which are essential for simulation, visualization, and control. For humanoid robots, precise URDF modeling is paramount due to their complex articulated structure and balance requirements.

## What is URDF?

URDF represents a robot as a set of **links** (rigid bodies) connected by **joints** (degrees of freedom). Each link and joint has associated properties:

*   **Links:** Define the physical and visual characteristics of a robot component (e.g., mass, inertia, collision geometry, visual mesh). For humanoids, these could be torso, head, upper arm, forearm, hand, thigh, shank, foot, etc.
*   **Joints:** Define the kinematic and dynamic properties of the connection between two links. Common joint types include `revolute` (rotating), `prismatic` (sliding), and `fixed` (rigid connection). Humanoid joints are primarily revolute (e.g., shoulder, elbow, hip, knee, ankle).

## Structure of a URDF File

A typical URDF file starts with a `<robot>` tag, containing multiple `<link>` and `<joint>` definitions. It also includes elements for robot visualization and collision properties.

```xml
<!-- my_robot_description/urdf/simple_humanoid.urdf -->
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link: Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.4 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.4 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Neck Joint: connects torso and head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/> <!-- Offset from torso origin to top -->
    <axis xyz="0 0 1"/> <!-- Rotate around Z-axis -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

### Explanation of Elements:

*   **`<robot name="simple_humanoid">`**: The root element, giving a name to our robot.
*   **`<link name="torso">`**: Defines the main body of the robot.
    *   **`<visual>`**: Describes how the link looks (geometry, color). Here, a blue box.
    *   **`<collision>`**: Describes the link's collision properties, often identical to visual for simple shapes.
    *   **`<inertial>`**: Defines physical properties like mass and inertia, crucial for physics simulation.
*   **`<link name="head">`**: Defines the robot's head, using a red sphere.
*   **`<joint name="neck_joint" type="revolute">`**: Connects the `torso` (parent) and `head` (child).
    *   **`<origin>`**: Specifies the XYZ offset and Roll/Pitch/Yaw rotation of the joint relative to the parent link's origin.
    *   **`<axis>`**: Defines the axis of rotation for a `revolute` joint.
    *   **`<limit>`**: Sets the joint's movement limits (lower, upper, effort, velocity).

## XACRO: Enhancing URDF

While URDF is powerful, it can become verbose and repetitive for complex robots like humanoids with many similar joints and links. **XACRO (XML Macros)** is an XML macro language that allows you to use variables, mathematical expressions, and macros to write more concise and readable robot descriptions, which are then processed into standard URDF.

## Visualizing URDF in ROS 2

Once you have a URDF file, you can visualize your robot model in `RViz`, the ROS Visualization tool. This allows you to inspect the robot's kinematics, joint states, and sensor data.

```bash
# To view a URDF file directly in RViz (requires installing ros-DISTRO-joint-state-publisher-gui)
ros2 launch urdf_tutorial display.launch.py model:=src/my_robot_description/urdf/simple_humanoid.urdf
```
*(Note: Replace `ros-DISTRO-joint-state-publisher-gui` with your ROS 2 distribution, e.g., `ros-humble-joint-state-publisher-gui`)*

## Exercises

1.  Expand the `simple_humanoid.urdf` example by adding an upper arm and forearm link, connected by an `elbow_joint`. Define appropriate visual, collision, and inertial properties.
2.  Explain the importance of the `<inertial>` tag in a URDF file, especially for dynamic simulations of humanoid robots.
3.  What are the benefits of using XACRO over plain URDF for describing complex robots like humanoids? Provide a hypothetical example where XACRO would simplify a repetitive pattern.