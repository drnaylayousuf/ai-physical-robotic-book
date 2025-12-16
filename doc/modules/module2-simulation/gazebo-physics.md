# Gazebo and Physics Simulation: Bringing Robots to Life Virtually

Gazebo is a powerful open-source 3D robotics simulator that allows you to accurately test your robot designs and algorithms in a virtual environment. It's an essential tool for physical AI development, providing realistic physics simulation, high-quality rendering, and a robust interface for sensors and actuators. For humanoid robots, which involve complex dynamics and balance, Gazebo's accurate physics engine is indispensable.

## Key Features of Gazebo

*   **Physics Engine:** Gazebo integrates with various high-performance physics engines (e.g., ODE, Bullet, DART, Simbody) to accurately simulate rigid body dynamics, gravity, friction, and collisions.
*   **3D Graphics:** Provides realistic rendering of robots and environments, aiding in visualization and debugging.
*   **Sensor Simulation:** Offers plugins to simulate common robot sensors like cameras (RGB, depth, monochrome), LiDAR, IMUs, contact sensors, and more.
*   **Actuator Control:** Allows control of robot joints and mechanisms, including motors, servos, and grippers.
*   **ROS 2 Integration:** Seamlessly integrates with ROS 2, enabling your ROS 2 nodes to communicate directly with the simulated robot and environment.

## SDF: Describing Worlds and Models

Gazebo uses the **Simulation Description Format (SDF)** to define robots (models) and their environments (worlds). SDF is an XML format designed to describe elements commonly found in robotics, including:

*   **`<model>`:** Defines a robot or an object within the simulation, similar to URDF but encompassing more physical properties for simulation (e.g., inertia, friction coefficients, joint limits, sensor definitions).
*   **`<world>`:** Describes the overall simulation environment, including gravity, lights, ground planes, static objects, and models to be spawned.

### Simple SDF Model Example (for a humanoid link)

Let's define a simple box model representing a rigid link of a humanoid:

```xml
<!-- my_gazebo_models/models/simple_link/model.sdf -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_link">
    <link name="body">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.1</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.1</iyy><iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>0.1 0.1 0.5</size></box></geometry>
        <material>
          <ambient>0.0 0.0 0.8 1.0</ambient>
          <diffuse>0.0 0.0 0.8 1.0</diffuse>
          <specular>0.0 0.0 0.8 1.0</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.1 0.1 0.5</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
```

This SDF defines a simple `simple_link` model with a single `body` link. It includes inertial properties, visual representation (a blue box), and collision geometry. In a full humanoid, multiple such links would be connected by joints.

## Launching Gazebo and Spawning Models

Gazebo can be launched directly or via ROS 2 launch files. To run Gazebo and spawn a model, you typically use `ros2 launch`.

### Example ROS 2 Launch File to Spawn a URDF in Gazebo

Let's assume you have a URDF file for your humanoid (e.g., `humanoid.urdf` in `my_robot_description` package). You can convert it to SDF on the fly and spawn it.

```python
# my_robot_bringup/launch/spawn_humanoid.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the URDF file path
    robot_description_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'humanoid.urdf'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(robot_description_path).read()}]
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_humanoid'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity
    ])
```

To run this, you would first need a `my_robot_description` package with your `humanoid.urdf` and a `my_robot_bringup` package with this launch file. Then:

```bash
ros2 launch my_robot_bringup spawn_humanoid.launch.py
```

## Exercises

1.  Explain the primary difference in purpose between URDF and SDF. When would you use one over the other in a ROS 2 Gazebo simulation context?
2.  Modify the `simple_link` SDF example to include a simple camera sensor. (Hint: Look up Gazebo's sensor documentation for `<sensor>` tags).
3.  Describe the steps involved in launching Gazebo with a custom robot model defined in URDF, using ROS 2 launch files.
