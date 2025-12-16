---
id: api-reference
title: API Reference for Physical AI & Humanoid Robotics
sidebar_label: API Reference
---

# API Reference for Physical AI & Humanoid Robotics

This document provides reference information for key APIs and interfaces used in Physical AI and humanoid robotics applications. It covers ROS 2 interfaces, NVIDIA Isaac ROS packages, and other relevant APIs.

## ROS 2 Core APIs

### Node Interface
```python
import rclpy
from rclpy.node import Node

class PhysicalAINode(Node):
    def __init__(self):
        super().__init__('physical_ai_node')
        # Initialize publishers, subscribers, services, etc.
```

### Publisher Interface
```python
from std_msgs.msg import String

publisher = self.create_publisher(String, 'topic_name', 10)
msg = String()
msg.data = 'Hello, world!'
publisher.publish(msg)
```

### Subscriber Interface
```python
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: {msg.data}')

subscription = self.create_subscription(
    String,
    'topic_name',
    listener_callback,
    10
)
```

### Service Interface
```python
from example_interfaces.srv import AddTwoInts

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    return response

service = self.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
```

### Action Interface
```python
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
```

## Common Message Types

### Standard Messages
* `std_msgs/Bool` - Boolean value
* `std_msgs/Int32` - 32-bit integer
* `std_msgs/Float64` - 64-bit floating point
* `std_msgs/String` - String value
* `std_msgs/ColorRGBA` - RGBA color values

### Geometry Messages
* `geometry_msgs/Point` - 3D point coordinates
* `geometry_msgs/Pose` - Position and orientation
* `geometry_msgs/Twist` - Linear and angular velocity
* `geometry_msgs/Transform` - Transformation between coordinate frames

### Sensor Messages
* `sensor_msgs/Image` - Image data
* `sensor_msgs/LaserScan` - LIDAR scan data
* `sensor_msgs/PointCloud2` - 3D point cloud
* `sensor_msgs/Imu` - Inertial measurement unit data

## NVIDIA Isaac ROS APIs

### Image Pipeline Accelerators
```python
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image

# Hardware-accelerated image processing
image_processor = self.create_subscription(
    Image,
    'image_raw',
    self.process_image_callback,
    10
)
```

### Detection and Perception
```python
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

# Object detection results
detections = self.create_subscription(
    Detection2DArray,
    'detections',
    self.detection_callback,
    10
)
```

## Navigation2 APIs

### Action Interfaces
```python
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# Navigation goal
goal = NavigateToPose.Goal()
goal.pose = PoseStamped()
# Set target pose
```

### Services
```python
from nav2_msgs.srv import LoadMap
from nav2_msgs.srv import ManageLifecycleNodes

# Map loading service
map_loader = self.create_client(LoadMap, '/map_server/load_map')
```

## MoveIt! APIs

### Motion Planning
```python
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import RobotState

# Motion planning interface
move_group = self.create_client(MoveGroupAction, 'move_group')
```

## Common Services and Parameters

### Robot State Services
* `/get_robot_state` - Retrieve current robot state
* `/set_parameters` - Configure robot parameters
* `/list_parameters` - List available parameters

### TF (Transform) Services
* `/tf` - Transformation between coordinate frames
* `/tf_static` - Static transformations
* `tf2_ros` - Advanced transform library

## Service Response Codes

### Common Result Codes
* `SUCCESS (0)` - Operation completed successfully
* `FAILURE (1)` - Operation failed
* `IN_PROGRESS (2)` - Operation still in progress
* `CANCELED (3)` - Operation was canceled
* `UNKNOWN (4)` - Unknown result

## Parameter Definitions

### Common Robot Parameters
* `robot_description` - URDF of the robot
* `use_sim_time` - Whether to use simulation time
* `update_rate` - Rate at which to update controllers
* `tf_prefix` - Prefix for TF frame names

### Physical AI Parameters
* `ai_model_path` - Path to AI model files
* `confidence_threshold` - Minimum confidence for detections
* `max_processing_time` - Maximum allowed processing time
* `safety_timeout` - Timeout for safety-critical operations

## Quality of Service (QoS) Profiles

### Available Profiles
* `qos_profile_sensor_data` - For sensor data (best effort)
* `qos_profile_services_default` - For services (reliable)
* `qos_profile_parameters` - For parameter changes
* `qos_profile_action_status_default` - For action status

### Custom QoS Settings
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

custom_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

## Error Handling

### Common Error Types
* `RCLError` - ROS communication errors
* `ParameterException` - Parameter-related errors
* `InvalidStateError` - State machine errors
* `ActionError` - Action-specific errors

### Exception Handling Pattern
```python
try:
    # ROS 2 operations
    result = service_client.call(request)
except Exception as e:
    self.get_logger().error(f'Service call failed: {e}')
```

## Hardware Interface APIs

### Joint State Interface
```python
from sensor_msgs.msg import JointState

# Joint position, velocity, effort
joint_state = JointState()
joint_state.name = ['joint1', 'joint2', 'joint3']
joint_state.position = [0.0, 0.0, 0.0]
joint_state.velocity = [0.0, 0.0, 0.0]
joint_state.effort = [0.0, 0.0, 0.0]
```

### Control Interface
```python
from control_msgs.msg import JointTrajectoryControllerState

# Joint trajectory commands
trajectory_publisher = self.create_publisher(
    JointTrajectory,
    '/joint_trajectory_controller/joint_trajectory',
    10
)
```

## Vision and Perception APIs

### OpenCV Integration
```python
from cv_bridge import CvBridge
import cv2

cv_bridge = CvBridge()
cv_image = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
```

### Object Detection Results
```python
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

# Detection with confidence scores
for detection in detections.detections:
    for hypothesis in detection.results:
        confidence = hypothesis.hypothesis.classification[0].score
        if confidence > confidence_threshold:
            # Process detection
```

## Large Language Model APIs

### OpenAI API Integration
```python
import openai

response = openai.ChatCompletion.create(
    model="gpt-3.5-turbo",
    messages=[
        {"role": "system", "content": "You are a helpful robotics assistant."},
        {"role": "user", "content": user_command}
    ]
)
```

### Local LLM Integration
```python
# Using Hugging Face transformers
from transformers import pipeline

generator = pipeline('text-generation', model='gpt2')
result = generator(user_input, max_length=100)
```

## Safety and Monitoring APIs

### Emergency Stop Interface
```python
from std_msgs.msg import Bool

def emergency_stop_callback(self, msg):
    if msg.data:
        self.emergency_stop()

stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
```

### System Monitoring
```python
from diagnostic_msgs.msg import DiagnosticArray

# System health monitoring
diagnostics = self.create_subscription(
    DiagnosticArray,
    '/diagnostics',
    self.diagnostic_callback,
    10
)
```

## Configuration Examples

### Launch File Parameters
```xml
<launch>
  <node pkg="my_robot_package" exec="my_robot_node" name="physical_ai_node">
    <param name="model_path" value="$(find-pkg-share my_robot_package)/models/robot_model.onnx"/>
    <param name="confidence_threshold" value="0.7"/>
    <param name="processing_rate" value="30"/>
  </node>
</launch>
```

### Parameter YAML Files
```yaml
physical_ai_node:
  ros__parameters:
    ai_model_path: "/path/to/model.onnx"
    confidence_threshold: 0.7
    max_processing_time: 0.1
    safety_timeout: 5.0
    debug_mode: false
```

---

*This API reference provides stubs and examples for common interfaces in Physical AI and humanoid robotics. For complete and up-to-date API documentation, refer to the official ROS 2, NVIDIA Isaac, and other relevant documentation sources.*