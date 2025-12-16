---
id: nodes-topics-services
title: ROS 2 Nodes, Topics, and Services
sidebar_label: Nodes, Topics, Services
---

# ROS 2 Nodes, Topics, and Services: The Communication Backbone

In the previous section, we introduced the core components of ROS 2 architecture. Now, we'll dive deeper into the practical implementation of Nodes, Topics, and Services, which are the fundamental building blocks for inter-process communication in ROS 2. We will primarily use `rclpy`, the Python client library, for our examples.

## 1. ROS 2 Nodes

As discussed, a node is a process that performs computation. Each node in ROS 2 should ideally handle a single, well-defined task to promote modularity and reusability.

### Creating a Simple ROS 2 Node (Python)

Let's create a basic `minimal_publisher` node that initializes ROS 2 and then shuts down.

First, ensure you are in your ROS 2 workspace's `src` directory (e.g., `~/ros2_ws/src`). If you haven't already, create a new Python package:

```bash
ros2 pkg create --build-type ament_python my_ros2_pkg
```

Now, inside `my_ros2_pkg/my_ros2_pkg`, create a Python file named `minimal_node.py`:

```python
# my_ros2_pkg/my_ros2_pkg/minimal_node.py

import rclpy
from rclpy.node import Node

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal Node has been started.')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()
    rclpy.spin_once(minimal_node, timeout_sec=1)
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To make this node executable and discoverable, edit `my_ros2_pkg/setup.py` and add the following inside the `entry_points` dictionary:

```python
# ...
entry_points={
    'console_scripts': [
        'minimal_node = my_ros2_pkg.minimal_node:main',
    ],
},
# ...
```

Now, build your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_pkg
source install/setup.bash
```

And run your node:

```bash
ros2 run my_ros2_pkg minimal_node
```

You should see the output: `[INFO] [minimal_node]: Minimal Node has been started.`

## 2. ROS 2 Topics (Publish-Subscribe)

Topics enable asynchronous, many-to-many data streaming. Publishers send messages, and subscribers receive them.

### Creating a Publisher Node

Modify `minimal_node.py` (or create a new file `minimal_publisher.py` in the same directory and update `setup.py` accordingly):

```python
# my_ros2_pkg/my_ros2_pkg/minimal_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard string message type

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher) # Keep node alive until Ctrl+C
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Update `setup.py` if you created a new file:

```python
# ...
entry_points={
    'console_scripts': [
        'minimal_node = my_ros2_pkg.minimal_node:main',
        'minimal_publisher = my_ros2_pkg.minimal_publisher:main',
    ],
},
# ...
```

Rebuild and source your workspace.

### Creating a Subscriber Node

Create a new file `minimal_subscriber.py` in `my_ros2_pkg/my_ros2_pkg`:

```python
# my_ros2_pkg/my_ros2_pkg/minimal_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Update `setup.py`:

```python
# ...
entry_points={
    'console_scripts': [
        'minimal_node = my_ros2_pkg.minimal_node:main',
        'minimal_publisher = my_ros2_pkg.minimal_publisher:main',
        'minimal_subscriber = my_ros2_pkg.minimal_subscriber:main',
    ],
},
# ...
```

Rebuild and source. Run publisher in one terminal, subscriber in another:

```bash
# Terminal 1
ros2 run my_ros2_pkg minimal_publisher

# Terminal 2
ros2 run my_ros2_pkg minimal_subscriber
```

## 3. ROS 2 Services (Request-Response)

Services enable synchronous, one-to-one communication for request-response patterns.

### Creating a Service Server Node

Create `minimal_service.py` in `my_ros2_pkg/my_ros2_pkg`:

```python
# my_ros2_pkg/my_ros2_pkg/minimal_service.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Standard service type

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a} b={request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Update `setup.py`:

```python
# ...
entry_points={
    'console_scripts': [
        # ... existing entries ...
        'minimal_service = my_ros2_pkg.minimal_service:main',
    ],
},
# ...
```

### Creating a Service Client Node

Create `minimal_client.py` in `my_ros2_pkg/my_ros2_pkg`:

```python
# my_ros2_pkg/my_ros2_pkg/minimal_client.py

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        f'Result of add_two_ints: for {minimal_client.req.a} + {minimal_client.req.b} = {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Update `setup.py`:

```python
# ...
entry_points={
    'console_scripts': [
        # ... existing entries ...
        'minimal_service = my_ros2_pkg.minimal_service:main',
        'minimal_client = my_ros2_pkg.minimal_client:main',
    ],
},
# ...
```

Rebuild and source. Run service server in one terminal, client in another:

```bash
# Terminal 1
ros2 run my_ros2_pkg minimal_service

# Terminal 2 (provide two integers as arguments)
ros2 run my_ros2_pkg minimal_client 5 3
```

## Exercises

1.  Modify the `minimal_publisher` node to publish a custom message type (you'll need to define a `.msg` file and update `package.xml` and `CMakeLists.txt` for your package).
2.  Extend the `minimal_service` and `minimal_client` to perform a different mathematical operation (e.g., multiplication or subtraction).
3.  Explain how ROS 2 nodes facilitate a modular design in robotics, and why this is beneficial for large-scale projects like humanoid robots.