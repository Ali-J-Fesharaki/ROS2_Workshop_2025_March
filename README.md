This resource is designed to help students and enthusiasts grasp the core concepts of both ROS 1 and ROS 2, focusing on practical implementations such as publishers, services, and actions. The repo features hands-on examples—primarily using the classic `turtlesim` simulator—including the foundation for scalable multi-agent tasks (planned for future releases).

**Note:** This repository is under active development. The multi-agent project (scaling turtles on turtlesim) is not yet available in this version, but the codebase is structured to allow for these future expansions.

---

## Contents

- **ros1_tut/**: ROS 1 tutorials and scripts
  - Publisher/Subscriber examples (e.g., velocity publisher)
  - Service and Action servers/clients
  - Custom messages and service definitions
  - Utility scripts

- **ros2_tut/**: ROS 2 tutorials and scripts
  - Updated publisher/subscriber and service/client nodes
  - Custom interfaces for spawning multiple agents
  - Launch files for orchestrating complex node setups

---

## Features

### ROS 1 Tutorials (`ros1_tut`)
- **Publisher Example:**  
  [`velocity_publisher.py`](ros1_tut/src/publisher_subscriber/scripts/velocity_publisher.py)  
  Publishes velocity commands to move and rotate a turtle in turtlesim.

- **Service Example:**  
  [`spawn_service_server.py`](ros1_tut/src/service_server_client/scripts/spawn_service_server.py)  
  Implements a service to spawn multiple turtles with unique names and randomized positions.

- **Custom Action:**  
  [`Robot2dNav.action`](ros1_tut/src/action_server_client/action/Robot2dNav.action)  
  Defines a 2D navigation action interface for turtles.

- **Utilities:**  
  Bash scripts and tools to aid automation and testing.

### ROS 2 Tutorials (`ros2_tut`)
- **Custom Interface Example:**  
  [`MultipleSpawner.srv`](ros2_tut/src/custom_interfaces/srv/MultipleSpawner.srv)  
  Service definition for spawning multiple agents in a scalable way.

- **Publisher/Subscriber Package:**  
  [`setup.py`](ros2_tut/src/publisher_subscriber/setup.py)  
  ROS 2 Python package for publisher/subscriber nodes.

- **Integrated Launch File:**  
  [`service_server_client.launch.py`](ros2_tut/src/service_server_client/launch/service_server_client.launch.py)  
  Orchestrates the launch of turtlesim, service server, and client with parameterized settings.

---

## Getting Started

### Prerequisites

- ROS 1 (Noetic recommended) for `ros1_tut`
- ROS 2 (Foxy/Humble recommended) for `ros2_tut`
- Python 3.x
- `turtlesim` package installed in both ROS environments

### Building and Running

#### ROS 1

```bash
cd ros1_tut/src
catkin_make
source devel/setup.bash
# Example: Run velocity publisher
rosrun publisher_subscriber velocity_publisher.py
```

#### ROS 2

```bash
cd ros2_tut/src
colcon build
source install/setup.bash
# Example: Launch service server/client with turtlesim
ros2 launch service_server_client service_server_client.launch.py
```

---

## Educational Goals

- Understand the differences and similarities between ROS 1 and ROS 2
- Learn to implement basic ROS nodes: publishers, subscribers, services, and actions
- Get hands-on with turtlesim for rapid prototyping and visualization
- Prepare for advanced topics such as multi-agent robotics (coming soon!)

---

## Roadmap

- [ ] Add multi-agent turtlesim project with scalable agent management
- [ ] Expand ROS 2 examples to cover actions and more advanced interfaces
- [ ] Provide comprehensive documentation and step-by-step tutorials

---

## Contributing

Contributions, bug reports, and feature suggestions are welcome!  
Please open an issue or pull request.

---




