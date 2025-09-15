# 🐢 TurtleBot3 Sensor Logger & Simulation Overlay

This ROS 2-based project integrates a TurtleBot3 simulation with real-time sensor logging, LiDAR visualization, and autonomous motion control. Designed for robotics development, it includes a custom Gazebo world, a URDF model for RViz2, and modular ROS 2 nodes for sensor processing and behavior management.

---

## 📁 Project Structure

```
src/
├── sensor_logger/
│   ├── package.xml
│   ├── resource/
│   │   └── sensor_logger/
│   ├── sensor_logger/
│   │   ├── __init__.py
│   │   ├── __pycache__/
│   │   │   ├── __init__.cpython-310.pyc
│   │   │   └── logger_node.cpython-310.pyc
│   │   ├── logger_node.py
│   │   ├── movement_node.py
│   │   └── visualizer_node.py
│   ├── setup.cfg
│   ├── setup.py
│   └── test/
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
└── tb3_camera_overlay/
    ├── models/
    │   └── turtlebot3_waffle_pi_camera/
    │       ├── model.config
    │       └── model.sdf
    ├── urdf/
    │   └── turtlebot3_waffle_pi_camera.urdf
    └── worlds/
        └── office_camera.world
```

---

## 🚀 Launch Instructions

### 1. Build and Source the Workspace

```bash
colcon build
source install/setup.bash
```

### 2. Set Environment Variables

```bash
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=/workspace/src/tb3_camera_overlay/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
source ~/.bashrc
```

### 3. Launch Gazebo Simulation

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/workspace/src/tb3_camera_overlay/worlds/office_camera.world
```

### 4. Start Sensor Logger & Visualization Nodes

```bash
ros2 run sensor_logger logger_node
ros2 run sensor_logger visualizer_node
ros2 run sensor_logger move_robot
```

---

## 📊 Features

- ✅ **Real-time LiDAR Visualization**
- ✅ **CSV Logging**: LiDAR, IMU, and camera sensor data
- ✅ **URDF Support**: Robot model visualization in RViz2
- ✅ **Motion Control**: Circular motion via `/cmd_vel`
- ✅ **Obstacle Ring**: Useful for repeatable sensor testing
- ✅ **Camera Overlay World**: Enhanced simulation realism in Gazebo

---

## 🧭 RViz2 Visualization Setup

```bash
ros2 run robot_state_publisher robot_state_publisher urdf/turtlebot3_waffle_pi_camera.urdf
ros2 run joint_state_publisher joint_state_publisher
ros2 run rviz2 rviz2
```

### In RViz2:

1. Set **Fixed Frame** to: `base_link`  
2. Add **RobotModel** display  
3. Set **Description Topic** to: `/robot_description`

---

## 📌 Requirements

- ROS 2 Humble or newer
- Gazebo (compatible with your ROS 2 distribution)
- TurtleBot3 packages installed
- `colcon` build tool

---
