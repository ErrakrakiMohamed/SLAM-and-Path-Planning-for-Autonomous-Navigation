# Autonomous Indoor Delivery Robot

> A real-time autonomous navigation system for indoor delivery robots using SLAM and advanced path planning

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()

## Overview

This project implements a complete autonomous navigation system for indoor delivery robots, developed as part of our Real-Time Embedded Systems course. The system enables compact wheeled robots to navigate complex indoor environments like offices and hospitals while delivering items safely and efficiently.

### Core Capabilities

- **SLAM (Simultaneous Localization and Mapping)** - Real-time environment mapping and self-localization
- **Dynamic Path Planning** - Intelligent obstacle avoidance with real-time trajectory optimization
- **Robust Navigation** - Reliable operation in changing environments with moving obstacles

## Key Features

✅ **Real-time 2D mapping** using advanced SLAM algorithms  
✅ **Autonomous goal-oriented navigation** with collision avoidance  
✅ **Dynamic obstacle detection** and path re-planning  
✅ **High-precision localization** using particle filter methods  
✅ **Simulation-ready** with full Gazebo integration  

## Technology Stack

| Component | Technology | Purpose |
|-----------|------------|---------|
| **Framework** | ROS2 Humble | Core robotics platform |
| **SLAM** | Gmapping | 2D mapping and localization |
| **Navigation** | Nav2 Stack | Path planning and execution |
| **Localization** | AMCL | Adaptive Monte Carlo localization |
| **Simulation** | Gazebo | Virtual testing environment |
| **Visualization** | RViz2 | Real-time data visualization |
| **Language** | Python | Node implementation |

## Quick Start

### Prerequisites

- Ubuntu 20.04 LTS or newer
- ROS2 Humble Hawksbill
- Colcon build tools

### Installation

```bash
# Clone the repository
git clone <YOUR_REPO_URL>
cd autonomous-delivery-robot

# Build the workspace
colcon build --symlink-install

# Source the environment
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Running the System

#### 1. Launch Simulation Environment
```bash
ros2 launch my_robot_description robot_world.launch.py
```

#### 2. Start SLAM Mapping
```bash
ros2 launch my_slam_project slam_gmapping_launch.py
rviz2
```

#### 3. Build Your Map
- Use keyboard teleoperation to explore the environment
- Watch the map build in real-time through RViz2
- Save the completed map:
```bash
ros2 run nav2_map_server map_saver_cli -f indoor_map
```

#### 4. Autonomous Navigation
```bash
ros2 launch my_navigation_project navigate_full_stack.launch.py \
  map:=/path/to/indoor_map.yaml \
  use_sim_time:=True
```

Use RViz2 to set initial pose and navigation goals with the interactive tools.

## Project Architecture

```
├── SLAM Module
│   ├── Gmapping Integration
│   └── Real-time Map Building
├── Localization
│   ├── AMCL Particle Filter
│   └── Pose Estimation
├── Path Planning
│   ├── Global Path Planning
│   ├── Local Trajectory Generation
│   └── Dynamic Obstacle Avoidance
└── Navigation Stack
    ├── Costmap Management
    ├── Behavior Trees
    └── Recovery Behaviors
```

## Advanced Features

### Dynamic Obstacle Management
Our system features advanced real-time obstacle detection and avoidance:
- Dynamic costmap updates from sensor data
- DWA (Dynamic Window Approach) local planning
- Predictive obstacle motion analysis
- Seamless path re-planning without mission interruption

### Real-time Performance
Optimized for real-time operation with:
- Low-latency sensor processing
- Efficient path computation algorithms
- Adaptive behavior based on environment complexity

## Future Roadmap

- [ ] **3D SLAM Integration** - Upgrade to Cartographer for volumetric mapping
- [ ] **Enhanced Sensor Fusion** - Integrate depth cameras and IMU data
- [ ] **Multi-Robot Coordination** - Fleet management capabilities
- [ ] **Physical Hardware Deployment** - Real robot implementation
- [ ] **Advanced UI/UX** - Intuitive control interface
- [ ] **Machine Learning Integration** - Predictive navigation behaviors

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

## Team

- **[Member 1 Name]** - Project Lead & SLAM Development
- **[Member 2 Name]** - Navigation Stack & Path Planning
- **[Member 3 Name]** - Simulation & Testing

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<p align="center">
  <strong>Built with ❤️ for RTS Final Projct</strong>
</p>
