🤖 SLAM and Path Planning for Autonomous Navigation
📦 Project: Indoor Delivery Robot
This repository contains the code and documentation related to our autonomous navigation project for a mobile robot, developed as part of the Real-Time Embedded Systems course. The objective is to create a system capable of enabling a compact wheeled robot to navigate and deliver small items autonomously in complex and potentially dynamic indoor environments, such as offices or hospitals.

🚀 Project Overview
The core of this project lies in the integration of two fundamental robotic functionalities:

🗺️ Simultaneous Localization and Mapping (SLAM): The robot constructs a map of its environment while simultaneously determining its own position within that map.

🧭 Path Planning: Once the map is established and the robot's position is known, it calculates and executes the best collision-free path to a goal, adapting in real-time to new obstacles.

Without these capabilities, an autonomous delivery robot could not operate safely and efficiently in real-world, changing environments. Our project combines these functions to create a fully autonomous and robust navigation system.

✨ Key Features & Expected Outcomes
By the end of this project, our autonomous navigation system will be capable of:

🗺️ Creating an accurate 2D map of an unknown environment using SLAM (with Gmapping).

➡️ Navigating autonomously to a goal while actively avoiding obstacles.

⏱️ Demonstrating real-time navigation and pathfinding, even in dynamic environments (e.g., with moving people or displaced objects).

💡 Our Approach: Overcoming Existing Limitations
Many existing navigation solutions, while functional, present significant shortcomings, particularly concerning dynamic environment management. Basic approaches can be error-prone, resource-intensive, or fail to react in real-time to mobile obstacles.

Our solution is designed to address these weaknesses by leveraging the ROS2 ecosystem:

🗺️ Initial Mapping (SLAM): We use Gmapping for the initial construction of accurate 2D maps, providing a reliable foundation for navigation.

📍 Robust Localization: Thanks to AMCL (Adaptive Monte Carlo Localization) integrated within Nav2, our robot localizes itself with high precision on the map.

🛡️ Real-Time Dynamic Obstacle Management (Under Development): This is our key differentiator and a primary focus of ongoing development. By fully utilizing the ROS2 Navigation2 (Nav2) stack, we are working towards integrating dynamic costmap layers (updated by real-time sensor data) and advanced local planners (like DWA - Dynamic Window Approach). This will allow the robot to:

Detect obstacles that appear or move.

Dynamically recalculate its trajectory to avoid them without interrupting its mission.

Adapt to unforeseen situations in the environment.

This integrated approach enables intelligent and adaptive navigation, making repetitive tasks more efficient, reliable, and autonomous.

🛠️ Technologies Used
⚙️ Robot Operating System 2 (ROS2): Robotics development framework.

🗺️ Gmapping: ROS2 package for 2D SLAM.

🧭 Navigation2 (Nav2): Autonomous navigation stack for ROS2 (including AMCL, Costmaps, Global/Local Planners, Behavior Trees).

🖥️ Rviz: Visualization tool for ROS2.

** simulate Gazebo**: 3D simulator for testing the robot and its algorithms in a virtual environment.

💻 Python : Programming languages for ROS2 nodes.

🚀 Quick Start (Simulation Setup)
This project is primarily developed and tested in a simulation environment.

✅ Prerequisites
Ubuntu 20.04 LTS (or newer)

ROS2 (Humble Hawksbill distribution recommended)

colcon (ROS2 build tool)

1️⃣ Clone the Repository
git clone <YOUR_REPO_URL>
cd <YOUR_REPO_NAME>

2️⃣ Initialize and Build the ROS2 Workspace
cd ~/your_ros2_workspace/src
# Copy project packages here, or clone directly if this is the project root
colcon build --symlink-install

3️⃣ Source the ROS2 Environment
Ensure you source your ROS2 installation and workspace:

source /opt/ros/humble/setup.bash  # Or your ROS2 distribution
source install/setup.bash

4️⃣ Launch Simulation and Mapping (SLAM)
To start building a map of the simulated environment:

ros2 launch my_robot_description robot_world.launch.py # Launches the robot in Gazebo
ros2 launch my_slam_project slam_gmapping_launch.py   # Launches the Gmapping node
rviz2                                                 # Launches Rviz to visualize the map and robot

In Rviz, add LaserScan, Map, TF, and RobotModel displays.

Teleoperate the robot in Gazebo to explore the environment and build the map.

5️⃣ Save the Map
Once the map is satisfactory, save it:

ros2 run nav2_map_server map_saver_cli -f my_indoor_environment

This will create my_indoor_environment.yaml and my_indoor_environment.pgm.

6️⃣ Launch Autonomous Navigation
For autonomous navigation, use your saved map and the Nav2 stack:

ros2 launch my_navigation_project navigate_full_stack.launch.py map:=/path/to/my_indoor_environment.yaml use_sim_time:=True # Adapt the path
rviz2

In Rviz, use the "2D Pose Estimate" tool to initialize the robot's position on the map.

Use the "2D Nav Goal" tool to set a goal, and observe the robot navigating.

🚧 Next Steps & Future Perspectives
📈 Transition to 3D SLAM: Explore solutions like Cartographer for volumetric mapping.

➕ Advanced Sensor Integration: Add depth cameras for improved obstacle perception.

🎯 Advanced Dynamic Obstacle Avoidance: Integrate motion prediction algorithms.

🤝 Multi-Robot Navigation: Coordinate multiple robots in the same environment.

⚙️ Deployment on Physical Robot: Port the system to real hardware.

📱 Enhanced User Interface: Develop a more intuitive interface for interaction.

🧑‍🤝‍🧑 Team Members
[Member 1 Name]

[Member 2 Name]

[Member 3 Name]

📄 License
This project is licensed under the MIT License. See the LICENSE file for more details.
