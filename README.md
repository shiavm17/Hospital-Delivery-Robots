# 🏥 Hospital Delivery Robot

Autonomous hospital delivery robot simulation using ROS2, Nav2, and Gazebo.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## 📖 Overview

This project simulates an autonomous mobile robot that delivers medicines and supplies in a hospital environment. The robot uses SLAM for mapping, Nav2 for autonomous navigation, and custom task management for delivery operations.

## ✨ Features

- 🗺️ **SLAM Mapping** - Creates maps using Cartographer
- 🤖 **Autonomous Navigation** - Nav2-based path planning and obstacle avoidance
- 🏥 **Hospital Environment** - Realistic simulation with rooms, pharmacy, lab
- 📦 **Delivery System** - Automated and interactive delivery modes
- 🎮 **Interactive Control** - Manual destination selection

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11
- Python 3.10+

### Installation
```bash
# Clone repository
git clone https://github.com/YOUR_USERNAME/hospital-delivery-robot.git
cd hospital-delivery-robot

# Install dependencies
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs

# Build
colcon build
source install/setup.bash

# Set environment
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### Running the Simulation

**Terminal 1 - Launch Hospital World:**
```bash
source install/setup.bash
ros2 launch hospital_delivery_robot hospital_world.launch.py
```

**Terminal 2 - Launch Navigation:**
```bash
source install/setup.bash
ros2 launch hospital_delivery_robot hospital_navigation.launch.py
```

**Terminal 3 - Interactive Delivery:**
```bash
source install/setup.bash
ros2 run hospital_control interactive_delivery
```

## 🏗️ Project Structure
```
hospital-delivery-robot/
├── src/
│   ├── hospital_delivery_robot/
│   │   ├── launch/              # Launch files
│   │   ├── worlds/              # Gazebo world files
│   │   ├── maps/                # Generated maps
│   │   ├── config/              # Nav2 parameters
│   │   └── CMakeLists.txt
│   └── hospital_control/
│       ├── hospital_control/
│       │   ├── delivery_manager.py
│       │   └── interactive_delivery.py
│       └── setup.py
└── README.md
```

## 🎯 Locations

- **Room 101, 102, 103** - Patient rooms
- **Pharmacy** - Medicine pickup
- **Laboratory** - Lab results
- **Home Base** - Robot starting position

## 📸 Screenshots

Add screenshots of your simulation here!

## 🛠️ Customization

### Add New Locations

Edit `delivery_manager.py`:
```python
self.locations['new_room'] = {'x': 3.0, 'y': 4.0, 'z': 0.0}
```

### Modify Hospital Layout

Edit `worlds/hospital_simple.world` to add/modify rooms and obstacles.

## 📝 Future Enhancements

- [ ] Multi-floor navigation with elevators
- [ ] Dynamic obstacle avoidance (moving people)
- [ ] Battery management system
- [ ] Priority-based task scheduling
- [ ] Web-based monitoring dashboard

## 🤝 Contributing

Contributions welcome! Please open an issue or submit a pull request.

## 📄 License

MIT License - see LICENSE file for details

## 👤 Author

**Your Name**
- GitHub: [@shiavm17](https://github.com/shiavm17)
- Email: shivamchaturvedi.in@gmail.com

## 🙏 Acknowledgments

- ROS2 Navigation Stack
- TurtleBot3 Simulation
- Gazebo Simulator
