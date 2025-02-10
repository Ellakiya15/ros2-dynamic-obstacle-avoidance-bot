# Dynamic Obstacle Avoidance Robot in ROS 2

## 📌 Overview
This project implements a **custom robot** that performs **autonomous navigation using Nav2** in **Gazebo** while avoiding **dynamic obstacles**.

## 🛠 Features
✅ Autonomous Navigation with ROS 2 Nav2  
✅ Dynamic Obstacle Avoidance  
✅ Custom C++ Node for Navigation (`navigation_node`)  
✅ RViz Visualization  
✅ Gazebo Simulation

## Installation & Setup
### 1️⃣ Clone the Repository
```bash
git clone https://github.com/your_username/Dynamic-obstacle-avoidance-bot.git
cd Dynamic-obstacle-avoidance-bot
```

### 2️⃣ Build the package
```bash
colcon build
source install/setup.bash
```
### 3️⃣ Start Simulation
```bash
ros2 launch bot_gazebo spawn_robot.launch.xml
```

### 4️⃣ Start Navigation
```bash
ros2 launch bot_nav navigation.launch.xml
```
### 5️⃣ Start Rviz
```bash
rviz2
```
### 6️⃣ Navigation node
```bash
ros2 run bot_nav navigation_node
```
## How navigation_node Works
The ROS 2 C++ node (navigation_node) is responsible for managing navigation goals interactively.

#### 1️⃣ Set Initial Pose:

The node sets an initial pose estimate to localize the robot in the simulated world.
#### 2️⃣ Send a Navigation Goal:

The user enters a target location (x, y, yaw), and the node sends this as a goal to the Nav2 stack.
#### 3️⃣ Print "Goal Reached" When Goal is Achieved:

Once the robot reaches the target location, "Goal Reached" is printed in the terminal.
#### 4️⃣ Prompt for Next Goal:

After reaching the goal, the user is prompted to enter a new target location.
The process repeats until the user decides to exit.

## Expected Terminal Output
```bash
[INFO] [navigation_node]: Setting initial pose...
Enter new goal (x, y, yaw):  2.0 3.5 1.57
[INFO] [navigation_node]: Navigating to (2.0, 3.5, 1.57)...
[INFO] [navigation_node]: Goal reached!
[INFO] [navigation_node]: Adding marker at (2.0, 3.5)
Enter new goal (x, y, yaw):  -1.0 4.0 0.0
[INFO] [navigation_node]: Navigating to (-1.0, 4.0, 0.0)...
[INFO] [navigation_node]: Goal reached!
[INFO] [navigation_node]: Adding marker at (-1.0, 4.0)
Enter new goal (x, y, yaw):  3.0 -2.5 1.0
[INFO] [navigation_node]: Navigating to (3.0, -2.5, 1.0)...
[WARN] [navigation_node]: Goal failed!
Enter new goal (x, y, yaw):  
```
