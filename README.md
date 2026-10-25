# Camera–LiDAR Sensor Fusion Based Obstacle Avoidance for an Autonomous Mobile Robot in Gazebo

**Student:** Navdeep Doriya  
**Roll Number:** M26MER005  
**Degree Program:** M.Tech in Robotics  
**Department:** Department of Mechanical Engineering  
**Institution:** Indian Institute of Technology Jodhpur, Rajasthan, India  
**Course:** M.Tech Individual Project (Mid-Term Progress Submission)  

---

## 1. Project Overview

This repository contains the software implementation, simulation assets, and technical mid-term progress documentation for a **Camera–LiDAR Sensor Fusion Obstacle Avoidance System** developed for an autonomous mobile robot (TurtleBot3 Waffle Pi) simulated in Gazebo.

The system combines monocular computer vision (color segmentation, morphological filtering, spatial moments) and planar LiDAR range querying within an angular acceptance cone to track obstacles of interest and trigger reactive collision avoidance maneuvers.

### Target Environment & Tech Stack
* **Operating System:** Ubuntu 20.04 LTS (Focal Fossa)
* **Middleware:** Robot Operating System (ROS Noetic Ninjemys)
* **Simulator:** Gazebo 11
* **Language:** Python 3.8
* **Vision & Image Processing:** OpenCV 4.x, `cv_bridge`
* **Robot Platform:** TurtleBot3 Waffle Pi (`export TURTLEBOT3_MODEL=waffle_pi`)

---

## 2. Repository Structure

```
robot_fusion_ws/
├── models/
│   └── red_obstacle.sdf             # SDF model definition of the 0.3m³ red box
├── src/
│   └── robot_sensor_fusion/
│       ├── CMakeLists.txt           # Catkin build configuration
│       ├── package.xml              # ROS package manifest
│       └── scripts/
│           └── obstacle_avoidance.py# Main Camera-LiDAR fusion controller node
├── figures/                         # Simulation snapshots and terminal logs
│   ├── fig_gazebo_camera_view.png   # Perspective Gazebo view with rqt_image_view
│   ├── fig_gazebo_world_overhead.png# Overhead view illustrating pillar failure mode
│   ├── fig_controller_terminal_log.png # Diagnostic telemetry terminal log
│   ├── fig_obstacle_spawn.png       # Gazebo model spawning log
│   └── fig_cmd_vel_echo.png         # rostopic echo /cmd_vel verification
├── main.tex                         # Official IEEE conference LaTeX report
├── MTech_Robotics_Midterm_Progress_Report_Navdeep_Doriya.docx # Formatted Word report
├── generate_report_docx.py          # Script generating the styled Word document
└── README.md                        # Project documentation and reproduction guide
```

---

## 3. Step-by-Step Installation and Reproduction Guide

Follow these instructions on your Ubuntu 20.04 LTS workstation running ROS Noetic.

### Step 1: Set Up ROS Catkin Workspace
If you do not already have the workspace configured:
```bash
# Create catkin workspace
mkdir -p ~/robot_fusion_ws/src
cd ~/robot_fusion_ws/src

# Clone or copy robot_sensor_fusion package into src/
# Place obstacle_avoidance.py in ~/robot_fusion_ws/src/robot_sensor_fusion/scripts/
chmod +x ~/robot_fusion_ws/src/robot_sensor_fusion/scripts/obstacle_avoidance.py

# Build workspace
cd ~/robot_fusion_ws
catkin_make
source devel/setup.bash
```

### Step 2: Install TurtleBot3 Simulation Dependencies
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-turtlebot3-gazebo \
  ros-noetic-turtlebot3-description \
  ros-noetic-cv-bridge \
  ros-noetic-rqt-image-view \
  python3-opencv \
  python3-numpy
```

### Step 3: Launch Gazebo Simulation
Open Terminal 1:
```bash
source /opt/ros/noetic/setup.bash
source ~/robot_fusion_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Step 4: Spawn the Red Target Obstacle
Open Terminal 2:
```bash
source /opt/ros/noetic/setup.bash
rosrun gazebo_ros spawn_model \
  -sdf \
  -file ~/robot_fusion_ws/models/red_obstacle.sdf \
  -model red_obstacle \
  -x -0.8 -y -0.5 -z 0.3
```

### Step 5: (Optional) Launch Visual Camera Monitor
Open Terminal 3:
```bash
source /opt/ros/noetic/setup.bash
rosrun rqt_image_view rqt_image_view
# In rqt_image_view GUI, select topic: /camera/rgb/image_raw
```

### Step 6: Run the Fusion Controller Node
Open Terminal 4:
```bash
source /opt/ros/noetic/setup.bash
source ~/robot_fusion_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
rosrun robot_sensor_fusion obstacle_avoidance.py
```

### Step 7: Inspect Velocity Commands
Open Terminal 5:
```bash
source /opt/ros/noetic/setup.bash
rostopic echo /cmd_vel
```

---

## 4. Current Implementation Summary

* **Topic Subscriptions:**
  * `/camera/rgb/image_raw` (`sensor_msgs/Image`)
  * `/scan` (`sensor_msgs/LaserScan`)
* **Topic Publication:**
  * `/cmd_vel` (`geometry_msgs/Twist`)
* **Vision Pipeline:**
  * OpenCV dual-band HSV thresholding for red (`[0, 100, 80]`–`[10, 255, 255]` & `[170, 100, 80]`–`[180, 255, 255]`)
  * Morphological opening with $5 \times 5$ kernel
  * Contour area gating ($A > 300\text{ px}^2$)
  * Horizontal bearing estimation: $\theta_{\text{obj}} = -\bar{x} \cdot \frac{\text{FOV}}{2}$ assuming $60^\circ$ FOV
* **LiDAR Filtering:**
  * Angular discrepancy normalization to $[-\pi, \pi]$
  * Acceptance cone filter $|\Delta\theta| \le 5.0^\circ$
  * Minimum valid distance extraction: $d_{\text{obs}} = \min \{r_i\}$
* **Control Loop (10 Hz):**
  * Watchdog timer halts robot if sensor data $> 1.0\text{ s}$ stale
  * Forward velocity $v_x = 0.10\text{ m/s}$ when path is clear
  * Obstacle threshold $d_{\text{stop}} = 0.60\text{ m}$ halts translation ($v_x = 0$) and triggers rotation $\omega_z = \pm 0.40\text{ rad/s}$

---

## 5. Experimental Observations & Identified Limitations

During simulation approach trials:
* Visual detection tracked the target bearing between $-0.0^\circ$ and $-0.2^\circ$.
* Filtered LiDAR distance decreased monotonically: $1.11\text{ m} \rightarrow 1.01\text{ m} \rightarrow 0.92\text{ m} \rightarrow 0.79\text{ m} \rightarrow 0.70\text{ m} \rightarrow 0.60\text{ m}$.
* At $d < 0.60\text{ m}$, the avoidance maneuver triggered, halting forward translation and rotating at $+0.40\text{ rad/s}$.

### Identified Failure Mode
When the red cube was placed adjacent to a cylindrical concrete pillar in `turtlebot3_world`:
1. The robot steered counter-clockwise away from the cube.
2. Because the controller **only** monitors LiDAR rays within $\pm 5^\circ$ of the red target, it had no spatial awareness of the pillar.
3. The robot turned directly into the pillar and became immobilized.

---

## 6. Roadmap for Pending Work

1. **Multi-Sector LiDAR Scanning:** Partition scan into Front ($[-15^\circ, +15^\circ]$), Left ($[+15^\circ, +75^\circ]$), and Right ($[-75^\circ, -15^\circ]$) sectors to detect non-red obstacles and boundaries.
2. **Clearance-Based Steering:** Turn toward the direction with maximum free clearance ($\max(d_{\text{left}}, d_{\text{right}})$).
3. **Finite State Machine:** Implement states (`FORWARD`, `EVALUATE`, `TURN_LEFT`, `TURN_RIGHT`, `RECOVERY`) with dwell hysteresis to eliminate steering chatter.
4. **Coordinate Frame Alignment:** Integrate ROS `tf2` to properly project rays between `camera_rgb_optical_frame` and `base_scan`.
5. **Simulation Benchmarking:** Execute 20 repeated trials per world configuration and quantify:
   * Collision Count
   * Avoidance Success Rate (%)
   * Minimum Obstacle Clearance (m)
   * Trajectory Length (m)
   * Goal Completion Time (s)

---

## 7. Instructions for Publishing to GitHub

Since this repository is currently maintained locally on the development workstation, use the following steps to publish it:

```bash
cd ~/robot_fusion_ws/src/robot_sensor_fusion

# Initialize local git repository
git init

# Stage all files
git add CMakeLists.txt package.xml scripts/ models/ README.md

# Commit
git commit -m "Initial commit: Camera-LiDAR fusion obstacle avoidance for TurtleBot3"

# Link to GitHub repository
git branch -M main
git remote add origin https://github.com/NavdeepDoriya/camera-lidar-obstacle-avoidance.git

# Push to GitHub
git push -u origin main
```

*Note: Replace `NavdeepDoriya` with your exact GitHub username after creating the empty repository on GitHub.*

---

## 8. Author & Academic Declaration

This is an **individual academic project** submitted by **Navdeep Doriya (Roll No: M26MER005)** in partial fulfillment of the requirements for the M.Tech Robotics degree at the Indian Institute of Technology Jodhpur. All simulation experiments, sensor fusion algorithms, and documentation are independently developed by the author.
