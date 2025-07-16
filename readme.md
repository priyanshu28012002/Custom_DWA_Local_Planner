
# Dynamic Window Approach (DWA) - Implementation Plan

# Objective
Implement the **Dynamic Window Approach (DWA)** for local trajectory planning in mobile robots. The algorithm generates multiple forward trajectories and selects the best one based on motion constraints and cost functions related to goal direction, velocity, and obstacle clearance.

---

## 🧩 Key Implementation Steps

### 1. Get Current State `odom` `scan`


### 2. Generate Velocity Samples


### 3. Generate Multiple Trajectories


## 4. Cost Function Evaluation

### For each trajectory:
Calculate three scores:

#### 4.1 Heading to Goal Score
- Measures how well the final point of the trajectory aligns with the goal pose.
- Encourages motion toward the goal with correct orientation.

#### 4.2 Clearance Score
- Checks the distance from trajectory points to the nearest obstacle in the costmap.
- Penalizes trajectories that pass close to or through obstacles.

#### 4.3 Velocity Score
- Rewards faster motion to encourage efficient travel.
- Penalizes slower trajectories unless necessary for safety.

Each score is normalized and combined:

```
Total Score = α × Heading + β × Clearance + γ × Velocity
```

*Weights (α, β, γ) are tuning parameters based on robot/environment.*

---
### 5. Select Best Velocity



### 6. Publish to `cmd_vel`

![DWA Planner in Action](Image/dwa.gif)

## ROS 2 Humble – DWA Planner Project Setup Guide

This guide walks you through setting up a ROS 2 workspace, installing TurtleBot3, and launching your custom **Dynamic Window Approach (DWA)** planner.


###  Step 0: Create a ROS 2 Workspace

```bash
mkdir -p ~/dwa_ws/src
cd ~/dwa_ws
colcon build
source install/setup.bash
```

---

###  Step 1: Install TurtleBot3 Packages

Install the necessary TurtleBot3 packages from the ROS 2 Humble distribution:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* 
```

#### Set TurtleBot3 Model

You must define the TurtleBot3 model you are using (e.g., `burger`, `waffle`, or `waffle_pi`):

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

###  Step 2: Clone Your DWA Planner Package

Navigate to the `src` directory and clone your planner's repository:

```bash
cd ~/dwa_ws/src
git clone 
```

---

###  Step 3: Build the Workspace

```bash
cd ~/dwa_ws
colcon build --symlink-install
source install/setup.bash
```
---

### Step 4: Launch the System

Use the following launch commands to run your system components:

```bash
ros2 launch custom_dwa_planner turtle_bot.launch.py   # Load TurtleBot in simulation
ros2 launch custom_dwa_planner setup.launch.py        # Initialize transforms, sensors
ros2 launch custom_dwa_planner dwa.launch.py          # Start the DWA planner node
```
