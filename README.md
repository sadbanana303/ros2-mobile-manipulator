# Mobile Manipulator Pick-and-Place (ROS 2 Jazzy)

Autonomous pick-and-place with a mobile manipulator in Gazebo. The robot navigates to a pick table with Nav2, detects a cube, grasps it with MoveIt, drives to a second table, and places it.

![Mission demo](docs/mission.gif)

## Features
- Autonomous navigation with Nav2 (AMCL + planner + controller)
- Cube detection from the robot camera
- Arm motion planning and grasping with MoveIt 2
- Closed-loop fine positioning using Gazebo ground truth before each pick and place
- Full 5-stage mission state machine

## Mission stages
1. Drive to pick table
2. Pick cube
3. Drive to place table
4. Place cube
5. Mission complete

## Stack
ROS 2 Jazzy, Gazebo, Nav2, MoveIt 2, Python (rclpy)

## Run it
Each command goes in its own sourced terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/mm_ws/install/setup.bash
```
```bash
ros2 launch mm_robot sim.launch.py       # wait for Gazebo + 30 s
ros2 launch mm_robot nav.launch.py       # wait for "Managed nodes are active" twice
ros2 launch mm_robot moveit.launch.py    # wait for "You can start planning now!"
ros2 run mm_robot detector.py
ros2 run mm_robot mission.py --ros-args -p use_sim_time:=true
```

## Results
### Detection
![Detection](docs/07_detection.png)

### Mission complete
![Mission complete](docs/08_complete.png)

## Repository structure
- `src/mm_robot/scripts/` mission and detector nodes
- `src/mm_robot/config/` Nav2 parameters
- `src/mm_robot/urdf/` robot description
- `docs/` screenshots and demo GIF

## Known limitations
- Nav2 pose can differ from Gazebo truth, so the mission uses a fine-positioning step with ground truth.
- Simulation only; the mission takes several minutes in real time.
