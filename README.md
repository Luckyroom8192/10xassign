# Custom DWA Local Planner

This package is an implementation of Dynamic Window Approach (DWA) local planner from scratch for a Turtlebot3 robot in ROS 2 Humble and Gazebo.

The `dwa_planner_node` subscribes to `/odom` and `/scan` topics to know about the robot's state and get an idea of its environment. It generates a possible set of feasible trajectories based on the constraints we set on v and w, and then scores them based on how near it is to a goal, how far it is from an obstacle, and the velocity it travels, and then publishes the best velocity command to the `/cmd_vel` topic.

## Features

* **Dynamic Window Sampling:** Samples velocities based on the robot's current velocity and acceleration limits.
* **Trajectory Prediction:** Simulates a unicycle model for an amr to predict future paths.
* **Cost-Based Evaluation:** Scores trajectories using a weighted cost function for:
    * Distance to goal
    * Obstacle clearance
    * Forward velocity
* **RViz Visualization:** Publishes all sampled trajectories (`/visualization_marker_array`) with invalid paths in red, valid paths in blue, and the chosen path in green.

---

## Dependencies

* **ROS 2 Humble Hawksbill**
* **Gazebo Classic** (version 11.10.2)
* **Turtlebot3 Simulation:** `ros-humble-turtlebot3-simulations`

I'm proceeding here assuming you have turtlebot3 simulation installed else install it by running
```bash
sudo apt update
sudo apt install ros-humble-turtlebot3-simulations

## Steps

1. Clone the repo to your local system
2. Get inside the workspace root (replace '~10xassign' with path of your workspace root)
```bash
cd ~/10xassign
3. Build the package
```bash
colcon build --package-select 10xassign
4. source your workspace and launch the simulation (replace '~10xassign' with path of your workspace root)
```bash
source ~/10xassign/install/setup.bash
ros2 launch 10xassign dwa_planner.launch.py
5. In the RViz popup add SetGoal and set a goal in the rviz window

Voila, There you have it... You should see the bot moving now while avoiding obstacles. I haven't properly tuned the cost function weights yet (Goal desire and Obstacle fear), but incase if you want to tune it to your liking you can go ahead and find `{'w_goal_dist': 3.0}` `{'w_obstacle': 0.5}` `{'w_velocity': 0.1}` in the `dwa_planner_node` section which you can find in launch/dwa_planner.launch.py




