# IMU Drift Suppression for robot_localization (Zero-Velocity Pose Hold)

A small ROS 1 node, written in 2020 for an Autonomous Mobile Robot (AMR). It fixes a practical problem: when the robot is **standing still**, IMU bias makes the EKF pose from [`robot_localization`](http://wiki.ros.org/robot_localization) slowly drift, even though the wheels haven't moved.

## How it works

```
wheel odom ──► is the robot still?
                 │ yes, for ≥ timer (5 s)
                 ▼
          latch last filtered pose
                 │
                 ▼  every pose_correct_timer (10 s)
          reset EKF via set_pose  ◄── stops the drift
                 │
                 ▼  robot starts moving
          one final reset, release hold → EKF runs normally
```

This is a simple form of a **zero-velocity update (ZUPT)**. Wheel odometry is trusted to tell the node when the robot is truly stationary, and during that time the fused pose is held fixed instead of being left to integrate IMU noise.

## Usage

```bash
cd ~/catkin_ws/src
git clone https://github.com/vishrutsingh1/IMU_ekf_drift.git
cd .. && catkin_make && source devel/setup.bash

# with your robot_localization EKF already running:
roslaunch imu_fusion imu_fusion.launch
```

| Parameter | Default | Meaning |
|---|---|---|
| `~timer` | `5.0` | Seconds of zero velocity before the pose is latched |
| `~pose_correct_timer` | `10.0` | Seconds between pose resets while stationary |
| `~set_pose_service` | `/set_pose` | `robot_localization` SetPose service |

| Topic / service | Type | Role |
|---|---|---|
| `odom` | `nav_msgs/Odometry` | Wheel odometry, used for stillness detection |
| `odometry/filtered` | `nav_msgs/Odometry` | EKF output, the pose that gets latched |
| `set_pose` | `robot_localization/SetPose` | Resets the EKF state |

**Requirements:** ROS Noetic and `robot_localization`. The original was written on ROS Melodic; to run it there, change the script's first line to `#!/usr/bin/env python`.

## Limitations

- Stillness detection checks for velocity of *exactly* zero. That works with encoder odometry that reports a true 0, but a noisy source needs a small threshold instead.
- It does not estimate IMU bias. It only stops the pose drifting while the robot is still. A filter that tracks the bias as part of its state would fix the root cause.

## License

MIT
