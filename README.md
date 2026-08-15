Multi-Filter Robot Teleoperation with Cyber Attack Detection

Overview

This repository contains four ROS-based implementations of state estimation filters for robot teleoperation with integrated cyber attack detection and mitigation. The system supports multiple filtering approaches (EKF, KF, UKF, PF) and can detect and respond to three types of cyber attacks:

- Flood Attack: High-frequency spoofed velocity commands (1000 Hz)
- Overflow Attack: Oversized LaserScan payload flooding the sensor bus
- Replay Attack: Stale odometry messages re-published to the system

Features

- Multiple Filter Implementations:
  - Extended Kalman Filter (EKF) with non-linear motion model
  - Linear Kalman Filter (KF) for baseline comparison
  - Unscented Kalman Filter (UKF) with numerical stability improvements
  - Particle Filter (PF) for Monte-Carlo state estimation

- Attack Detection & Mitigation:
  - Real-time anomaly detection with configurable thresholds
  - Attack-specific response strategies:
    - Flood: Corrective control override with safe velocity
    - Overflow: Multi-criteria overflow scoring (payload size, temporal consistency, range drops)
    - Replay: Variance-based pattern detection with timestamp analysis

- Enhanced LIDAR Processing:
  - Overflow detection via payload size thresholds
  - Front-range extraction and dead-reckoning consistency checks
  - Continuous overflow anomaly scoring (0.0-1.0)

- Data Logging:
  - Comprehensive CSV logging of all states, anomalies, and attack metrics
  - Timestamped data for post-processing and analysis
  - Particle standard deviation tracking (PF only)

 Filter Implementations

 Extended Kalman Filter (EKF)
- 6-DOF state: [x, y, θ, vx, vy, ω]
- Non-linear motion model with Jacobian linearization
- Attack-adaptive measurement noise
- Anomaly detection via residual analysis

 Linear Kalman Filter (KF)
- Linear motion model for baseline comparison
- Simplified state transition and observation matrices
- Attack-adaptive covariance inflation

 Unscented Kalman Filter (UKF)
- Sigma-point based propagation (no Jacobians required)
- Joseph-form covariance update
- Symmetry enforcement after operations
- Eigendecomposition-based covariance repair
- Progressive jitter for Cholesky factorization
- Divergence guard with covariance reset

Particle Filter (PF)
- 800 particles for 3-DOF state estimation
- Flood attack improvements:
  - Uses last-safe velocity for prediction during attack
  - 50 Hz corrective publisher to override flood commands
  - Suppressed resampling during flood to preserve diversity
- Overflow attack detection:
  - Dedicated overflow anomaly channel (separate from particle residual)
  - Three sub-scores: payload size, temporal consistency, range drop
  - Vehicle speed reduction during confirmed overflow

Dependencies

ROS dependencies
rospy
geometry_msgs
nav_msgs
sensor_msgs

Python packages
numpy
squaternion

Installation
Clone the repository
git clone <repository-url>
cd <repository-name>

Install Python dependencies
pip install numpy squaternion

Make scripts executable
chmod +x *.py

Source ROS environment
source /opt/ros/<your-ros-distro>/setup.bash

Usage

Running a Filter
Each filter implementation is self-contained in its own Python file:

EKF_MultipleCyberattack.txt - EKF Python Code
LKF_MultipleCyberattack.txt - LKF Python Code
PF_MultipleCyberattack.txt - PF Python Code
UKF_MultipleCyberattack.txt - UKF Python Code
odom_lidar_ekf_data_1777135835.csv - EKF CSV File
odom_lidar_kf_data_1777131388.csv - LKF CSV File
odom_lidar_ukf_data_1777224022.csv - UKF CSV File
pf_data_1777225403.csv  - PF CSV File

Teleoperation Controls

| Key | Action |
|-----|--------|
| Movement | |
| `w` | Forward + Turn Right |
| `z` | Backward + Turn Right |
| `a` | Forward + Turn Left |
| `d` | Backward + Turn Left |
| `l` | Turn Left (stationary) |
| `r` | Turn Right (stationary) |
| `f` | Forward (straight) |
| `b` | Backward (straight) |
| `s` | Stop |
| Attack Toggles | |
| `t` / `y` | Flood Attack ON / OFF |
| `v` / `m` | Overflow Attack ON / OFF |
| `q` / `e` | Replay Attack ON / OFF |
| Quit | |
| `Ctrl+C` | Exit program |

Attack Simulation
1. Flood Attack (`t`):
   - Spoofs 1000 Hz velocity commands
   - Filter predicts using safe velocity (EKF/UKF/PF)
   - Corrective control active (PF only)

2. Overflow Attack (`v`):
   - Publishes oversized LaserScan payloads
   - Overflow scoring triggers anomaly detection
   - Vehicle speed reduction on confirmation

3. Replay Attack (`q`):
   - Replays recorded odometry messages
   - Detected via positional/heading variance analysis
   - Filter adapts measurement noise accordingly

Output Data
Each filter generates a CSV file with the following columns:

| Column | Description |
|--------|-------------|
| `time` | UNIX timestamp |
| `attack_type` | Current attack mode (normal/flood/overflow/replay) |
| `odom_x, odom_y, odom_theta` | Raw odometry measurements |
| `lidar_x, lidar_y, lidar_front` | LIDAR measurements |
| `est_x, est_y, est_theta` | Filter state estimates |
| `diff` | Innovation/residual magnitude |
| `rate_of_change` | Rate of measurement change |
| `expected_change` | Expected change from control input |
| `covariance_trace` | Trace of covariance matrix (EKF/KF/UKF) |
| `particle_std_x/y/theta` | Particle standard deviation (PF only) |
| `overflow_score` | Overflow anomaly score (0.0-1.0) |
| `linear_vel, angular_vel` | Commanded velocities |
| `anomaly_detected` | Binary anomaly flag |
| `attack_active` | Binary attack flag |

Key Parameters
Common Parameters
- `process_noise`: Process noise covariance (default: 0.01)
- `measurement_noise`: Measurement noise covariance (default: 0.1)
- `error_threshold`: Anomaly detection threshold (default: 1.0)

EKF/UKF Specific
- `OVERFLOW_LIDAR_THRESHOLD`: LIDAR overflow detection threshold (default: 10,000 beams)
- `MAX_CONSECUTIVE_REPAIRS`: Covariance repair limit before reset (UKF, default: 10)

PF Specific
- `num_particles`: Number of particles (default: 800)
- `motion_noise`: Motion noise for particles (default: 0.08)
- `std_threshold`: Particle spread threshold for reset (default: 0.8)

Architecture
┌─────────────────────────────────────────────────────┐
│                   ROS Environment                   │
├─────────────────────────────────────────────────────┤
│                                                     │
│  ┌─────────────┐    ┌─────────────┐                 │
│  │   Odometry  │    │   LIDAR     │                 │ 
│  │  Subscriber │    │ Subscriber  │                 │
│  └──────┬──────┘    └──────┬──────┘                 │
│         │                  │                        │
│         └────────┬─────────┘                        │
│                  ▼                                  │
│         ┌────────────────┐                          │
│         │  State Filter  │                          │
│         │ (EKF/KF/UKF/PF)│                          │
│         └────────────────┘                          │
│                  │                                  │
│         ┌────────┴────────┐                         │
│         ▼                 ▼                         │
│  ┌─────────────┐   ┌─────────────┐                  │
│  │   Anomaly   │   │  Corrective │                  │
│  │  Detection  │   │   Control   │                  │
│  └─────────────┘   └─────────────┘                  │
│                                                     │
└─────────────────────────────────────────────────────┘

License
This project is licensed under the MIT License - see the LICENSE file for details.

Acknowledgments
- ROS community for robot middleware
- Contributors to the squaternion library for quaternion operations
