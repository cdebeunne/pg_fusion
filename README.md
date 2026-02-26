# PG Fusion

PG Fusion is a ROS 2-based package for integrating pose graph optimization with visual SLAM and GNSS data. It provides a pipeline for fusing visual odometry, GNSS measurements, and other sensor data to produce accurate and robust localization and mapping.

## Features

- **Pose Graph Optimization**: Implements pose graph optimization using Ceres Solver to refine pose estimates.
- **Sensor Fusion**: Combines visual SLAM, GNSS, and IMU data for robust localization.
- **ROS 2 Integration**: Publishes and subscribes to ROS 2 topics for real-time operation.
- **Visualization**: Provides RViz visualization for trajectories, poses, and maps.
- **Configurable Pipeline**: Easily configurable through YAML files for different datasets and sensor setups.

## Installation

### Prerequisites

- **ROS 2 Galactic** or later
  - Separate branch for ROS 2 Jazzy
- [SaDVIO](https://github.com/ISAE-PNX/SaDVIO) that comes with all the dependencies, it must be installed as a library, not as a ROS2 node.

### Build Instructions

1. Clone the repository and into your ROS 2 workspace:
   ```bash
   cd ~/colcon_ws/src
   git clone https://github.com/your-repo/pg_fusion.git
   ```

2. Build the workspace:
   ```bash
   cd ~/colcon_ws
   colcon build
   ```

3. Source the workspace:
   ```bash
   source ~/colcon_ws/install/setup.bash
   ```

## Usage

### Launch the Node

To launch the `pg_fusion` node along with RViz for visualization, run:
```bash
ros2 launch pg_fusion pg_launch.xml
```

### Configuration

The behavior of the pipeline can be configured using the `config.yaml` file. Key parameters include:

- `slam_config_path`: Path to the SLAM configuration directory.
- `gnss_topic`: ROS topic for GNSS data.
- `thresh_cov`: Threshold for GNSS covariance.
- `window_size`: Sliding window size for pose graph optimization.
- `remove_z_estimate`: Whether to ignore the Z-axis estimate from GNSS.

### Visualization

The package provides RViz visualization for:

- **Trajectories**: `/pg_traj` and `/pg_traj_vo`
- **Poses**: `/pg_pose` and `/pg_slam`

### Example Dataset

TO DO

## Development

### Code Overview

- **Pipeline**: The main processing pipeline is implemented in [`pipeline.cpp`](pipeline.cpp) and [`pipeline.hpp`](pipeline.hpp).
- **Pose Graph**: Pose graph optimization logic is in [`poseGraph.cpp`](poseGraph.cpp) and [`poseGraph.hpp`](poseGraph.hpp).
- **Visualization**: RViz visualization utilities are in [`rosVisualizer.hpp`](rosVisualizer.hpp).
- **Sensor Subscribers**: ROS 2 subscribers for GNSS, IMU, and camera data are in [`sensorSubscriber.h`](sensorSubscriber.h).


