# LASER OpenVINS Core

This package serves as the **core integration point** for the OpenVINS (Visual-Inertial Navigation System) within the Laser UAV System (LUS). It acts as a configuration manager and launch wrapper, ensuring that the standard `ov_msckf` algorithm runs correctly with the specific parameters and topic names required by our drone architecture.

## Overview

The `laser_open_vins_core` package is responsible for:
1.  **Orchestration:** Launching the OpenVINS node (`ov_msckf`) with dynamic ROS 2 namespaces and topic remappings.
2.  **Configuration Management:** Storing and loading calibrated parameters (intrinsics/extrinsics) and estimator settings for different camera/IMU setups (e.g., Realsense D435i, Bluefox).
3.  **Visualization:** Providing helper scripts and configurations for RViz and PlotJuggler to visualize the VIO state seamlessly.

It is designed to work in conjunction with `laser_vins_republisher` (for covariance rotation and path generation) and `laser_vins_imu_filter` (for signal conditioning).

## How It Works

This package creates an abstraction layer over the standard OpenVINS launch process. Instead of manually editing configuration files for every flight or drone, the launch files here accept arguments to select the hardware profile dynamically.

### 1. Dynamic Parameter Loading
The system uses the `open_vins_params_folder` argument to locate the correct configuration files.
-   Inside the `params/` directory, each subfolder (e.g., `rs_244622073910`) corresponds to a specific sensor set.
-   When the launch file runs, it constructs the path to `estimator_config.yaml` inside that folder and passes it to the `ov_msckf` node.
-   This allows multiple drones with different camera calibrations to share the same codebase.

### 2. Topic Abstraction
The launch file accepts generic arguments for sensor topics (`topic_imu`, `topic_camera0`) and handles the remapping to the estimator's expected inputs. It also automatically applies the UAV's `namespace` to ensure multi-robot compatibility.

### 3. Visualization Helpers
To make debugging easier, this package includes scripts (e.g., `refactor_rviz_config.sh`) that automatically update RViz configuration files with the current UAV namespace before launching the visualizer. This prevents the common issue of RViz listening to `/uav1/odom` when you are flying `/uav2`.

## Launch Files

### 1. `open_vins.launch.py`
This is the main launch file to start the Visual-Inertial Odometry estimator.

-   **Usage:**
    ```bash
    ros2 launch laser_open_vins_core open_vins.launch.py open_vins_params_folder:=rs_244622073910 namespace:=uav1
    ```

-   **Arguments:**
    -   `namespace` (default: `uav1`): Top-level namespace for the drone.
    -   `use_sim_time` (default: `false`): Set to `true` if running in simulation.
    -   `open_vins_params_folder` (default: `rs_238222072435`): **Crucial**. Specifies the folder name inside `params/` containing the calibration files for the specific sensor suite being used.
    -   `verbosity` (default: `DEBUG`): Logging level (`ALL`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `SILENT`).
    -   `topic_imu`: Topic name for the IMU stream.
    -   `topic_camera0`: Topic name for the first camera (infra1/left).
    -   `topic_camera1`: Topic name for the second camera (infra2/right).

### 2. `rviz.launch.py`
Launches RViz 2 with a pre-configured view for OpenVINS, automatically refactoring topic names based on the environment.

-   **Usage:**
    ```bash
    ros2 launch laser_open_vins_core rviz.launch.py
    ```

## Configuration

The `params/` directory contains subfolders for each specific sensor setup. To add a new drone or camera setup, create a new folder here containing the standard OpenVINS configuration files:
-   `estimator_config.yaml`: Core filter parameters (noise, features, window size).
-   `kalibr_imucam_chain.yaml`: Camera-IMU extrinsics and camera intrinsics.
-   `kalibr_imu_chain.yaml`: IMU noise characteristics.

### Helper Scripts

The package includes scripts in the `scripts/` directory to facilitate visualization:
-   `refactor_rviz_config.sh`: Automatically updates RViz configuration files with the correct UAV namespace.
-   `refactor_plotjuggler_config.sh`: Automatically updates PlotJuggler layouts.

## Packages Included

| Package | Description | Repository |
| :--- | :--- | :--- |
| `open_vins` | Contains the node that implements the OpenVINS algorithm. | [Link to repo](https://github.com/LASER-Robotics/open_vins) |
| `laser_vins_republisher` | Contains the node that formats and republishes the raw odometry output from OpenVINS. | [Link to repo](https://github.com/LASER-Robotics/laser_vins_republisher) |
| `laser_vins_imu_filter` | Contains the node that filters IMU data before it is fed into OpenVINS. | [Link to repo](https://github.com/LASER-Robotics/laser_vins_imu_filter) |

## **TODOs**

  * [ ] Create a code to align the ground truth odometry with the OpenVINS odometry
