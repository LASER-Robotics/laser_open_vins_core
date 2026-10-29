# LASER OpenVINS Core

`laser_open_vins_core` integrates OpenVINS into the LASER UAV System (LUS). It provides camera/IMU profiles, launch files and visualization settings. OpenVINS estimates motion from images and IMU measurements; `laser_vins_republisher` converts its odometry for the estimation manager.

The integration package lives in `ros_packages/laser_open_vins_core`. [.gitman.yml](.gitman.yml) selects the OpenVINS and republisher repositories.

## Install and run

Install the component repositories from this directory:

```bash
gitman install
```

With their ROS packages available in the workspace:

```bash
colcon build --packages-up-to laser_open_vins_core laser_vins_republisher
source install/setup.bash
```

Select the profile for your camera and IMU:

```bash
export UAV_NAME=uav1
ros2 launch laser_open_vins_core open_vins.launch.py open_vins_params_folder:=rs_244622073910 namespace:=uav1
```

Start the odometry republisher in another terminal:

```bash
ros2 launch laser_vins_republisher vins_republisher.launch.py namespace:=uav1
```

Camera drivers and IMU filtering run separately. Use `laser_uav_imu_filter` when the sensor stream needs filtering. The OpenVINS launch starts only the estimator.

## Interfaces

The OpenVINS node is named `ov_msckf` under the selected namespace. For `uav1`:

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| Input | `topic_imu`, default `/uav1/rgbd/imu` | `sensor_msgs/msg/Imu` | Inertial measurements. |
| Input | `topic_camera0`, default `/uav1/rgbd/infra1/image_raw` | `sensor_msgs/msg/Image` | First camera images. |
| Input | `topic_camera1`, default `/uav1/rgbd/infra2/image_raw` | `sensor_msgs/msg/Image` | Second camera images when enabled by the profile. |
| Output | `/uav1/ov_msckf/odomimu` | `nav_msgs/msg/Odometry` | Estimated IMU state, consumed by the republisher. |
| Output | `/uav1/ov_msckf/poseimu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Estimated pose and covariance. |
| Output | `/uav1/ov_msckf/pathimu` | `nav_msgs/msg/Path` | Estimated path. |
| Output | `/uav1/ov_msckf/points_msckf`, `points_slam`, `points_aruco` | `sensor_msgs/msg/PointCloud2` | Feature visualizations under the same node prefix. |
| Output | `/uav1/ov_msckf/num_points_slam`, `num_points_msckf` | `std_msgs/msg/Float64` | Feature counts under the same node prefix. |

The republisher publishes `/uav1/vins_republisher/odometry` (`nav_msgs/msg/Odometry`) for the estimation manager. See its [README](https://github.com/LASER-Robotics/laser_vins_republisher) for transforms and configuration.

## Configuration

### Launch arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `namespace` | `UAV_NAME`, or `uav1` | Namespace for the OpenVINS node. |
| `use_sim_time` | `true` when `REAL_UAV=false` | Uses the simulation clock. |
| `open_vins_params_folder` | `rs_238222072435` | Profile folder inside the installed integration's `params/`. |
| `verbosity` | `DEBUG` | `ALL`, `DEBUG`, `INFO`, `WARNING`, `ERROR` or `SILENT`. |
| `topic_imu`, `topic_camera0`, `topic_camera1` | Paths in the interface table | Sensor inputs. Defaults are derived from `UAV_NAME`. |

If `namespace` differs from `UAV_NAME`, pass the input topic paths explicitly too.

### Calibration profiles

The [params directory](ros_packages/laser_open_vins_core/params/) contains `rs_238222072435`, `rs_243522071667`, `rs_244622073910` and `bluefox_simulation`. Each profile has:

| File | Contents |
| --- | --- |
| `estimator_config.yaml` | Filter, initialization and visual-tracker settings. |
| `kalibr_imucam_chain.yaml` | Camera intrinsics, distortion, camera/IMU extrinsics and timing. |
| `kalibr_imu_chain.yaml` | IMU noise and calibration settings. |

Use the profile calibrated for your sensor, mounting and image resolution. The estimator configuration uses OpenCV YAML syntax, including its `%YAML:1.0` header.

| Estimator parameter group | Meaning |
| --- | --- |
| `use_stereo`, `max_cameras` | Camera count and whether to use stereo constraints. |
| `use_fej`, `integration` | First-estimate Jacobians and state integration method. |
| `calib_cam_*`, `calib_imu_*` | Online calibration options for camera and IMU parameters. |
| `max_clones`, `max_slam`, `max_slam_in_update`, `max_msckf_in_update` | Sliding-window size and feature/update limits. |
| `init_*` | Static or dynamic initialization settings. |
| `try_zupt`, `zupt_*` | Zero-velocity detection and update settings. |
| `use_klt`, `num_pts`, `fast_threshold`, `grid_x`, `grid_y`, `min_px_dist` | Feature tracking method and distribution. |
| `track_frequency`, `downsample_cameras`, `num_opencv_threads`, `histogram_method` | Tracking rate, image processing and CPU settings. |
| `up_*` | Feature measurement noise and update thresholds. |
| `gravity_mag` | Gravity magnitude in m/s². |
| `relative_config_imu`, `relative_config_imucam` | Calibration files relative to the estimator profile. |
| `record_timing_*`, `save_total_state`, `filepath_*` | Timing and state output files. |

## Visualization

`rviz.launch.py` and `plotjuggler.launch.py` use the supplied layouts and namespace-adjustment scripts. The `tmux/bluefox_simulation/` session starts the VIO simulation and the LUS stack with its own sensor and manager settings.

For the algorithm, evaluation tools and citation, see the [OpenVINS README](https://github.com/LASER-Robotics/open_vins).
