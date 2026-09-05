# autobus_autoware_launch

A launch configuration repository for Autobus based on [Autoware](https://github.com/autowarefoundation/autoware), containing node configurations and their parameters.

## Core Localization Bug Fix & Patch Note

### `SmartPoseBuffer` Underflow Fix (`autoware_core`)
When running LiDAR at 20 Hz (e.g., dual Livox / Velodyne) while EKF odometry updates at 14–18 Hz, consecutive pointcloud frames can arrive between EKF updates. In upstream `autoware_localization_util`, `SmartPoseBuffer::pop_old()` popped poses aggressively, reducing `pose_buffer_.size() < 2`. This caused NDT scan matcher interpolation to fail, leading to `target mode is not available` / `scan_matching_status WARN`.

**Fix applied in `autoware_localization_util` (`smart_pose_buffer.cpp`):**
- `pop_old()` now ensures `pose_buffer_.size() > 2` before popping the front element.
- Retaining at least 2 poses guarantees that `interpolate()` can always linearly extrapolate even if consecutive sensor frames arrive before a new EKF update.
- Patch file provided at: `patches/0001-fix-autoware_localization_util-prevent-SmartPoseBuff.patch`

**To apply this patch to `src/core/autoware_core`:**
```bash
cd src/core/autoware_core
git apply ../../launcher/autobus_autoware_launch/patches/0001-fix-autoware_localization_util-prevent-SmartPoseBuff.patch
```

## Parameter Tuning Notes
- **NDT Scan Matcher**: `converged_param_nearest_voxel_transformation_likelihood: 2.3`, `initial_to_result_distance_tolerance_m: 3.0`, `resolution: 2.0`.
- **EKF Localizer**: `extend_state_step: 300`, `max_pose_queue_size: 50`, `tf_rate: 100.0`.
- **Localization Error Monitor**: `error_ellipse_size: 3.0`, `warn_ellipse_size: 2.0`, `error_ellipse_size_lateral_direction: 0.6`.
