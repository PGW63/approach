# approach_preprocess

Point-cloud preprocessing shared by the approach runners: NaN removal, robot-body box filter, voxel downsampling and statistical outlier removal.

Library target: `approach_preprocess_lib` (PCL). Parameters are passed in by the caller; see `preprocess_*` and `robot_filter_*` in [`approach_map_runner/config/runner_config.yaml`](../approach_map_runner/config/runner_config.yaml).

Part of the [approach](../README.md) workspace.
