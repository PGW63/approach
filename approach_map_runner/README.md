# approach_map_runner

ROS 2 nodes that run the approach libraries on live data.

| Executable | Role |
|---|---|
| `node_icp_cuda` | Accumulates and registers LiDAR scans on request (service: start, rate, target frame) |
| `approach_map_runner_node` | Preprocesses the cloud and publishes obstacle, clearance and heading-feasible maps |
| `approach_cost_runner_node` | Scores candidates and publishes the final cost map and best pose arrow |
| `approach_waiting` | Watches the obstacle map around the robot and calls the waiting service when enabled |

```bash
ros2 launch approach_map_runner map_runner_with_cost.launch.py use_sim_time:=false
```

Config: [`runner_config.yaml`](config/runner_config.yaml) · [`cost_runner_config.yaml`](config/cost_runner_config.yaml). Depends on `inha_interfaces`.

Part of the [approach](../README.md) workspace.
