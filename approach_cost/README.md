# approach_cost

Scores feasible approach cells from [`approach_mapping`](../approach_mapping).

`cost_common` = weighted average of three min-max-normalized terms (over candidate cells only):

1. distance from the target
2. stability — number of state transitions in a window around the cell
3. distance from the robot

`compute_final_cost()` adds mode-specific terms; mode 1 uses `cost_common` as is.

Library target: `approach_cost_lib`. Config: [`config/cost_config.yaml`](config/cost_config.yaml). Design: [`docs/cost_design.md`](docs/cost_design.md).

Part of the [approach](../README.md) workspace.
