# approach_mapping

ROS-independent core that turns ground/obstacle 2D points into three grids:

| Output | Meaning |
|---|---|
| Obstacle map | Cell state (unknown / free / obstacle) from persistent evidence with hysteresis |
| Clearance map | Distance to the nearest obstacle (8-neighbour propagation) |
| Heading-feasible map | Per heading bin, whether the rotated robot footprint fits |

Library target: `approach_map_lib`. Public API: `approach_map::Builder` (`beginUpdate` → `add*Observation` → `endUpdate`).

```text
include/approach_map/map/   types.hpp · builder.hpp · config_io.hpp
src/map/                    types · builder · evidence · clearance · feasibility · config_io
config/map_config.yaml      grid size, evidence weights, thresholds, footprint, heading bins
docs/                       design report, algorithm flow, code rationale
```

Design: [approach_map_report.md](docs/approach_map_report.md) · [map_algorithm_flow.md](docs/map_algorithm_flow.md) · [map_code_rationale.md](docs/map_code_rationale.md)

Part of the [approach](../README.md) workspace.
