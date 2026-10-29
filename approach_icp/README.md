# approach_icp

Registration wrapper used to accumulate sparse Livox MID-360 scans.

- `registration` — PCL ICP / GICP and fast_GICP / fast_VGICP (CPU, CUDA) behind one interface
- `performance_metrics` — wall time, CPU time and RSS per registration
- `tf_utils` — TF ↔ Eigen helpers

Library target: `approach_icp_core`. Looks for fast_gicp in `../fast_gicp/local_install` first, then in the workspace.

## Docs

| Document | Contents |
|---|---|
| [icp_benchmark_ko.md](docs/icp_benchmark_ko.md) | Seven registration methods compared: wall time, CPU, memory |
| [fast_gicp_local_build_guide_ko.md](docs/fast_gicp_local_build_guide_ko.md) | Building fast_gicp locally |
| [fast_gicp_cuda12_6_report_ko.md](docs/fast_gicp_cuda12_6_report_ko.md) | CUDA 12.6 build notes |
| [fast_gicp_integration_report_ko.md](docs/fast_gicp_integration_report_ko.md) | CMake integration |
| [node_icp_cuda_accumulation_guide_ko.md](docs/node_icp_cuda_accumulation_guide_ko.md) | Accumulation node usage |
| [approach_icp_current_blockers_report_ko.md](docs/approach_icp_current_blockers_report_ko.md) | Build issues found and how they were fixed |

Part of the [approach](../README.md) workspace.
