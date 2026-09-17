# Baseline Benchmark: Stanley Controller vs. SteadyPath MPC (Day 11)

> **Snehal's Proof 3**: Quantifiable head-to-head comparison demonstrating why SteadyPath MPC
> is superior in tracking accuracy, curvature handling, and control smoothness.

| Scenario / Experiment | Metric | Baseline (Stanley) | Proposed (SteadyPath MPC) | Improvement / Difference |
| :--- | :--- | :--- | :--- | :--- |
| `exp02_curved` | **RMS Lateral Error** | `0.1432 m` | `0.0043 m` | **+97.0%** (Better) |
| | **Max Lateral Error** | `0.2456 m` | `0.0075 m` | **+96.9%** |
| | **RMS Heading Error** | `1.91°` | `0.18°` | **+90.8%** |
| | **Steering Jerk / Smoothness** | `0.91` | `0.03` | **+96.5%** (Smoother) |
| | **Min Obstacle Clearance** | `999.000 m` | `999.000 m` | `+0.000 m` |
| | **Goal Distance Error** | `0.1276 m` | `0.0794 m` | `-0.0482 m` |
| `run_curved` | **RMS Lateral Error** | `0.1236 m` | `0.0043 m` | **+96.5%** (Better) |
| | **Max Lateral Error** | `0.2122 m` | `0.0075 m` | **+96.5%** |
| | **RMS Heading Error** | `1.64°` | `0.18°` | **+89.3%** |
| | **Steering Jerk / Smoothness** | `0.26` | `0.03` | **+87.7%** (Smoother) |
| | **Min Obstacle Clearance** | `999.000 m` | `999.000 m` | `+0.000 m` |
| | **Goal Distance Error** | `0.0839 m` | `0.0794 m` | `-0.0045 m` |
| `run_moving` | **RMS Lateral Error** | `0.1236 m` | `0.0068 m` | **+94.5%** (Better) |
| | **Max Lateral Error** | `0.2122 m` | `0.0295 m` | **+86.1%** |
| | **RMS Heading Error** | `1.64°` | `0.85°` | **+48.1%** |
| | **Steering Jerk / Smoothness** | `0.26` | `1.43` | **-449.2%**  |
| | **Min Obstacle Clearance** | `1.089 m` | `1.206 m` | `+0.117 m` |
| | **Goal Distance Error** | `0.0839 m` | `0.0865 m` | `+0.0026 m` |
