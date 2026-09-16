# Baseline Benchmark: Stanley Controller vs. SteadyPath MPC (Day 11)

> **Snehal's Proof 3**: Quantifiable head-to-head comparison demonstrating why SteadyPath MPC
> is superior in tracking accuracy, curvature handling, and control smoothness.

| Scenario / Experiment | Metric | Baseline (Stanley) | Proposed (SteadyPath MPC) | Improvement / Difference |
| :--- | :--- | :--- | :--- | :--- |
| `exp02_curved` | **RMS Lateral Error** | `0.1432 m` | `0.0043 m` | **+97.0%** (Better) |
| | **Max Lateral Error** | `0.2456 m` | `0.0075 m` | **+96.9%** |
| | **RMS Heading Error** | `1.91°` | `0.18°` | **+90.8%** |
| | **Steering Jerk / Smoothness** | `0.91` | `0.03` | **+96.5%** (Smoother) |
| | **Min Obstacle Clearance** | `1389.536 m` | `1389.695 m` | `+0.159 m` |
| | **Goal Distance Error** | `0.1276 m` | `0.0794 m` | `-0.0482 m` |
