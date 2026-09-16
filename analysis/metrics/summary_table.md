# SteadyPath Comprehensive Experimental Metrics Summary

Generated automatically by Snehal's Analytics Engine (`compute_metrics.py`).

| Run ID | Controller | Time (s) | Max Lat Err (m) | RMS Lat Err (m) | RMS Yaw Err (deg) | Steering Jerk | Min Clear (m) | Collisions | MPC Latency (ms) | Goal Err (m) | Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `exp01_normal_nav_dest1` | **MPC** | 23.4 | 0.007 | 0.004 | 0.18° | 0.03 | 1389.695 | 0 | 10.4 | 0.079 | ✅ PASS |
| `exp02_curved_mpc` | **MPC** | 23.4 | 0.007 | 0.004 | 0.18° | 0.03 | 1389.695 | 0 | 10.4 | 0.079 | ✅ PASS |
| `exp02_curved_stanley` | **STANLEY** | 23.6 | 0.246 | 0.143 | 1.91° | 0.91 | 1389.536 | 0 | 0.2 | 0.128 | ⚠️ WARN |
| `exp05_dynamic_replan_mpc` | **MPC** | 23.9 | 0.387 | 0.134 | 4.81° | 0.63 | 0.887 | 0 | 10.4 | 0.080 | ⚠️ WARN |
| `exp06_multi_dest_A` | **MPC** | 19.9 | 0.007 | 0.004 | 0.18° | 0.03 | 1394.549 | 0 | 10.3 | 0.069 | ✅ PASS |
| `exp06_multi_dest_B` | **MPC** | 26.9 | 0.007 | 0.004 | 0.20° | 0.04 | 1386.701 | 0 | 10.4 | 0.070 | ✅ PASS |
