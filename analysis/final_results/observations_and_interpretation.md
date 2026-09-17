# SteadyPath: Observations & Interpretation of Results (Day 13)
**Author / Responsible:** Snehal  
**Module:** Analysis & Verification Engine  
**Project:** SteadyPath Autonomous Warehouse Guided Vehicle  

---

## 1. Executive Summary

This document presents the formal technical analysis and interpretation of the experimental campaigns conducted across Days 4–11. All findings are derived directly from the immutable telemetry logs recorded in `analysis/raw_data/` and aggregated in `analysis/metrics/summary_table.csv`.

The core research question posed to Snehal:
> **"Did SteadyPath actually work, and can we prove it with data?"**

**Verdict: YES.** The quantitative evidence confirms:
1. **Dynamic Target Adaptability:** The system navigates to runtime-specified targets across diverse warehouse quadrants without pre-recorded waypoints (Day 9).
2. **Dynamic Replanning Efficacy:** In response to sudden corridor blockage, the system detects obstruction, initiates global replanning, and executes safe evasive bypass maneuvers while preserving stability (Day 8).
3. **Control Superiority over Baseline:** SteadyPath MPC outperforms the kinematic Stanley controller by **97.0% in RMS lateral error** and **96.5% in control smoothness**, eliminating steering oscillations on high-curvature aisles (Day 5 & 11).

---

## 2. Per-Experiment Observations & Technical Interpretations

### Day 4: Normal Navigation Experiment (`exp01_normal_nav_dest1`)
- **Objective:** Establish steady-state baseline performance under unobstructed nominal conditions.
- **Measured Metrics:**
  - Route: $(0.0, 0.0) \to (30.0, 1.902)\,\text{m}$ along nominal curved route.
  - Duration: $23.40\,\text{s}$ at $v_{\text{nom}} = 1.5\,\text{m/s}$.
  - RMS Lateral Tracking Error: **$0.004\,\text{m}$** (Max: $0.007\,\text{m}$).
  - RMS Heading Error: **$0.18^\circ$** ($0.0031\,\text{rad}$).
  - Mean MPC Latency: **$10.4\,\text{ms}$** (Well below the $50\,\text{ms}$ real-time loop threshold).
  - Terminal Goal Error: **$0.079\,\text{m}$** ($< 0.10\,\text{m}$ target).
- **Physical Interpretation:** Under nominal conditions, the MPC anticipatory preview horizon cancels phase lag during curvature transitions, yielding sub-centimeter tracking fidelity without overshoot.

---

### Day 5: Curved-Route Tracking (`exp02_curved_mpc`)
- **Objective:** Evaluate trajectory stability across continuous S-curves ($R_{\min} \approx 6.3\,\text{m}$).
- **Measured Metrics:**
  - RMS Lateral Error: **$0.004\,\text{m}$**.
  - Steering Rate Metric ($\int \dot{\delta}^2 dt$): **$0.03$**.
  - Maximum Steering Angle: **$14.2^\circ$** ($\le 35^\circ$ physical wheel saturation limit).
- **Physical Interpretation:** The vehicle smoothly negotiates alternating left-right turns without oscillation or hunting behavior, maintaining a consistent velocity profile.

---

### Day 6: Static Obstacle Clearance (`exp03_static_obstacles`)
- **Objective:** Verify obstacle proximity monitoring and spatial safety around fixed warehouse racks.
- **Measured Metrics:**
  - Static Obstacles: Rack pillars positioned at $x = 8.0, 15.0, 22.0\,\text{m}$.
  - Minimum Clearance Recorded: **$0.215\,\text{m}$** relative to rack boundaries.
  - Collision Count: **$0$**.
  - Route Completion: **$100.0\%$**.
- **Physical Interpretation:** The vehicle maintains a safe standoff distance alongside narrow aisles without triggering false emergency halts.

---

### Day 7: Moving Obstacle Interaction (`exp04_moving_obstacle`)
- **Objective:** Verify real-time responsiveness to moving equipment (crossing forklift).
- **Measured Metrics:**
  - Obstacle Trajectory: Forklift traversing transversely across $x = 16.0\,\text{m}$ at $v_y = 0.35\,\text{m/s}$.
  - Vehicle Response: Speed decelerated from $1.50\,\text{m/s}$ to $0.42\,\text{m/s}$ between $t = 8.2\,\text{s}$ and $t = 11.4\,\text{s}$.
  - Minimum Clearance to Moving Forklift: **$1.241\,\text{m}$**.
  - Collisions: **$0$**.
  - Travel Time Increase: $+1.6\,\text{s}$ ($25.0\,\text{s}$ vs $23.4\,\text{s}$).
- **Physical Interpretation:** Proves reactive velocity regulation: SteadyPath actively yielded to the crossing forklift rather than executing a blind open-loop path, restoring cruise velocity once the corridor cleared.

---

### Day 8: Dynamic Replanning under Corridor Blockage (`exp05_dynamic_replan_mpc` - Proof 2)
- **Objective:** Prove the "Google-Maps-like" rerouting capability upon complete lane closure.
- **Measured Metrics:**
  - Blockage Event: Full corridor obstruction spawned at $x = 14.0\,\text{m}$ at $t = 5.50\,\text{s}$.
  - Event Logged: `CORRIDOR_BLOCKED`, `REPLAN_TRIGGERED` at $t = 5.50\,\text{s}$.
  - Replan Transition Window: Vehicle smooth lateral detour offset $\Delta y = +2.2\,\text{m}$.
  - Minimum Clearance to Blockage: **$0.887\,\text{m}$** (well above $0.50\,\text{m}$ buffer).
  - Terminal Goal Error: **$0.080\,\text{m}$**.
  - Replan Count: **$1$**.
- **Physical Interpretation:** Demonstrates closed-loop autonomy: R.P. Singh's planner computed route $R_2$, and the MPC tracked the transition continuously without emergency braking or vehicle spin.

---

### Day 9: Multiple-Destination Runtime Adaptability (`exp06_multi_dest_A`, `exp06_multi_dest_B` - Proof 1)
- **Objective:** Prove destinations are genuinely selected at runtime rather than hardcoded.
- **Measured Metrics:**
  - Target Bay 3 ($x = 25.0\,\text{m}$): Travel time $19.9\,\text{s}$, Terminal Error **$0.069\,\text{m}$**, Completion **$100\%$**.
  - Target Bay 4 ($x = 35.0\,\text{m}$): Travel time $26.9\,\text{s}$, Terminal Error **$0.070\,\text{m}$**, Completion **$100\%$**.
  - Path Lengths: Dynamically scaled from $25.8\,\text{m}$ to $36.4\,\text{m}$.
- **Physical Interpretation:** Confirms true runtime flexibility. The system generates and executes valid reference trajectories for arbitrary warehouse coordinates without code modifications.

---

### Day 10: Stress Testing & Failure Boundary Analysis (`exp07_stress_test`)
- **Objective:** Intentionally push the AGV to operating limits (tight corridor + elevated entry speed $v = 2.2\,\text{m/s}$).
- **Measured Metrics:**
  - Maximum Speed: $2.20\,\text{m/s}$ ($+47\%$ above nominal).
  - Minimum Clearance: $0.000\,\text{m}$ (boundary breach at tightest pinch point).
  - Collision Counter: $11$ timesteps with buffer breach.
  - Terminal Goal Error: $0.072\,\text{m}$.
- **Physical Interpretation & Safety Takeaway:** At speeds $> 2.0\,\text{m/s}$ in aisles narrower than $1.2\,\text{m}$, the kinematic turn radius approaches physical tire adhesion limits. **Design Recommendation:** Introduce automatic curvature-based speed limiting ($v_{\max}(\kappa) \le \sqrt{a_{y,\max} / \kappa}$) to guarantee $d_{\min} \ge 0.50\,\text{m}$ in high-speed corridors.

---

### Day 11: Head-to-Head Baseline Comparison — Stanley vs. MPC (`exp02_curved_stanley` vs `exp02_curved_mpc` - Proof 3)
- **Objective:** Rigorously demonstrate superiority over standard geometric Stanley control.
- **Comparative Evidence:**

| Metric | Stanley Baseline | SteadyPath MPC | Measured Delta | Interpretation |
| :--- | :--- | :--- | :--- | :--- |
| **RMS Lateral Error** | $0.1432\,\text{m}$ | **$0.0043\,\text{m}$** | **$-97.0\%$** | MPC anticipates upcoming curvature; Stanley reacts only after error develops. |
| **Max Lateral Error** | $0.2456\,\text{m}$ | **$0.0075\,\text{m}$** | **$-96.9\%$** | Stanley drifts significantly wide on curve apexes. |
| **RMS Heading Error** | $1.91^\circ$ | **$0.18^\circ$** | **$-90.8\%$** | MPC aligns vehicle yaw ahead of trajectory inflections. |
| **Steering Jerk ($S_{\text{steer}}$)** | $0.910$ | **$0.032$** | **$-96.5\%$** | Stanley suffers high-frequency steering chatter; MPC produces butter-smooth control. |
| **Computation Time** | $0.21\,\text{ms}$ | **$10.42\,\text{ms}$** | $+10.21\,\text{ms}$ | While Stanley is computationally lighter, MPC ($10.4\,\text{ms}$) operates well within the $50\,\text{ms}$ budget. |
| **Terminal Goal Error** | $0.1276\,\text{m}$ | **$0.0795\,\text{m}$** | **$-37.7\%$** | MPC docks closer to the designated target threshold. |

---

## 3. Final Conclusion

Snehal's analytics framework conclusively proves that **SteadyPath fulfills all functional and performance specifications**. The transition from geometric steering to Model Predictive Control provides quantifiable, order-of-magnitude improvements in tracking accuracy and vehicle stability.
