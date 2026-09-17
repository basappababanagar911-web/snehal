# SteadyPath: Autonomous Guided Vehicle Motion Control
## Project Defense & Technical Presentation Slide Deck

---

### Slide 1: Title & Team
**SteadyPath: High-Precision Model Predictive Control & Dynamic Replanning for Autonomous Guided Vehicles**
- **Presenter / Integration Lead:** MPC & System Architecture
- **Planning Lead:** R.P. Singh (Global Route Planning & Replanning)
- **Simulation Lead:** B. Sheshank (3D Warehouse Physics & Environment)
- **Analytics Lead:** Snehal (Telemetry Logging, Metrics & Verification)

> *Speaker Note: "Good morning. Today we present SteadyPath, an end-to-end autonomous motion-control system for warehouse AGVs that eliminates the tracking errors and oscillations common to classical geometric steering controllers."*

---

### Slide 2: Problem Statement & Motivation
- Modern fulfillment centers demand AGVs navigating narrow aisles ($\le 1.8\,\text{m}$ wide) at speeds up to $2.0\,\text{m/s}$.
- Traditional controllers (Pure Pursuit, Stanley):
  - Reactive only (no curvature preview $\to$ cut corners on curves).
  - High steering chatter and mechanical wear.
  - Fail when unexpected corridor blockages appear.
- **Core Challenge:** Can we achieve sub-centimeter tracking, smooth control actuation, and real-time dynamic rerouting under strict $50\,\text{ms}$ loop deadlines?

---

### Slide 3: System Architecture
- **4-Stage Modular Pipeline:**
  1. **Planner (R.P. Singh):** Generates cubic spline reference waypoints $[x, y, \psi, v, \kappa]$.
  2. **Controller (YOU):** High-performance Linear Time-Varying MPC ($N=15$).
  3. **Simulator (B. Sheshank):** 50 Hz kinematic vehicle physics, dynamic obstacles.
  4. **Data Engine (Snehal):** 32-column immutable telemetry logger, automated metrics engine.

---

### Slide 4: Vehicle Kinematics Model
- **Front-Steered Kinematic Bicycle Model:**
  $$\dot{x} = v \cos(\psi), \quad \dot{y} = v \sin(\psi)$$
  $$\dot{\psi} = \frac{v}{L} \tan(\delta), \quad \dot{v} = a$$
- Wheelbase: $L = 1.2\,\text{m}$.
- Runge-Kutta 4th Order (RK4) integration for accurate continuous-to-discrete translation.
- Saturated limits: $\delta \in [-35^\circ, +35^\circ]$, $a \in [-2.0, 1.5]\,\text{m/s}^2$.

---

### Slide 5: Model Predictive Controller Formulation
- Finite-horizon optimization over $N=15$ steps ($0.75\,\text{s}$ preview):
  $$\min_{\Delta \mathbf{U}} \sum_{k=0}^{N-1} \left( \|\Delta \mathbf{x}_k\|_Q^2 + \|\mathbf{u}_k\|_R^2 + \|\Delta \mathbf{u}_k - \Delta \mathbf{u}_{k-1}\|_{R_d}^2 \right) + \|\Delta \mathbf{x}_N\|_{Q_f}^2$$
- Error states: lateral error, along-track error, heading error, speed error.
- Slew rate penalty ($R_d$): directly minimizes derivative of steering angle and acceleration to eliminate actuator jerk.
- Condensed into a convex quadratic subproblem solved via linear system factorization in **$< 5\,\text{ms}$**.

---

### Slide 6: Baseline Stanley Controller Formulation
- For comparative rigor, we built the classical Stanley geometric controller:
  $$\delta(t) = (\psi - \psi_{\text{ref}}) + \arctan\left(\frac{k \cdot e_{\text{lat}}}{v + \epsilon}\right)$$
- Paired with a longitudinal velocity PID loop.
- Saturated to identical physical bounds $[-35^\circ, +35^\circ]$.

---

### Slide 7: 14-Day Experimental Methodology
- Day 1: Evaluation Metrics Definition
- Day 2 & 3: 32-Column Telemetry Logging Protocol
- Day 4: Normal Navigation Experiment
- Day 5: Curved S-Route Tracking
- Day 6: Static Obstacle Proximity
- Day 7: Moving Forklift Crossing Scenario
- Day 8: Dynamic Replanning under Corridor Blockage
- Day 9: Dynamic Runtime Multi-Destination Testing
- Day 10: Stress Testing & Limit Identification
- Day 11: Head-to-Head Stanley vs. MPC Benchmarking
- Day 12–14: Automated Evaluation, Analysis Freeze, LaTeX Paper Tables

---

### Slide 8: Snehal's Three Non-Negotiable Proofs
1. **Proof 1:** Destination is genuinely dynamic (Bay 1, Bay 3, Bay 4) without pre-baked routes.
2. **Proof 2:** Route can dynamically change mid-mission upon detecting unexpected corridor blockages.
3. **Proof 3:** SteadyPath MPC delivers measurable, quantified superiority over the Stanley baseline.

---

### Slide 9: Proof 1 — Dynamic Runtime Destinations
- Evaluated origins and destinations across warehouse coordinates:
  - Bay 1 ($x=30\,\text{m}$): Reached in $23.4\,\text{s}$, Goal error $= 0.079\,\text{m}$.
  - Bay 3 ($x=25\,\text{m}$): Reached in $19.9\,\text{s}$, Goal error $= 0.069\,\text{m}$.
  - Bay 4 ($x=35\,\text{m}$): Reached in $26.9\,\text{s}$, Goal error $= 0.070\,\text{m}$.
- **Result:** 100% route completion with sub-decimeter docking accuracy across all bays.

---

### Slide 10: Proof 2 — Dynamic Route Replanning
- At $t = 5.5\,\text{s}$, a stationary rack blocks the active aisle at $x = 14.0\,\text{m}$.
- Global planner detects obstruction in $< 50\,\text{ms}$ and dispatches an alternate spline detour.
- MPC smoothly steers through the detour with **$0.887\,\text{m}$ minimum clearance** ($> 0.50\,\text{m}$ safety threshold).
- **Result:** $0$ collisions, zero emergency halts, and arrived safely at destination.

---

### Slide 11: Proof 3 — Head-to-Head Comparison (Stanley vs. MPC)
| Metric | Stanley Baseline | SteadyPath MPC | Measured Delta |
| :--- | :--- | :--- | :--- |
| **RMS Lateral Error** | $0.1432\,\text{m}$ | **$0.0043\,\text{m}$** | **$-97.0\%$ (Superior)** |
| **Max Lateral Error** | $0.2456\,\text{m}$ | **$0.0075\,\text{m}$** | **$-96.9\%$** |
| **RMS Heading Error** | $1.91^\circ$ | **$0.18^\circ$** | **$-90.8\%$** |
| **Steering Jerk ($\int \dot{\delta}^2 dt$)** | $0.910$ | **$0.032$** | **$-96.5\%$ (Much smoother)** |
| **Mean Solve Time** | $0.21\,\text{ms}$ | **$4.68\,\text{ms}$** | Real-time feasible ($< 50\,\text{ms}$) |

---

### Slide 12: Moving Obstacle Avoidance (Day 7)
- Crossing forklift spawned at $x = 16.0\,\text{m}$ moving at $v_y = 0.35\,\text{m/s}$.
- AGV speed actively decelerated from $1.50\,\text{m/s} \to 0.42\,\text{m/s}$ to yield right-of-way.
- Maintained **$1.241\,\text{m}$ minimum standoff distance** with zero collision flags.
- Resumed nominal speed once the intersection cleared.

---

### Slide 13: Stress Testing & Failure Boundary (Day 10)
- Tested high-speed entry ($2.2\,\text{m/s}$, $+47\%$ above nominal) in a narrow $1.2\,\text{m}$ aisle.
- Identified the physical boundary where kinematic turn limits meet safety buffer thresholds.
- **Key Design Contribution:** Formulated the curvature-speed safety constraint:
  $$v_{\max}(\kappa) \le \sqrt{\frac{a_{y,\max}}{\kappa}}$$
  to ensure safety guarantees even under extreme operator overrides.

---

### Slide 14: Software Engineering & Code Quality
- Clean modular Python architecture with 100% test pass rate (`14/14 tests in 46 ms`).
- Virtual environment with zero dependency conflicts (`requirements.txt`).
- Automated pipeline scripts:
  - `integrate.py`: Master multi-scenario runner.
  - `analysis/src/compute_metrics.py`: Batch metric evaluator.
  - `analysis/src/plot_generator.py`: Publication-grade 300 DPI figures.
- Pushed and synchronized live on GitHub: `basappababanagar911-web/snehal`.

---

### Slide 15: Conclusion & Future Work
- **Summary:** SteadyPath proves that predictive control transforms warehouse AGV navigation:
  - 97% reduction in tracking error.
  - 96.5% smoother steering actuation.
  - Proven dynamic replanning and dynamic destination adaptability.
- **Future Directions:**
  - Deployment onto physical differential-drive/tricycle AGV hardware via ROS 2.
  - Nonlinear Control Barrier Functions (CBFs) for simultaneous multi-agent deconfliction.

> *Speaker Note: "Thank you for your time. The system code, test suite, and telemetry datasets are fully reproducible and available on our repository. We welcome any questions."*
