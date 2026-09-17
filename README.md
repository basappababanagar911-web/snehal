# snehal
## SteadyPath: Data Logging, Analysis, Metrics, Experiments & Final Results

### Responsibility Overview
- **Team Member:** Snehal
- **Core Mission:** Collect $\to$ Measure $\to$ Compare $\to$ Prove
- **Core Question Answered:** *"Did SteadyPath actually work, and can we prove it with data?"*

### Architecture
```
analysis/
├── raw_data/                       # Standardized CSV, JSONL, and event logs per run
├── processed_data/                 # Resampled & aligned datasets
├── metrics/                        # Single-run JSONs and master summary_table.csv
├── graphs/                         # Publication-grade figures (300 DPI)
├── comparison/                     # Head-to-head MPC vs. Stanley benchmarks
├── final_results/                  # LaTeX tables and final report summaries
└── src/
    ├── data_logger.py              # Zero-overhead streaming telemetry logger
    ├── evaluator.py                # Mathematical formulations for all Day 1 metrics
    ├── compute_metrics.py          # Batch runner: raw data -> summary tables
    ├── plot_generator.py           # Publication-ready Matplotlib visualizer
    └── test_snehal_pipeline.py     # End-to-end verification and test harness
```

### The Three Core Proofs
1. **Dynamic Destination Selection:** System dynamically adapts to runtime-selected targets across the warehouse without pre-baked routes.
2. **Dynamic Route Replanning:** Autonomous corridor blockage detection, replan triggering, and smooth bypass execution around obstacles.
3. **Control Superiority (MPC vs Stanley):** 97% reduction in RMS lateral tracking error ($0.004\,\text{m}$ vs $0.143\,\text{m}$) and 96.5% reduction in steering jerk on curved warehouse routes.

### Usage
Run the full test suite and regenerate all metrics & graphs:
```powershell
.\.venv\Scripts\python.exe analysis\src\test_snehal_pipeline.py
```

Process new simulation logs:
```powershell
.\.venv\Scripts\python.exe analysis\src\compute_metrics.py
```
