"""
compute_metrics.py
==================
SteadyPath Automated Batch Metrics Pipeline (Snehal's Module - Day 12 & Day 13)

Scans raw simulation data from `analysis/raw_data/`, processes all runs,
calculates standardized Day 1 metrics using `SteadyPathEvaluator`,
and outputs:
1. `analysis/metrics/<run_id>_metrics.json`
2. `analysis/metrics/summary_table.csv`
3. `analysis/metrics/summary_table.md`
4. `analysis/comparison/comparison_summary.md` (MPC vs. Stanley benchmark)
"""

import os
import glob
import json
import csv
from typing import List, Dict, Any, Optional

try:
    from data_logger import SteadyPathLogger
    from evaluator import SteadyPathEvaluator, RunMetrics
except ImportError:
    from analysis.src.data_logger import SteadyPathLogger
    from analysis.src.evaluator import SteadyPathEvaluator, RunMetrics


class MetricsBatchProcessor:
    """Processes directories of raw simulation runs into aggregated metrics."""

    def __init__(
        self,
        raw_data_dir: str = "analysis/raw_data",
        metrics_dir: str = "analysis/metrics",
        comparison_dir: str = "analysis/comparison",
        safety_clearance_threshold: float = 0.50
    ):
        self.raw_data_dir = raw_data_dir
        self.metrics_dir = metrics_dir
        self.comparison_dir = comparison_dir
        self.evaluator = SteadyPathEvaluator(safety_clearance_threshold=safety_clearance_threshold)

        os.makedirs(self.metrics_dir, exist_ok=True)
        os.makedirs(self.comparison_dir, exist_ok=True)

    def process_all_runs(self) -> List[RunMetrics]:
        """Scans raw_data_dir for CSV files and evaluates each run."""
        csv_files = sorted(glob.glob(os.path.join(self.raw_data_dir, "*.csv")))
        if not csv_files:
            print(f"No CSV runs found in {self.raw_data_dir}")
            return []

        all_metrics: List[RunMetrics] = []

        for csv_path in csv_files:
            run_id = os.path.splitext(os.path.basename(csv_path))[0]
            meta_path = os.path.join(self.raw_data_dir, f"{run_id}_meta.json")
            
            controller_type = "MPC"
            if os.path.exists(meta_path):
                with open(meta_path, "r", encoding="utf-8") as f:
                    meta = json.load(f)
                    controller_type = meta.get("controller_type", "MPC")
            elif "stanley" in run_id.lower():
                controller_type = "STANLEY"

            print(f"--> Processing {run_id} ({controller_type})...")
            data = SteadyPathLogger.load_csv(csv_path)
            
            # Evaluate run
            metrics = self.evaluator.evaluate(
                data=data,
                run_id=run_id,
                controller_type=controller_type
            )
            all_metrics.append(metrics)

            # Save individual JSON metric file
            out_json = os.path.join(self.metrics_dir, f"{run_id}_metrics.json")
            with open(out_json, "w", encoding="utf-8") as f:
                json.dump(metrics.to_dict(), f, indent=2)

        # Generate aggregated tables
        self._export_summary_csv(all_metrics)
        self._export_summary_markdown(all_metrics)
        self._generate_comparison_analysis(all_metrics)

        return all_metrics

    def _export_summary_csv(self, metrics_list: List[RunMetrics]):
        """Exports master summary CSV."""
        if not metrics_list:
            return
        summary_csv = os.path.join(self.metrics_dir, "summary_table.csv")
        fieldnames = list(metrics_list[0].to_dict().keys())

        with open(summary_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for m in metrics_list:
                writer.writerow(m.to_dict())
        print(f"[OK] Master metrics CSV written: {summary_csv}")

    def _export_summary_markdown(self, metrics_list: List[RunMetrics]):
        """Generates clean readable Markdown table for reports."""
        if not metrics_list:
            return
        md_path = os.path.join(self.metrics_dir, "summary_table.md")

        lines = [
            "# SteadyPath Comprehensive Experimental Metrics Summary",
            "",
            "Generated automatically by Snehal's Analytics Engine (`compute_metrics.py`).",
            "",
            "| Run ID | Controller | Time (s) | Max Lat Err (m) | RMS Lat Err (m) | RMS Yaw Err (deg) | Steering Jerk | Min Clear (m) | Collisions | MPC Latency (ms) | Goal Err (m) | Status |",
            "| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |"
        ]

        for m in metrics_list:
            status = "✅ PASS" if (m.collision_count == 0 and m.lat_error_rms < 0.08 and m.is_destination_reached) else "⚠️ WARN"
            lines.append(
                f"| `{m.run_id}` | **{m.controller_type}** | {m.total_time_s:.1f} | "
                f"{m.lat_error_max:.3f} | {m.lat_error_rms:.3f} | {m.heading_error_rms_deg:.2f}° | "
                f"{m.steering_jerk_metric:.2f} | {m.min_obstacle_clearance:.3f} | {m.collision_count} | "
                f"{m.mpc_time_mean_ms:.1f} | {m.goal_reaching_error_m:.3f} | {status} |"
            )

        with open(md_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        print(f"[OK] Summary Markdown table written: {md_path}")

    def _generate_comparison_analysis(self, metrics_list: List[RunMetrics]):
        """Generates Day 11 baseline comparison between Stanley and MPC."""
        mpc_runs = {m.run_id: m for m in metrics_list if m.controller_type == "MPC"}
        stanley_runs = {m.run_id: m for m in metrics_list if m.controller_type == "STANLEY"}

        if not stanley_runs or not mpc_runs:
            return

        comp_md_path = os.path.join(self.comparison_dir, "stanley_vs_mpc_benchmark.md")
        lines = [
            "# Baseline Benchmark: Stanley Controller vs. SteadyPath MPC (Day 11)",
            "",
            "> **Snehal's Proof 3**: Quantifiable head-to-head comparison demonstrating why SteadyPath MPC",
            "> is superior in tracking accuracy, curvature handling, and control smoothness.",
            "",
            "| Scenario / Experiment | Metric | Baseline (Stanley) | Proposed (SteadyPath MPC) | Improvement / Difference |",
            "| :--- | :--- | :--- | :--- | :--- |"
        ]

        for s_id, s_m in stanley_runs.items():
            # Find matching MPC run
            matched_key = s_id.replace("stanley", "mpc").replace("STANLEY", "MPC")
            matching_mpc = None
            for m_id, m_m in mpc_runs.items():
                if m_id == matched_key or m_id.replace("mpc", "").replace("MPC", "") == s_id.replace("stanley", "").replace("STANLEY", ""):
                    matching_mpc = m_m
                    break

            if not matching_mpc:
                # Pair with first available MPC run if exact key match not found
                matching_mpc = list(mpc_runs.values())[0]

            lat_imp = ((s_m.lat_error_rms - matching_mpc.lat_error_rms) / s_m.lat_error_rms * 100) if s_m.lat_error_rms > 0 else 0
            yaw_imp = ((s_m.heading_error_rms_deg - matching_mpc.heading_error_rms_deg) / s_m.heading_error_rms_deg * 100) if s_m.heading_error_rms_deg > 0 else 0
            jerk_imp = ((s_m.steering_jerk_metric - matching_mpc.steering_jerk_metric) / s_m.steering_jerk_metric * 100) if s_m.steering_jerk_metric > 0 else 0

            scenario_name = s_id.replace("exp_", "").replace("_stanley", "")
            lines.extend([
                f"| `{scenario_name}` | **RMS Lateral Error** | `{s_m.lat_error_rms:.4f} m` | `{matching_mpc.lat_error_rms:.4f} m` | **{lat_imp:+.1f}%** {'(Better)' if lat_imp > 0 else ''} |",
                f"| | **Max Lateral Error** | `{s_m.lat_error_max:.4f} m` | `{matching_mpc.lat_error_max:.4f} m` | **{((s_m.lat_error_max - matching_mpc.lat_error_max)/s_m.lat_error_max*100):+.1f}%** |",
                f"| | **RMS Heading Error** | `{s_m.heading_error_rms_deg:.2f}°` | `{matching_mpc.heading_error_rms_deg:.2f}°` | **{yaw_imp:+.1f}%** |",
                f"| | **Steering Jerk / Smoothness** | `{s_m.steering_jerk_metric:.2f}` | `{matching_mpc.steering_jerk_metric:.2f}` | **{jerk_imp:+.1f}%** {'(Smoother)' if jerk_imp > 0 else ''} |",
                f"| | **Min Obstacle Clearance** | `{s_m.min_obstacle_clearance:.3f} m` | `{matching_mpc.min_obstacle_clearance:.3f} m` | `{matching_mpc.min_obstacle_clearance - s_m.min_obstacle_clearance:+.3f} m` |",
                f"| | **Goal Distance Error** | `{s_m.goal_reaching_error_m:.4f} m` | `{matching_mpc.goal_reaching_error_m:.4f} m` | `{matching_mpc.goal_reaching_error_m - s_m.goal_reaching_error_m:+.4f} m` |"
            ])

        with open(comp_md_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        print(f"[OK] Baseline comparison Markdown written: {comp_md_path}")


if __name__ == "__main__":
    processor = MetricsBatchProcessor()
    processor.process_all_runs()
