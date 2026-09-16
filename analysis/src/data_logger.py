"""
data_logger.py
================
SteadyPath Telemetry Data Logger (Snehal's Module - Day 2 & Day 3)

Provides standardized, high-performance streaming logging for:
1. Sheshank's 3D Warehouse Simulation (actual states, obstacles, events)
2. MPC Controller (applied controls, predicted horizons, computation times)
3. R.P. Singh's Global Planner (reference trajectories, replan triggers, destination)

Supports dual export:
- CSV: High-speed flat tabular format for instant pandas/Excel analysis
- JSONL: Complete hierarchical format capturing full MPC horizon arrays and event logs
"""

import os
import csv
import json
import time
from typing import Dict, List, Optional, Any, Union
from dataclasses import dataclass, field, asdict


# Day 2 Standard CSV Column Headers
STANDARD_CSV_COLUMNS = [
    "timestamp",
    "step",
    # Actual vehicle state (Sheshank)
    "x",
    "y",
    "yaw",
    "v",
    "omega",
    # Reference target state (R.P. Singh)
    "x_ref",
    "y_ref",
    "yaw_ref",
    "v_ref",
    "curvature_ref",
    # Tracking errors
    "lateral_error",
    "heading_error",
    "velocity_error",
    "longitudinal_error",
    # Controls applied (MPC / Controller Lead)
    "accel",
    "steering_angle",
    "jerk_accel",
    "steering_rate",
    # MPC Diagnostics
    "mpc_comp_time_ms",
    "mpc_status",
    # Safety & Obstacles
    "min_clearance",
    "nearest_obstacle_id",
    "collision_detected",
    "collision_count",
    # Planner & Destination
    "active_destination_id",
    "goal_x",
    "goal_y",
    "replan_count",
    "replan_in_progress",
    "route_completion_ratio"
]


@dataclass
class TelemetryRecord:
    """Represents a single timestep snapshot of the full system."""
    timestamp: float
    step: int
    
    # Actual vehicle state
    x: float
    y: float
    yaw: float
    v: float
    omega: float = 0.0
    
    # Reference target state
    x_ref: float = 0.0
    y_ref: float = 0.0
    yaw_ref: float = 0.0
    v_ref: float = 0.0
    curvature_ref: float = 0.0
    
    # Errors (can be precalculated or computed by logger)
    lateral_error: float = 0.0
    heading_error: float = 0.0
    velocity_error: float = 0.0
    longitudinal_error: float = 0.0
    
    # Controls applied
    accel: float = 0.0
    steering_angle: float = 0.0
    jerk_accel: float = 0.0
    steering_rate: float = 0.0
    
    # MPC solver info
    mpc_comp_time_ms: float = 0.0
    mpc_status: int = 1  # 1 = optimal/success, 0 = warning, -1 = failure
    predicted_trajectory: List[List[float]] = field(default_factory=list)  # [[x, y, yaw, v], ...]
    
    # Safety & obstacles
    min_clearance: float = 999.0
    nearest_obstacle_id: str = "none"
    collision_detected: bool = False
    collision_count: int = 0
    
    # Planner metadata
    active_destination_id: str = "DEST_DEFAULT"
    goal_x: float = 0.0
    goal_y: float = 0.0
    replan_count: int = 0
    replan_in_progress: bool = False
    route_completion_ratio: float = 0.0

    def to_csv_row(self) -> Dict[str, Any]:
        """Convert to flat dictionary matching STANDARD_CSV_COLUMNS."""
        return {
            "timestamp": round(self.timestamp, 4),
            "step": self.step,
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "yaw": round(self.yaw, 4),
            "v": round(self.v, 4),
            "omega": round(self.omega, 4),
            "x_ref": round(self.x_ref, 4),
            "y_ref": round(self.y_ref, 4),
            "yaw_ref": round(self.yaw_ref, 4),
            "v_ref": round(self.v_ref, 4),
            "curvature_ref": round(self.curvature_ref, 4),
            "lateral_error": round(self.lateral_error, 4),
            "heading_error": round(self.heading_error, 4),
            "velocity_error": round(self.velocity_error, 4),
            "longitudinal_error": round(self.longitudinal_error, 4),
            "accel": round(self.accel, 4),
            "steering_angle": round(self.steering_angle, 4),
            "jerk_accel": round(self.jerk_accel, 4),
            "steering_rate": round(self.steering_rate, 4),
            "mpc_comp_time_ms": round(self.mpc_comp_time_ms, 3),
            "mpc_status": self.mpc_status,
            "min_clearance": round(self.min_clearance, 4),
            "nearest_obstacle_id": self.nearest_obstacle_id,
            "collision_detected": int(self.collision_detected),
            "collision_count": self.collision_count,
            "active_destination_id": self.active_destination_id,
            "goal_x": round(self.goal_x, 4),
            "goal_y": round(self.goal_y, 4),
            "replan_count": self.replan_count,
            "replan_in_progress": int(self.replan_in_progress),
            "route_completion_ratio": round(self.route_completion_ratio, 4),
        }

    def to_json_dict(self) -> Dict[str, Any]:
        """Convert to hierarchical dictionary including predicted trajectory."""
        d = asdict(self)
        d["timestamp"] = round(self.timestamp, 4)
        return d


class SteadyPathLogger:
    """
    Core logging facility for SteadyPath simulation runs.
    
    Usage:
        logger = SteadyPathLogger(
            run_id="exp01_normal_nav_dest1",
            controller_type="MPC",
            output_dir="analysis/raw_data"
        )
        logger.start_run(metadata={"map": "warehouse_main", "speed_limit": 2.0})
        
        # Inside simulation loop:
        logger.log_step(record)
        
        # When special events happen:
        logger.log_event("REPLAN_TRIGGERED", {"reason": "forklift_blockage", "at_x": 12.0})
        
        logger.end_run()
    """

    def __init__(
        self,
        run_id: str,
        controller_type: str = "MPC",  # "MPC" or "STANLEY"
        output_dir: str = "analysis/raw_data",
        export_csv: bool = True,
        export_jsonl: bool = True,
        buffer_size: int = 50
    ):
        self.run_id = run_id
        self.controller_type = controller_type.upper()
        self.output_dir = output_dir
        self.export_csv = export_csv
        self.export_jsonl = export_jsonl
        self.buffer_size = buffer_size

        os.makedirs(self.output_dir, exist_ok=True)
        
        self.csv_path = os.path.join(self.output_dir, f"{self.run_id}.csv")
        self.jsonl_path = os.path.join(self.output_dir, f"{self.run_id}.jsonl")
        self.events_path = os.path.join(self.output_dir, f"{self.run_id}_events.jsonl")
        self.metadata_path = os.path.join(self.output_dir, f"{self.run_id}_meta.json")

        self._csv_file = None
        self._csv_writer = None
        self._jsonl_file = None
        self._events_file = None

        self._record_count = 0
        self._is_active = False
        self._metadata: Dict[str, Any] = {}
        self._last_accel = 0.0
        self._last_steering = 0.0
        self._last_time = 0.0

    def start_run(self, metadata: Optional[Dict[str, Any]] = None):
        """Initializes output files and writes header info."""
        self._metadata = metadata or {}
        self._metadata.update({
            "run_id": self.run_id,
            "controller_type": self.controller_type,
            "start_time_iso": time.strftime("%Y-%m-%d %H:%M:%S"),
            "columns": STANDARD_CSV_COLUMNS
        })

        # Save metadata
        with open(self.metadata_path, "w", encoding="utf-8") as f:
            json.dump(self._metadata, f, indent=2)

        # Open CSV writer
        if self.export_csv:
            self._csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=STANDARD_CSV_COLUMNS)
            self._csv_writer.writeheader()

        # Open JSONL writers
        if self.export_jsonl:
            self._jsonl_file = open(self.jsonl_path, "w", encoding="utf-8")

        self._events_file = open(self.events_path, "w", encoding="utf-8")
        
        self._is_active = True
        self._record_count = 0
        self.log_event("RUN_STARTED", {"controller": self.controller_type, "run_id": self.run_id})

    def log_step(self, record: TelemetryRecord):
        """Log a single simulation step record."""
        if not self._is_active:
            raise RuntimeError("Logger is not active. Call start_run() first.")

        # Automatically compute jerk and steering rate if not set
        dt = record.timestamp - self._last_time if self._record_count > 0 else 0.0
        if dt > 1e-4:
            if record.jerk_accel == 0.0:
                record.jerk_accel = (record.accel - self._last_accel) / dt
            if record.steering_rate == 0.0:
                record.steering_rate = (record.steering_angle - self._last_steering) / dt

        self._last_accel = record.accel
        self._last_steering = record.steering_angle
        self._last_time = record.timestamp

        # Write CSV
        if self._csv_writer:
            self._csv_writer.writerow(record.to_csv_row())

        # Write JSONL
        if self._jsonl_file:
            self._jsonl_file.write(json.dumps(record.to_json_dict()) + "\n")

        self._record_count += 1
        if self._record_count % self.buffer_size == 0:
            self.flush()

    def log_event(self, event_name: str, details: Optional[Dict[str, Any]] = None, timestamp: Optional[float] = None):
        """Log a discrete event (blockage appeared, replan initiated, destination reached)."""
        t = timestamp if timestamp is not None else self._last_time
        event_obj = {
            "timestamp": round(t, 4),
            "step": self._record_count,
            "event": event_name,
            "details": details or {}
        }
        if self._events_file:
            self._events_file.write(json.dumps(event_obj) + "\n")
            self._events_file.flush()

    def flush(self):
        """Flushes buffers to disk."""
        if self._csv_file:
            self._csv_file.flush()
        if self._jsonl_file:
            self._jsonl_file.flush()
        if self._events_file:
            self._events_file.flush()

    def end_run(self, final_status: str = "SUCCESS"):
        """Closes all file handles and logs completion."""
        if not self._is_active:
            return

        self.log_event("RUN_COMPLETED", {"status": final_status, "total_steps": self._record_count})
        self.flush()

        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
        if self._jsonl_file:
            self._jsonl_file.close()
            self._jsonl_file = None
        if self._events_file:
            self._events_file.close()
            self._events_file = None

        self._is_active = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        status = "ERROR" if exc_type else "SUCCESS"
        self.end_run(final_status=status)

    @staticmethod
    def load_csv(csv_path: str) -> Dict[str, List[float]]:
        """
        Helper to read logged CSV file into a dictionary of lists.
        Handles numeric conversions automatically.
        """
        data: Dict[str, List[Any]] = {col: [] for col in STANDARD_CSV_COLUMNS}
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for col in STANDARD_CSV_COLUMNS:
                    val = row.get(col, "")
                    # Convert types
                    if col in ("step", "collision_detected", "collision_count", "replan_count", "replan_in_progress", "mpc_status"):
                        data[col].append(int(val) if val != "" else 0)
                    elif col in ("nearest_obstacle_id", "active_destination_id"):
                        data[col].append(val)
                    else:
                        data[col].append(float(val) if val != "" else 0.0)
        return data

    @staticmethod
    def load_events(events_path: str) -> List[Dict[str, Any]]:
        """Reads logged events JSONL."""
        events = []
        if os.path.exists(events_path):
            with open(events_path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line:
                        events.append(json.loads(line))
        return events
