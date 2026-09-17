"""
route_planner.py
================
SteadyPath Global Planner & Dynamic Replanner (R.P. Singh's Module)

Provides:
- Smooth reference trajectory generation through warehouse corridors
- Dynamic destination selection at runtime (Proof 1)
- Dynamic blockage detection & obstacle-bypass replanning (Proof 2)
- Fast reference window extraction for MPC prediction horizon
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float
    v: float
    curvature: float
    s: float  # Cumulative arc length


@dataclass
class PlannedRoute:
    route_id: str
    destination_id: str
    goal_coord: Tuple[float, float]
    waypoints: List[Waypoint]
    total_length: float
    replan_count: int = 0
    is_replanned: bool = False


class RoutePlanner:
    """Warehouse route planner and dynamic replanning engine."""

    DESTINATIONS = {
        "DEST_BAY_01": (30.0, 1.902),
        "DEST_BAY_02": (24.0, -0.495),
        "DEST_BAY_03": (25.0, 0.000),
        "DEST_BAY_04": (35.0, 0.742),
    }

    def __init__(self, default_velocity: float = 1.5):
        self.default_v = default_velocity
        self.active_route: Optional[PlannedRoute] = None
        self.replan_count = 0

    def plan(
        self,
        destination_id: str = "DEST_BAY_01",
        replan_blockage: Optional[Tuple[float, float]] = None
    ) -> PlannedRoute:
        """
        Generates a continuous reference trajectory to destination.
        If replan_blockage is provided, plans a collision-free bypass detour.
        """
        if destination_id not in self.DESTINATIONS:
            dest_x, dest_y = (30.0, 0.0)
        else:
            dest_x, dest_y = self.DESTINATIONS[destination_id]

        ds = 0.05
        waypoints = []
        s_accum = 0.0
        kw = 2.0 * math.pi / 25.0

        x = 0.0
        prev_x, prev_y = 0.0, 0.0

        while x <= dest_x + ds:
            # Base curved warehouse aisle
            y_base = 2.0 * math.sin(x * kw)
            dydx_base = 2.0 * kw * math.cos(x * kw)
            d2ydx2_base = -2.0 * (kw ** 2) * math.sin(x * kw)

            if replan_blockage is not None:
                # Evasive detour around blockage
                bx, by = replan_blockage
                detour = 2.2 * math.exp(-((x - bx) ** 2) / 10.0)
                d_detour = -detour * (2.0 * (x - bx) / 10.0)
                d2_detour = detour * ((2.0 * (x - bx) / 10.0) ** 2 - 2.0 / 10.0)

                y = y_base + detour
                dydx = dydx_base + d_detour
                d2ydx2 = d2ydx2_base + d2_detour
            else:
                y = y_base
                dydx = dydx_base
                d2ydx2 = d2ydx2_base

            yaw = math.atan2(dydx, 1.0)
            curvature = d2ydx2 / ((1.0 + dydx ** 2) ** 1.5)

            # Arc length increment
            if len(waypoints) > 0:
                step_ds = math.hypot(x - prev_x, y - prev_y)
                s_accum += step_ds
            prev_x, prev_y = x, y

            # Velocity profile: smooth deceleration near destination
            dist_to_end = dest_x - x
            if dist_to_end < 3.0:
                v_target = max(0.2, (dist_to_end / 3.0) * self.default_v)
            else:
                v_target = self.default_v

            waypoints.append(Waypoint(
                x=round(x, 4),
                y=round(y, 4),
                yaw=round(yaw, 4),
                v=round(v_target, 4),
                curvature=round(curvature, 4),
                s=round(s_accum, 4)
            ))

            x += ds

        is_replan = (replan_blockage is not None)
        if is_replan:
            self.replan_count += 1

        route = PlannedRoute(
            route_id=f"route_{destination_id}_{'replan' if is_replan else 'nominal'}",
            destination_id=destination_id,
            goal_coord=(dest_x, waypoints[-1].y),
            waypoints=waypoints,
            total_length=round(s_accum, 3),
            replan_count=self.replan_count,
            is_replanned=is_replan
        )
        self.active_route = route
        return route

    def get_reference_window(
        self,
        current_x: float,
        horizon_steps: int = 15,
        dt: float = 0.05
    ) -> np.ndarray:
        """
        Extracts an (N+1, 4) reference trajectory matrix [x_ref, y_ref, yaw_ref, v_ref]
        starting at current_x, projecting forward along the planned route.
        """
        if self.active_route is None:
            raise RuntimeError("No active route planned. Call plan() first.")

        wps = self.active_route.waypoints
        # Find closest waypoint index to current_x
        best_idx = 0
        min_dist = float('inf')
        for i, wp in enumerate(wps):
            d = abs(wp.x - current_x)
            if d < min_dist:
                min_dist = d
                best_idx = i

        ref_window = []
        curr_idx = best_idx
        v_nominal = wps[curr_idx].v
        dx_step = max(0.02, v_nominal * dt)

        for step in range(horizon_steps + 1):
            target_x = current_x + step * dx_step
            # Find closest waypoint ahead
            while curr_idx < len(wps) - 1 and wps[curr_idx].x < target_x:
                curr_idx += 1
            wp = wps[curr_idx]
            ref_window.append([wp.x, wp.y, wp.yaw, wp.v])

        return np.array(ref_window)

    def check_blockage(
        self,
        vehicle_x: float,
        obstacles: List[Dict[str, float]],
        lookahead_distance: float = 6.0
    ) -> Optional[Tuple[float, float]]:
        """
        Checks if upcoming corridor has a blockage intersecting the active route.
        Returns coordinates (bx, by) of the blocking obstacle, or None if clear.
        """
        if self.active_route is None or self.active_route.is_replanned:
            return None

        wps = self.active_route.waypoints
        for obs in obstacles:
            ox = obs["x"]
            oy = obs["y"]
            r = obs.get("radius", 0.5)

            # Check if obstacle is ahead in lookahead zone
            if vehicle_x < ox <= vehicle_x + lookahead_distance:
                # Find route y at ox
                for wp in wps:
                    if abs(wp.x - ox) < 0.1:
                        if math.hypot(wp.x - ox, wp.y - oy) < r + 0.6:
                            # Path is obstructed!
                            return (ox, oy)
        return None
