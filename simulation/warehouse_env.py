"""
warehouse_env.py
================
SteadyPath 3D Warehouse Simulation Environment (B. Sheshank's Module)

Provides:
- High-fidelity kinematic vehicle simulation (50 Hz / 20 Hz)
- Dynamic moving obstacles (crossing forklifts, AGVs)
- Static storage racks, pillars, and boundary walls
- Ground-truth state and sensor clearance broadcasts
- Event dispatching (collisions, corridor blockages, destination reached)
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field


@dataclass
class ObstacleState:
    id: str
    x: float
    y: float
    radius: float
    is_moving: bool = False
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class SimSensorSnapshot:
    step: int
    timestamp: float
    vehicle_x: float
    vehicle_y: float
    vehicle_yaw: float
    vehicle_v: float
    min_clearance: float
    nearest_obstacle_id: str
    collision_detected: bool
    collision_count: int
    moving_obstacles: List[Dict[str, float]]
    static_obstacles: List[Dict[str, float]]


class WarehouseEnvironment:
    """Warehouse simulation physics and obstacle environment."""

    def __init__(
        self,
        dt: float = 0.05,
        wheelbase: float = 1.2,
        vehicle_radius: float = 0.35
    ):
        self.dt = dt
        self.L = wheelbase
        self.veh_radius = vehicle_radius

        # Internal state [x, y, psi, v]
        self.state = np.zeros(4)
        self.time = 0.0
        self.step_count = 0

        # Obstacles
        self.obstacles: List[ObstacleState] = []
        self.collision_count = 0
        self.total_collisions = 0

        # Physical limits
        self.steer_max = math.radians(35.0)
        self.v_max = 2.8

    def reset(self, initial_state: Optional[np.ndarray] = None):
        """Resets simulation to initial state."""
        if initial_state is not None:
            self.state = initial_state.copy()
        else:
            self.state = np.array([0.0, 0.0, 0.465, 0.0])
        self.time = 0.0
        self.step_count = 0
        self.total_collisions = 0
        self.obstacles = []

    def add_static_obstacle(self, x: float, y: float, radius: float = 0.45, obs_id: str = "static_rack"):
        """Adds a stationary rack or pillar."""
        self.obstacles.append(ObstacleState(
            id=obs_id,
            x=x,
            y=y,
            radius=radius,
            is_moving=False
        ))

    def add_moving_obstacle(
        self,
        start_x: float,
        start_y: float,
        vx: float = 0.0,
        vy: float = 0.35,
        radius: float = 0.45,
        obs_id: str = "moving_forklift"
    ):
        """Adds a dynamic moving obstacle (forklift/cart)."""
        self.obstacles.append(ObstacleState(
            id=obs_id,
            x=start_x,
            y=start_y,
            radius=radius,
            is_moving=True,
            vx=vx,
            vy=vy
        ))

    def step(self, accel: float, steer: float) -> SimSensorSnapshot:
        """
        Advances the simulation by one timestep dt.
        Applies control [accel, steer] and integrates kinematics.
        """
        dt = self.dt
        x, y, psi, v = self.state

        # Saturated steering
        steer_clamped = max(-self.steer_max, min(self.steer_max, steer))

        # Kinematic bicycle model update
        x_next = x + v * math.cos(psi) * dt
        y_next = y + v * math.sin(psi) * dt
        psi_next = psi + (v / self.L) * math.tan(steer_clamped) * dt
        psi_next = math.atan2(math.sin(psi_next), math.cos(psi_next))
        v_next = max(0.0, min(self.v_max, v + accel * dt))

        self.state = np.array([x_next, y_next, psi_next, v_next])
        self.time += dt
        self.step_count += 1

        # Advance moving obstacles
        for obs in self.obstacles:
            if obs.is_moving:
                obs.x += obs.vx * dt
                obs.y += obs.vy * dt

        # Collision detection & clearance calculations
        min_clear = 999.0
        nearest_id = "none"
        collision_now = False

        static_list = []
        moving_list = []

        for obs in self.obstacles:
            d = math.hypot(x_next - obs.x, y_next - obs.y) - (obs.radius + self.veh_radius)
            if d < min_clear:
                min_clear = d
                nearest_id = obs.id

            obs_info = {"id": obs.id, "x": obs.x, "y": obs.y, "radius": obs.radius}
            if obs.is_moving:
                obs_info.update({"vx": obs.vx, "vy": obs.vy})
                moving_list.append(obs_info)
            else:
                static_list.append(obs_info)

        min_clear = max(0.0, min_clear)
        if min_clear <= 0.0:
            collision_now = True
            self.total_collisions += 1

        return SimSensorSnapshot(
            step=self.step_count,
            timestamp=round(self.time, 4),
            vehicle_x=round(x_next, 4),
            vehicle_y=round(y_next, 4),
            vehicle_yaw=round(psi_next, 4),
            vehicle_v=round(v_next, 4),
            min_clearance=round(min_clear, 4),
            nearest_obstacle_id=nearest_id,
            collision_detected=collision_now,
            collision_count=self.total_collisions,
            moving_obstacles=moving_list,
            static_obstacles=static_list
        )
