from __future__ import annotations
from typing import Optional, Tuple

Vec2 = Tuple[int, int]


class Robot:
    """
    Grid robot:
    - position: (row, col)
    - velocity: (vx, vy) in grid-cells/s for the last command tick; else (0,0)
    - holding: one box char '1'..'4' or None
    """

    def __init__(self, start: Vec2):
        self.position: Vec2 = start
        self.velocity: Tuple[float, float] = (0.0, 0.0)
        self.holding: Optional[str] = None

    def set_velocity_once(self, dv: Vec2, step_dt: float = 0.1) -> None:
        """Set a transient velocity vector for one publish tick."""
        # 1 cell per command over dt seconds → simple velocity estimate
        self.velocity = (dv[0] / step_dt, dv[1] / step_dt)

    def clear_velocity(self) -> None:
        self.velocity = (0.0, 0.0)
