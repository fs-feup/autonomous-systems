from __future__ import annotations
from typing import Optional, Tuple

Vec2 = Tuple[int, int]


class Robot:
    """
    Grid robot:
    - position: (row, col)
    - velocity: (vx, vy) in world/map axes (x right, y up) for the
      last command tick; values are unit steps (-1, 0, +1), else (0, 0)
    - holding: one box char '1'..'4' or None
    """

    def __init__(self, start: Vec2):
        self.position: Vec2 = start
        self.velocity: Tuple[float, float] = (0.0, 0.0)
        self.holding: Optional[str] = None

    def set_velocity_once(self, dv: Vec2) -> None:
        """
        Set a transient velocity vector for one publish tick, expressed
        in world/map coordinates (x right, y up) with unit magnitude
        per step.
        """
        self.velocity = (self.velocity[0] + float(dv[0]), self.velocity[1] + float(dv[1]))

    def clear_velocity(self) -> None:
        self.velocity = (0.0, 0.0)
