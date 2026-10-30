"""Pure-pursuit path tracking with a speed profile."""

from __future__ import annotations

import math

from awake_sim.control.racing_line import RacingLine, speed_at
from awake_sim.sim.vehicle import Actuation


class PurePursuitController:
    def __init__(
        self,
        line: RacingLine,
        wheelbase_m: float,
        lookahead_m: float = 0.55,
        max_steering: float = 0.55,
        max_speed: float = 5.5,
        speed_scale: float = 0.85,
    ) -> None:
        self.line = line
        self.wheelbase = wheelbase_m
        self.lookahead = lookahead_m
        self.max_steering = max_steering
        self.max_speed = max_speed
        self.speed_scale = speed_scale

    def reset(self) -> None:
        pass

    def update(self, x: float, y: float, yaw: float, speed: float) -> Actuation:
        target, index = self.line.target_ahead(x, y, self.lookahead)
        dx = float(target[0] - x)
        dy = float(target[1] - y)
        alpha = math.atan2(dy, dx) - yaw
        alpha = (alpha + math.pi) % (2.0 * math.pi) - math.pi
        distance = max(0.15, math.hypot(dx, dy))
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), distance)
        steering = max(-self.max_steering, min(self.max_steering, steering))

        target_speed = speed_at(self.line, index) * self.speed_scale
        # Small proportional speed correction; wheel velocity control handles most.
        throttle = target_speed / self.max_speed
        correction = 0.08 * (target_speed - speed)
        throttle = max(0.0, min(1.0, throttle + correction))
        if target_speed < 0.4:
            throttle = 0.0
        return Actuation(throttle=throttle, steering=steering)
