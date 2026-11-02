"""Template controller: copy this file and replace ``update``.

Nothing here uses the simulator internals, ground truth or a map. It receives
only sensor observations and returns a throttle/steering command.
"""

from __future__ import annotations

import math

from onboard.api import Actuation, Observation

from algorithms.debug import DebugFrame, Readout


class TemplateController:
    """Safe placeholder that holds the car still."""

    def __init__(self) -> None:
        self.previous_steering = 0.0
        self.t = 0.0

    def reset(self) -> None:
        self.previous_steering = 0.0
        self.t = 0.0

    def update(
        self,
        observation: Observation,
        speed_limit: float | None = None,
    ) -> Actuation:
        # Replace this with your algorithm. Examples:
        #   scan = observation.lidar
        #   ranges = scan.ranges_m if scan is not None else ...
        #   error = ...
        self.t = observation.t
        steering = self.previous_steering
        return Actuation(throttle=0.0, steering=steering)

    def debug_frame(self) -> DebugFrame:
        """Optional example: return a small readout for the dashboard."""

        return DebugFrame(
            controller="template",
            title="Template",
            t=self.t,
            status="IDLE",
            readouts=(
                Readout(
                    "steering",
                    "Steering",
                    math.degrees(self.previous_steering),
                    "deg",
                    1,
                ),
            ),
        )
