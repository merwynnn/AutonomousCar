"""Template controller: copy this file and replace ``update``.

Nothing here uses the simulator internals, ground truth or a map. It receives
only sensor observations and returns a throttle/steering command.
"""

from __future__ import annotations

from onboard.api import Actuation, Observation


class TemplateController:
    """Safe placeholder that holds the car still."""

    def __init__(self) -> None:
        self.previous_steering = 0.0

    def reset(self) -> None:
        self.previous_steering = 0.0

    def update(self, observation: Observation) -> Actuation:
        # Replace this with your algorithm. Examples:
        #   scan = observation.lidar
        #   ranges = scan.ranges_m if scan is not None else ...
        #   error = ...
        steering = self.previous_steering
        return Actuation(throttle=0.0, steering=steering)
