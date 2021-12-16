import numpy as np
from loguru import logger
from typing import List, Tuple, Optional


class BehavioralRoutine:
    name = "base routine"

    def __init__(self, n_steps: Optional[int] = None):
        self.n_steps = n_steps
        self.steps_count = 0
        logger.debug(f"Initiate behavoioral routine: {self.name}")

    @property
    def completed(self) -> bool:
        if self.n_steps is None:
            return False
        return self.steps_count == self.n_steps


class Explore(BehavioralRoutine):
    name: str = "exploration"

    def __init__(self):
        super().__init__()

    def get_commands(
        self, agent, touching: List[bool], touching_distance: float
    ) -> Tuple[float, float]:
        # turn based on which ray is touching
        speed = agent.speed
        steer_angle = np.random.uniform(-10, 10)
        if touching[0] and not touching[-1]:
            # left ray touching -> turn right
            steer_angle = np.random.uniform(0, 25)
        elif not touching[0] and touching[-1]:
            # right ray touching -> turn left
            steer_angle = np.random.uniform(-25, 0)
        elif np.any(touching):
            # something else touching, turn more
            steer_angle = np.random.uniform(-25, 25)
            speed = agent.speed * (
                touching_distance / agent.collision_distance
            )
        return speed, steer_angle


class Backtrack(BehavioralRoutine):
    name = "back track"

    def __init__(self):
        super().__init__(n_steps=3)

    def get_commands(
        self, agent, touching: List[bool], touching_distance: float
    ) -> Tuple[float, float]:
        if self.steps_count < self.n_steps - 1:  # type: ignore
            speed = -agent.speed
            steer_angle = 0
        else:
            speed = 0
            steer_angle = np.random.uniform(120, 240)
        self.steps_count += 1
        return speed, steer_angle


class SpinScan(BehavioralRoutine):
    name = "spin scan"

    def __init__(self):
        super().__init__(n_steps=20)

    def get_commands(
        self, agent, touching: List[bool], touching_distance: float
    ) -> Tuple[float, float]:
        self.steps_count += 1
        return 0, 360 / self.n_steps  # type: ignore
