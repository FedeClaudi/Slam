import numpy as np
from loguru import logger
from typing import List, Tuple, Optional

from kino.geometry import Vector

from slam.planner import Planner


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


class NavigateToNode(BehavioralRoutine):
    distance_threshold: float = 1

    def __init__(self, agent, planner: Planner, target_node: dict):
        self.agent = agent
        self.planner = planner
        self.target_node = target_node

        self.name = f'navigate to node at ({target_node["x"]}, {target_node["y"]}) - agent @ ({self.agent_position()})'
        super().__init__()

        self.point_accessible: bool = True

    def agent_position(self) -> Vector:
        return Vector(
            self.agent.map.agent_trajectory["x"][-1],
            self.agent.map.agent_trajectory["y"][-1],
        )

    @property
    def completed(self) -> bool:
        """
            The routine is completed when the planner decides that the agent is at the node
        """
        if not self.point_accessible:
            logger.debug("      reached navigation goal.")
            return True

        target = Vector(self.target_node["x"], self.target_node["y"])
        if (
            self.agent_position() - target
        ).magnitude < self.distance_threshold:
            logger.debug("      reached navigation goal.")
            return True
        else:
            return False

    def get_commands(
        self, agent, touching: List[bool], touching_distance: float
    ) -> Tuple[float, float]:

        # get planned route to goal
        try:
            planned_route: List[dict] = self.planner.plan_route(
                agent, self.target_node
            )
        except:
            self.point_accessible = False
            return 0, 0

        _, _, theta = self.agent.map.get_agent_trajectory()

        # get angle between agent orientation and next node
        # by taking the avereage of the next N steps along the route
        steer_angle: float = 0
        for next_idx in (1, 2, 3):
            try:
                current_node, next_node = planned_route[0], planned_route[3]
            except IndexError:
                # reached the end
                self.point_accessible = False
                return 0, 0
            vect = Vector(
                next_node["x"] - current_node["x"],
                next_node["y"] - current_node["y"],
            )
            steer_angle -= theta[-1] - vect.angle2
        steer_angle /= 3
        steer_angle += np.random.uniform(-5, 5)

        # get motor commands
        if not np.all(touching):
            return 1, steer_angle
        elif touching_distance < self.agent.collision_distance:
            self.point_accessible = False
            return 0, 0
        else:
            return 1, steer_angle


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
