import numpy as np
from loguru import logger
from typing import List, Tuple, Optional

from kino.geometry import Vector

from slam.planner import Planner


class BehavioralRoutine:
    name: str = "base routine"
    ID: int = 0

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
    ID: int = 1

    distance_threshold: float = 2  # start scanning when close to target

    scan_turn_angles: List[int] = [
        30,
        30,
        -20,
        -20,
        -20,
        -20,
        -20,
        -20,
        30,
        30,
    ]

    def __init__(self, agent, planner: Planner, target_node: dict):
        self.agent = agent
        self.planner = planner
        self.target_node = target_node

        self.name = f'navigate to node at ({target_node["x"]}, {target_node["y"]}) - agent @ ({self.agent_position()})'
        super().__init__()

        self.interrupt: bool = False
        self.at_target: bool = False  # switches to true when at node
        self.scan_frame = -1  # keep track of scan duration

    def agent_position(self) -> Vector:
        return Vector(
            self.agent.map.agent_trajectory["x"][-1],
            self.agent.map.agent_trajectory["y"][-1],
        )

    def check_at_node(self) -> bool:
        """
            Checks if the agent is close to the node
        """
        if not self.at_target:
            target = Vector(self.target_node["x"], self.target_node["y"])
            if (
                self.agent_position() - target
            ).magnitude < self.distance_threshold:
                logger.debug("      reached navigation goal.")
                return True
            else:
                return False
        else:
            return self.at_target

    @property
    def completed(self) -> bool:
        """
            The routine is completed when the planner decides that the agent is at the node
        """
        if self.interrupt:
            logger.debug("      navigation interruped.")
            return True
        elif self.scan_frame >= len(self.scan_turn_angles) - 1:
            return True
        else:
            return False

    def _subroutine_navigate(
        self, touching: List[bool], touching_distance: float
    ) -> Tuple[float, float]:
        """
            Selects motor commands to navigate to the next node along a route to the goal
        """
        # get planned route to goal
        try:
            planned_route: List[dict] = self.planner.plan_route(
                self.agent, self.target_node
            )
        except:
            self.interrupt = True
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
                logger.debug(
                    f"      started scanning at {self.agent_position()}."
                )
                self.at_target = True
                return 0, 0
            vect = Vector(
                next_node["x"] - current_node["x"],
                next_node["y"] - current_node["y"],
            )
            steer_angle -= theta[-1] - vect.angle2
        steer_angle /= 3
        steer_angle += np.random.uniform(-5, 5)

        # get motor commands
        if touching_distance < self.agent.collision_distance:
            # too close to collision, interrupt routine
            self.interrupt = True
            return 0, 0
        else:
            return 1, steer_angle

    def _subroutine_scan(self) -> Tuple[float, float]:
        """
            Makes the agent turn in place to scan an area
        """
        return 0, self.scan_turn_angles[self.scan_frame]

    def get_commands(
        self, agent, touching: List[bool], touching_distance: float
    ) -> Tuple[float, float]:
        if not self.at_target:
            return self._subroutine_navigate(touching, touching_distance)
        else:
            self.scan_frame += 1
            return self._subroutine_scan()


class Explore(BehavioralRoutine):
    ID: int = 2

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
    ID: int = 3
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
    ID: int = 4
    name = "spin scan"

    def __init__(self):
        super().__init__(n_steps=20)

    def get_commands(
        self, agent, touching: List[bool], touching_distance: float
    ) -> Tuple[float, float]:
        self.steps_count += 1
        return 0, 360 / self.n_steps  # type: ignore
