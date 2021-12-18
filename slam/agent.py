import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np
from typing import List, Tuple
from loguru import logger

from kino.geometry.point import Point
from kino.geometry import Vector
from myterial import blue_dark, pink

from slam.environment import Environment
from slam.map import Map
from slam.ray import Ray
from slam.behavior import (
    BehavioralRoutine,
    Explore,
    Backtrack,
    SpinScan,
    NavigateToNode,
)
from slam.planner import Planner


class Agent:
    # geometry/drawing
    width: int = 3
    height: int = 4
    head_width: float = 1.5
    color: str = blue_dark
    head_color: str = pink

    # movement
    speed: float = 1

    # LIDAR rays
    ray_length: int = 14
    collision_distance: int = 6

    # SLAM
    update_map_every: int = 25  # update map every n timesteps

    def __init__(
        self,
        environment: Environment,
        x: float = 0,
        y: float = 0,
        angle: float = 0,
    ):
        self.environment = environment

        if self.environment.is_point_in_obstacle(Point(x, y)):
            logger.info(
                "Initial Agent point was in an obstacle, picked a random one instead."
            )
            point = self.environment.random_point()
            x, y = point.x, point.y

        self.x: float = x
        self.y: float = y
        self.angle: float = angle

        self.trajectory = dict(x=[x], y=[y])

        # make rays
        self.rays = [
            Ray(self, angle, self.ray_length)
            for angle in (-40, -20, 0, 20, 40)
        ]

        # update rays
        for ray in self.rays:
            ray.scan(self.environment.obstacles)

        # initiliaze map
        self.map = Map(self)
        self._current_routine: BehavioralRoutine = Explore()

        # initialize planner
        self.planner = Planner()

        self.n_time_steps = 0

    # -------------------------------- kinematics -------------------------------- #

    @property
    def COM(self) -> Vector:
        return Vector(self.x, self.y)

    @property
    def head_position(self) -> np.ndarray:
        head_shift = Vector(self.height / 2, 0).rotate(self.angle)
        return (self.COM + head_shift).as_array()

    def set(self, **kwargs):
        for k, val in kwargs.items():
            if k in self.__dict__.keys():
                setattr(self, k, val)
            else:
                raise ValueError(f'Cannot set value for "{k}"')

    # --------------------------------- behavior --------------------------------- #

    def check_touching(self) -> Tuple[List[bool], float]:
        """
            Checks which of the rays are touching an object and returns the 
            distance of the closest objct
        """
        touching_distance: float = self.collision_distance * 2
        touching = [False for n in range(len(self.rays))]
        for n, ray in enumerate(self.rays):
            if ray.contact_point is not None:
                if ray.contact_point.distance < self.collision_distance:
                    touching[n] = True
                    touching_distance = min(
                        touching_distance, ray.contact_point.distance
                    )
        return touching, touching_distance

    def select_routine(self, touching: List[bool], touching_distance: float):
        """
            Selects which routine to execute
        """
        if self._current_routine.name == "exploration":
            if touching_distance < self.speed:
                if touching[0] and touching[-1]:
                    self._current_routine = Backtrack()

            elif np.random.rand() < 0.005 and np.any(touching):
                # do a spin
                self._current_routine = SpinScan()

            elif (
                np.random.rand() < 0.05
                and not np.any(touching)
                and self.n_time_steps > 10
            ):
                # explore an 'uncertain' node in the graph
                self.slam()
                node = self.planner.get_uncertain_node()
                if node is not None:
                    self._current_routine = NavigateToNode(
                        self, self.planner, node
                    )

        else:
            if self._current_routine.completed:
                self._current_routine = Explore()

    def move(self):
        """
            Moves the agent
        """
        # check if we are within collision distance for any ray
        touching, touching_distance = self.check_touching()

        # get movement commands
        self.select_routine(touching, touching_distance)
        speed, steer_angle = self._current_routine.get_commands(
            self, touching, touching_distance
        )

        # store variables and move
        self._current_speed = speed
        self._current_omega = steer_angle

        self.x += speed * np.cos(np.radians(self.angle))
        self.y += speed * np.sin(np.radians(self.angle))
        self.angle += steer_angle

        self.trajectory["x"].append(self.x)
        self.trajectory["y"].append(self.y)

    def update(self):
        # move
        self.move()

        # update rays
        for ray in self.rays:
            ray.scan(self.environment.obstacles)

        # update map entries
        self.map.add(
            *[
                ray.contact_point
                for ray in self.rays
                if ray.contact_point is not None
            ]
        )

        # generate map
        if self.n_time_steps % self.update_map_every == 0:
            self.slam()

        self.n_time_steps += 1

    # ------------------------------- slam/planning ------------------------------ #
    def slam(self):
        """ Builds a map + agent localization and activates the planner
        """
        logger.debug(f"Agent, SLAM at timestep: {self.n_time_steps}")
        self.map.build()
        self.planner.build(list(self.map.grid_points.values()))

    # ----------------------------------- draw ----------------------------------- #

    def draw(self, ax: plt.Axes, just_agent: bool = False):
        """
            Draws the agent as a rectangle with a circle for head
        """
        # draw body, get rectangle corner first
        body_shift = Vector(-self.height / 2, -self.width / 2).rotate(
            self.angle
        )

        ax.add_artist(
            Rectangle(
                (self.COM + body_shift).as_array(),
                self.height,
                self.width,
                self.angle,
                facecolor=self.color,
                lw=1,
                edgecolor="k",
                zorder=100,
            )
        )
        # draw head
        ax.add_artist(
            Circle(
                self.head_position,
                self.head_width,
                facecolor=self.head_color,
                lw=1,
                edgecolor="k",
                zorder=100,
            )
        )

        if not just_agent:
            # add rays
            for ray in self.rays:
                ray.draw(ax)

            # draw trace
            ax.plot(
                self.trajectory["x"],
                self.trajectory["y"],
                lw=0.5,
                color="k",
                zorder=-1,
                alpha=0.5,
            )
