import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np
from typing import List, Dict, Tuple

from kino.geometry import Vector
from kino.geometry.point import Point
from myterial import blue_dark, pink, salmon_dark, blue_light

from slam.geometry import Line, distance
from slam.environment import Environment
from slam.obstacle import Obstacle


class Agent:
    width: int = 3
    height: int = 4
    head_width: float = 1.5
    color: str = blue_dark
    head_color: str = pink
    speed: float = 0.5
    collision_distance = 5

    def __init__(
        self,
        environment: Environment,
        x: float = 0,
        y: float = 0,
        angle: float = 0,
    ):
        self.environment = environment

        self.x: float = x
        self.y: float = y
        self.angle: float = angle

        self.trajectory = dict(x=[x], y=[y])

        # make rays
        self.rays = [Ray(self, angle, 20) for angle in (-45, 0, 45)]

        # update rays
        for ray in self.rays:
            ray.scan(self.environment.obstacles)

    def status(self):
        # print state
        print(
            f"""
Agent - 
    x: {self.x:.2f} cm
    y: {self.y:.2f} cm
    angle: {self.angle:.2f} deg
        """
        )

    def set(self, **kwargs):
        for k, val in kwargs.items():
            if k in self.__dict__.keys():
                setattr(self, k, val)
            else:
                raise ValueError(f'Cannot set value for "{k}"')

    @property
    def COM(self) -> Vector:
        return Vector(self.x, self.y)

    @property
    def head_position(self) -> np.ndarray:
        head_shift = Vector(self.height / 2, 0).rotate(self.angle)
        return (self.COM + head_shift).as_array()

    def move(self):
        """
            Moves the agent
        """
        # check if we are within collision distance for any ray
        turn = False
        for ray in self.rays:
            if ray.contact_point is not None:
                if ray.contact_distance < self.collision_distance:
                    turn = True

        if turn:
            self.angle += np.random.uniform(-45, 45)

        self.x += self.speed * np.cos(np.radians(self.angle))
        self.y += self.speed * np.sin(np.radians(self.angle))
        self.trajectory["x"].append(self.x)
        self.trajectory["y"].append(self.y)

    def update(self):
        # move
        self.move()

        # update rays
        for ray in self.rays:
            ray.scan(self.environment.obstacles)

    def draw(self, ax: plt.Axes):
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

        # add rays
        for ray in self.rays:
            ray.draw(ax)

        # draw trace
        ax.plot(
            self.trajectory["x"],
            self.trajectory["y"],
            lw=3,
            ls=":",
            color="k",
            zorder=-1,
        )


class Ray:
    """
        A lidar ray.
    """

    def __init__(
        self,
        agent: Agent,
        angle: float,  # relative to agent orientation
        length: int,
    ):
        self.agent = agent
        self.angle_shift = angle
        self.length = length

    @property
    def angle(self) -> float:
        return self.agent.angle + self.angle_shift

    @property
    def p0(self) -> Point:
        """ position of origin of ray """
        return Point(*self.agent.head_position)

    @property
    def p1(self) -> Point:
        """
            Position of point at end of ray
        """
        x = self.length * np.cos(np.radians(self.angle)) + self.p0.x
        y = self.length * np.sin(np.radians(self.angle)) + self.p0.y
        return Point(x, y)

    @property
    def line(self) -> Line:
        """  Line going through ray points
        """
        return Line.from_points(self.p0, self.p1, color=salmon_dark)

    def scan(self, obstacles: List[Obstacle]):
        """
            Scans through a list of objects to find intersections
        """
        intersection_points: List[Tuple[Point, float]] = []
        for obj in obstacles:
            # get the interesection between the ray line and
            # each edge-line of the obstacle
            obj_intersections: Dict[str, Tuple[Point, float]] = {}
            for name, line in obj.lines.items():

                intersection = self.line.intersection(line)
                if intersection is None:
                    continue
                # print(name, line, intersection)
                dist = distance(self.p0, intersection)

                if dist <= self.length:
                    obj_intersections[name] = (intersection, dist)

            if len(obj_intersections):
                closest = np.argmin([v[1] for v in obj_intersections.values()])
                intersection_points.append(
                    list(obj_intersections.values())[closest]
                )

        # keep the closest intersection points
        if intersection_points:
            closest = np.argmin([v[1] for v in intersection_points])
            self.contact_point = list(intersection_points)[closest][0]
            self.contact_distance = list(intersection_points)[closest][1]
        else:
            self.contact_point = None
            self.contact_distance = None

    def draw(self, ax: plt.Axes):
        p0 = self.p0
        p1 = self.p1

        ax.plot(
            [p0.x, p1.x],
            [p0.y, p1.y],
            lw=4,
            ls="--",
            color=salmon_dark,
            zorder=99,
        )
        # self.line.draw(ax)

        if self.contact_point is not None:
            ax.scatter(
                self.contact_point.x,
                self.contact_point.y,
                s=60,
                lw=1,
                ec="k",
                color=blue_light,
                zorder=100,
            )
