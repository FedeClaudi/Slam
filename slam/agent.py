import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np


from kino.geometry import Vector
from myterial import blue_dark, pink

from slam.environment import Environment
from slam.map import Map
from slam.ray import Ray


class Agent:
    width: int = 3
    height: int = 4
    head_width: float = 1.5
    color: str = blue_dark
    head_color: str = pink
    speed: float = 0.5

    collision_distance = 6

    def __init__(
        self,
        environment: Environment,
        x: float = 0,
        y: float = 0,
        angle: float = 0,
    ):
        self.environment = environment
        self.map = Map(self)

        self.x: float = x
        self.y: float = y
        self.angle: float = angle

        self.trajectory = dict(x=[x], y=[y])

        # make rays
        self.rays = [Ray(self, angle, 12) for angle in (-40, -20, 0, 20, 40)]

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
        touching_distance = self.collision_distance * 2
        touching = [False for n in range(len(self.rays))]
        for n, ray in enumerate(self.rays):
            if ray.contact_point is not None:
                if ray.contact_point.distance < self.collision_distance:
                    touching[n] = True
                    touching_distance = min(
                        touching_distance, ray.contact_point.distance
                    )

        # turn based on which ray is touching
        speed = self.speed
        if touching[0] and not touching[-1]:
            self.angle += np.random.uniform(0, 45)
        elif not touching[0] and touching[-1]:
            self.angle += np.random.uniform(-45, 0)
        elif np.any(touching):
            self.angle += np.random.uniform(-45, 45)
            speed = self.speed * (touching_distance / self.collision_distance)

        self.x += speed * np.cos(np.radians(self.angle))
        self.y += speed * np.sin(np.radians(self.angle))
        self.trajectory["x"].append(self.x)
        self.trajectory["y"].append(self.y)

    def update(self):
        # move
        self.move()

        # update rays
        for ray in self.rays:
            ray.scan(self.environment.obstacles)

        # update map
        self.map.add(
            *[
                ray.contact_point
                for ray in self.rays
                if ray.contact_point is not None
            ]
        )

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
