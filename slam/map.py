from typing import List, Dict
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from copy import deepcopy
from dataclasses import dataclass

from myterial import black, red_dark, blue_dark
from kino.geometry import Vector
from kino.geometry.point import Point

from slam.ray import Contact


@dataclass
class Gaussian:
    mean: float
    std: float
    distance: float
    angle_delta: int

    def draw(self, ax: plt.Axes):
        ax.add_artist(
            plt.Circle(
                self.point.xy,  # type: ignore
                self.std,
                color=blue_dark if self.mean < 0 else red_dark,
                alpha=0.2 if self.mean > 0 else 1,
                zorder=-1 if self.mean > 0 else 1,
            )
        )


class Map:
    """ Stores two types of information:
            events: pd.DataFrame with speed/angular velocity at each frame and the distance of 
                each ray detected event
            map_gaussians_events: dict. A dictionary storying at each explored location in space a gaussian
                which represents the belief that the point is either free or occupied.
    """

    free_gaussian_radius: float = 1.25
    occupied_gaussian_radius: float = 0.5

    def __init__(self, agent):
        self.agent = agent

        self.events: Dict[str, List[float]] = {
            **{"speed": [], "omega": []},
            **{str(ray.angle_shift): [] for ray in agent.rays},
        }

        self.map_gaussians_events: Dict[int, List[Gaussian]] = dict()
        self.time = 0  # to be incremented everytime the map is updated

    def add(self, *events: Contact):
        """
            Given a list of ray-object contact events (in egocentric coordinates)
            stores the distance each ray picked up an object together witht he robot's
            kinematics.

            It also stores the location of Free vs Occupied gaussians.
        """

        self.events["speed"].append(self.agent._current_speed)
        self.events["omega"].append(self.agent._current_omega)

        timestep_gaussians: List[Gaussian] = []
        for ray in self.agent.rays:
            ray_events = [ev for ev in events if ev.ray == ray]
            if not ray_events:
                # empty event
                self.events[str(ray.angle_shift)].append(np.nan)

                # store Free gaussians
                timestep_gaussians.extend(
                    [
                        Gaussian(
                            1,
                            self.free_gaussian_radius,
                            point.distance,
                            ray.angle_shift,
                        )
                        for point in ray.sample()
                    ]
                )
            else:
                # object detection !
                detection_distance = ray_events[0].distance
                self.events[str(ray.angle_shift)].append(detection_distance)

                # store Free gaussians
                free_points = [
                    pt
                    for pt in ray.sample()
                    if pt.distance < detection_distance
                ]
                timestep_gaussians.extend(
                    [
                        Gaussian(
                            1,
                            self.free_gaussian_radius,
                            point.distance,
                            ray.angle_shift,
                        )
                        for point in free_points
                    ]
                )

                # store Occupied gaussian
                timestep_gaussians.append(
                    Gaussian(
                        -1,
                        self.occupied_gaussian_radius,
                        detection_distance,
                        ray.angle_shift,
                    )
                )

        self.map_gaussians_events[self.time] = timestep_gaussians
        self.time += 1

    def build(self):
        """
            Integrates the stored robot motion to reconstruct the position of the dots
        """
        self.points_x: List[float] = []
        self.points_y: List[float] = []

        data = pd.DataFrame(self.events)

        # reconstruct vehicle position at each time step
        ray_names = [str(ray.angle_shift) for ray in self.agent.rays]
        x, y, theta = [0], [0], [0]
        for i, step in data.iterrows():
            thet_rad = np.radians(theta[-1])
            x.append(x[-1] + step.speed * np.cos(thet_rad))
            y.append(y[-1] + step.speed * np.sin(thet_rad))
            theta.append(theta[-1] + step.omega)
        self.agent_trajectory = dict(x=x, y=y, theta=theta)

        # reconstruct the map gaussians
        self.map_gaussians: List[Gaussian] = []
        for time, gaussians in self.map_gaussians_events.items():
            for gauss in gaussians:
                # get position of the head
                head_shift = Vector(self.agent.height / 2, 0).rotate(
                    theta[time + 1]
                )

                # get position of gaussian
                _theta = np.radians(theta[time + 1] + gauss.angle_delta)
                gauss.point = Point(
                    x[time] + head_shift.x + np.cos(_theta) * gauss.distance,
                    y[time] + head_shift.y + np.sin(_theta) * gauss.distance,
                )
                self.map_gaussians.append(gauss)

        # reconstruct the position of each sensore event
        for sensor in ray_names:
            delta = np.radians(float(sensor))
            for i, value in data[sensor].iteritems():
                if not np.isnan(value):
                    # get the position of the robot's body at that moment
                    _x, _y, _theta = (
                        x[i + 1],
                        y[i + 1],
                        np.radians(theta[i + 1]),
                    )

                    # get position of the head
                    head_shift = Vector(self.agent.height / 2, 0).rotate(
                        theta[i + 1]
                    )

                    # get the position of the event
                    self.points_x.append(
                        _x + head_shift.x + np.cos(_theta + delta) * value
                    )
                    self.points_y.append(
                        _y + head_shift.y + np.sin(_theta + delta) * value
                    )

    def draw(self, ax: plt.Axes):
        # build map points
        self.build()

        # plot reconstructed map points
        ax.scatter(
            self.points_x,
            self.points_y,
            zorder=150,
            s=5,
            color=black,
            label="detected obstacles",
        )

        # plot localized agent
        map_agent = deepcopy(self.agent)
        map_agent.set(
            x=self.agent_trajectory["x"][-1],
            y=self.agent_trajectory["y"][-1],
            angle=self.agent_trajectory["theta"][-1],
        )
        map_agent.draw(ax=ax, just_agent=True)
        ax.plot(
            self.agent_trajectory["x"],
            self.agent_trajectory["y"],
            lw=1,
            color="k",
            zorder=100,
            alpha=0.5,
            label="agent trajectory",
        )

        # plot gaussians
        for gauss in self.map_gaussians:
            gauss.draw(ax)

        ax.scatter(0, 0, color=red_dark, s=5, label="mapped places")
        ax.legend()
        ax.axis("off")
