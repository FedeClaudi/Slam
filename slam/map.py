from typing import List, Dict, Tuple, Optional
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from copy import deepcopy
from dataclasses import dataclass
from scipy.stats import norm
from loguru import logger

from myterial import black, red_dark, blue_dark, white
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
                1,
                color=blue_dark if self.mean < 0 else red_dark,
                alpha=0.2 if self.mean > 0 else 1,
                zorder=-1 if self.mean > 0 else 1,
            )
        )


@dataclass
class GridPoint:
    x: float
    y: float
    value: float = 0.1
    confidence_threshold: float = 1.5  # only map points with confidence > this are trusted

    @property
    def confidence(self) -> int:
        """
            Returns the confidence about it
            being an open spot (-1 for obstacle, 1 for certainly open otherwise 0)
        """
        if self.value < 0:
            return -1
        elif self.value < self.confidence_threshold:
            return 0
        else:
            return 1


class Map:
    """ Stores two types of information:
            events: pd.DataFrame with speed/angular velocity at each frame and the distance of 
                each ray detected event
            map_gaussians_events: dict. A dictionary storying at each explored location in space a gaussian
                which represents the belief that the point is either free or occupied.
    """

    free_gaussian_value: float = 1
    occupied_gaussian_value: float = -1

    free_gaussian_radius: float = 1
    occupied_gaussian_radius: float = 1

    def __init__(self, agent):
        self.agent = agent

        self.events: Dict[str, List[float]] = {
            **{"speed": [], "omega": []},
            **{str(ray.angle_shift): [] for ray in agent.rays},
        }

        self.map_gaussians_events: Dict[int, List[Gaussian]] = dict()
        self.map_gaussians: Dict[Tuple[int, int], Gaussian] = dict()
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
                            self.free_gaussian_value,
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
                            self.free_gaussian_value,
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
                        self.occupied_gaussian_value,
                        self.occupied_gaussian_radius,
                        detection_distance,
                        ray.angle_shift,
                    )
                )

        self.map_gaussians_events[self.time] = timestep_gaussians
        self.time += 1

    def get_agent_trajectory(self) -> Tuple[list, list, list]:
        """
            Reconstructs the agent's trajectory from the first recorded time step
        """
        data = pd.DataFrame(self.events)

        x, y, theta = [0], [0], [0]
        for i, step in data.iterrows():
            thet_rad = np.radians(theta[-1])
            x.append(x[-1] + step.speed * np.cos(thet_rad))
            y.append(y[-1] + step.speed * np.sin(thet_rad))
            theta.append(theta[-1] + step.omega)
        self.agent_trajectory = dict(x=x, y=y, theta=theta)

        return x, y, theta

    def get_map_gaussians(self):
        """
            Reconstructs the location of the gaussian distributions annotations
        """
        x = self.agent_trajectory["x"]
        y = self.agent_trajectory["y"]
        theta = self.agent_trajectory["theta"]

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
                if gauss.mean > 0:
                    gauss.point.x, gauss.point.y = (
                        int(gauss.point.x),
                        int(gauss.point.y),
                    )
                self.map_gaussians[(gauss.point.x, gauss.point.y)] = gauss

        # empty dictionary
        self.map_gaussians_events: Dict[int, List[Gaussian]] = dict()

    def get_events_location(self):
        """
            Reconstructs the location of laser-object detection events
        """
        x = self.agent_trajectory["x"]
        y = self.agent_trajectory["y"]
        theta = self.agent_trajectory["theta"]

        data = pd.DataFrame(self.events)
        ray_names = [str(ray.angle_shift) for ray in self.agent.rays]

        self.points_x: List[float] = []
        self.points_y: List[float] = []

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

    def get_grid_map(self):
        """
            Creates a 2D grid storing a value at each point, based on the sum
            of nearby gaussians: used for planning.
        """
        self.grid_points: Dict[Tuple[float, float], GridPoint] = dict()

        for gauss in self.map_gaussians.values():
            x, y = round(gauss.point.x, 0), round(gauss.point.y, 0)
            if (x, y) not in self.grid_points.keys():
                self.grid_points[(x, y)] = GridPoint(x, y, value=gauss.mean)

            # get sets of points around the center of the gaussian
            Xs = x + gauss.std * np.cos(np.linspace(0, 2 * np.pi, 8))
            Ys = y + gauss.std * np.sin(np.linspace(0, 2 * np.pi, 8))

            for x, y in zip(Xs, Ys):
                x, y = round(x, 0), round(y, 0)
                if (x, y) not in self.grid_points.keys():
                    self.grid_points[(x, y)] = GridPoint(
                        x, y, value=norm.pdf(gauss.std, 0, gauss.std)
                    )

                if self.grid_points[(x, y)].value >= 0:
                    self.grid_points[(x, y)].value += (
                        2 * norm.pdf(gauss.std, 0, gauss.std)
                    ) * np.sign(gauss.mean)

    def build(self):
        """
            Integrates the stored robot motion to reconstruct the position of the dots
        """
        # reconstruct agent position at each time step
        x, y, theta = self.get_agent_trajectory()

        # reconstruct the map gaussians
        self.get_map_gaussians()

        # reconstruct the position of each sensore event
        self.get_events_location()

        # reconstruct grid
        self.get_grid_map()
        logger.debug(
            f"Map built - {len(self.points_y)} event time steps, {len(self.map_gaussians)} annotated gaussians"
        )

    def draw(self, ax: plt.Axes, ax2: Optional[plt.Axes] = None):
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
            alpha=1,
            label="agent trajectory",
        )

        # plot gaussians
        # for gauss in self.map_gaussians.values():
        #     gauss.draw(ax)

        # plot points grid
        x = [k[0] for k in self.grid_points.keys()]
        y = [k[1] for k in self.grid_points.keys()]
        val = [v.confidence for v in self.grid_points.values()]
        ax.scatter(
            x,
            y,
            c=val,
            cmap="bwr",
            vmin=-1,
            vmax=1,
            lw=0.5,
            ec="k",
            s=20,
            alpha=0.7,
        )

        # plot points for legend
        ax.scatter(0, 0, color=red_dark, s=20, zorder=-100, label="accesible")
        ax.scatter(
            0, 0, color=blue_dark, s=20, zorder=-100, label="inaccessible"
        )
        ax.scatter(0, 0, color=white, s=20, zorder=-100, label="uncertain")

        ax.legend()
        ax.axis("off")
