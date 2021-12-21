from typing import List, Dict, Tuple, Optional
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from copy import deepcopy

from myterial import red_dark, blue_darker, red_light
from kino.geometry import Vector
from kino.geometry.point import Point

from slam.ray import Contact
from slam._map import Gaussian, GridPoint


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

        self.events = self.reset()

        self.map_gaussians_events: Dict[int, List[Gaussian]] = dict()
        self.map_gaussians: Dict[Tuple[int, int], Gaussian] = dict()
        self.time = 0  # to be incremented everytime the map is updated

        self.agent_trajectory = dict(x=[0], y=[0], theta=[0])

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
                            point_distance,
                            ray.angle_shift,
                        )
                        for point_distance in ray.sampled_distance
                    ]
                )
            else:
                # object detection !
                detection_distance = ray_events[0].distance
                self.events[str(ray.angle_shift)].append(detection_distance)

                # store Free gaussians
                free_points = [
                    point_distance
                    for point_distance in ray.sampled_distance
                    if point_distance < detection_distance
                ]
                timestep_gaussians.extend(
                    [
                        Gaussian(
                            self.free_gaussian_value,
                            self.free_gaussian_radius,
                            point_distance,
                            ray.angle_shift,
                        )
                        for point_distance in free_points
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

    def reset(self) -> dict:
        """
            Returns an empty dictionary
        """
        events: Dict[str, List[float]] = {
            **{"speed": [], "omega": []},
            **{str(ray.angle_shift): [] for ray in self.agent.rays},
        }
        return events

    def get_agent_trajectory(self):
        """
            Reconstructs the agent's trajectory from the first recorded time step
        """
        data = pd.DataFrame(self.events)

        for i, step in data.iterrows():
            thet_rad = np.radians(self.agent_trajectory["theta"][-1])
            self.agent_trajectory["x"].append(
                self.agent_trajectory["x"][-1] + step.speed * np.cos(thet_rad)
            )
            self.agent_trajectory["y"].append(
                self.agent_trajectory["y"][-1] + step.speed * np.sin(thet_rad)
            )
            self.agent_trajectory["theta"].append(
                self.agent_trajectory["theta"][-1] + step.omega
            )

            if self.agent_trajectory["theta"][-1] > 360:
                self.agent_trajectory["theta"][-1] -= 360
            elif self.agent_trajectory["theta"][-1] < 0:
                self.agent_trajectory["theta"][-1] += 360

        self.events = self.reset()

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

        # empty dictionary to speed up next time map is build
        self.map_gaussians_events: Dict[int, List[Gaussian]] = dict()

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
            Xs = x + gauss.std * np.cos(np.linspace(0, 2 * np.pi, 6))
            Ys = y + gauss.std * np.sin(np.linspace(0, 2 * np.pi, 6))

            for x, y in zip(Xs, Ys):
                x, y = round(x, 0), round(y, 0)
                if (x, y) not in self.grid_points.keys():
                    self.grid_points[(x, y)] = GridPoint(
                        x, y, value=gauss.mean * gauss.std
                    )

                if self.grid_points[(x, y)].value >= 0:
                    self.grid_points[(x, y)].value += (
                        2 * gauss.mean * gauss.std
                    )

    def build(self):
        """
            Integrates the stored robot motion to reconstruct the position of the dots,
            if a map was already built, it just adds to it.
        """
        # reconstruct agent position at each time step
        self.get_agent_trajectory()

        # reconstruct the map gaussians
        self.get_map_gaussians()

        # reconstruct grid
        self.get_grid_map()

    def draw(self, ax: plt.Axes, ax2: Optional[plt.Axes] = None):
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
            lw=0.75,
            color=[0.2, 0.2, 0.2],
            zorder=100,
            alpha=1,
            label="agent trajectory",
        )

        # plot points grid
        x = [k[0] for k, v in self.grid_points.items() if v.confidence < 0]
        y = [k[1] for k, v in self.grid_points.items() if v.confidence < 0]
        ax.scatter(
            x, y, color=blue_darker, lw=0.5, ec="k", s=20, alpha=1,
        )

        # plot points for legend
        ax.scatter(0, 0, color=red_dark, s=20, zorder=-100, label="accesible")
        ax.scatter(
            0, 0, color=blue_darker, s=20, zorder=-100, label="inaccessible"
        )
        ax.scatter(0, 0, color=red_light, s=20, zorder=-100, label="uncertain")

        ax.axis("off")
