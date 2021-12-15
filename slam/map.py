from typing import List, Dict
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from copy import deepcopy

from myterial import red
from kino.geometry import Vector

from slam.ray import Contact


class Map:
    def __init__(self, agent):
        self.agent = agent

        self.events: Dict[str, List[float]] = {
            **{"speed": [], "omega": []},
            **{str(ray.angle_shift): [] for ray in agent.rays},
        }

    def add(self, *events: Contact):
        """
            Given a list of ray-object contact events (in egocentric coordinates)
            stores the distance each ray picked up an object together witht he robot's
            kinematics
        """

        self.events["speed"].append(self.agent._current_speed)
        self.events["omega"].append(self.agent._current_omega)

        for ray in self.agent.rays:
            ray_events = [ev for ev in events if ev.ray == ray]
            if not ray_events:
                self.events[str(ray.angle_shift)].append(np.nan)
            else:
                self.events[str(ray.angle_shift)].append(
                    ray_events[0].distance
                )

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
        ax.scatter(self.points_x, self.points_y, zorder=150, s=15, color=red)

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
            zorder=-1,
            alpha=0.5,
        )

        ax.set(
            xlim=[
                np.min(self.agent_trajectory["x"]) - 20,
                np.max(self.agent_trajectory["x"]) + 20,
            ],
            ylim=[
                np.min(self.agent_trajectory["y"]) - 20,
                np.max(self.agent_trajectory["y"]) + 20,
            ],
        )

        ax.axis("off")
