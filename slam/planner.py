import warnings

warnings.filterwarnings(action="ignore", module="libpysal")

import matplotlib.pyplot as plt
from libpysal import weights
import networkx as nx
import numpy as np
from typing import List, Optional
from random import choice
from networkx.algorithms.shortest_paths.generic import shortest_path

from kino.geometry.point import Point

from slam._map import GridPoint


class Planner:
    distance_threshold: float = 1.5  # points within this distance are connected

    @property
    def accessible(self) -> List[dict]:
        """
            Returns a list of nodes (dicts) for nodes representing 
            accessible locations
        """
        idxs = list(nx.get_node_attributes(self.graph, "accessible").keys())
        return [self.graph.nodes[idx] for idx in idxs]

    @property
    def uncertain(self) -> List[dict]:
        """
            Returns a list of nodes (dicts) for nodes representing 
            uncertain locations
        """
        idxs = list(nx.get_node_attributes(self.graph, "uncertain").keys())
        return [self.graph.nodes[idx] for idx in idxs]

    def build(self, grid_points: List[GridPoint]):
        """
            Creates a network with physically close points being connected, including only nodes with reasonable confidence 
            of them being open.
        """
        self.grid_points = grid_points
        accessible_points = [
            pt for pt in self.grid_points if pt.confidence >= 0
        ]
        self.coordinates = np.array(
            [
                [pt.x for pt in accessible_points],
                [pt.y for pt in accessible_points],
            ]
        ).T
        self.graph = weights.DistanceBand.from_array(
            self.coordinates,
            threshold=self.distance_threshold,
            silence_warnings=True,
        ).to_networkx()

        # add confidence value to nodes and other attributes
        for node_n, confidence in enumerate(
            [pt.confidence for pt in accessible_points]
        ):
            self.graph.nodes[node_n]["confidence"] = confidence
            self.graph.nodes[node_n]["x"] = accessible_points[node_n].x
            self.graph.nodes[node_n]["y"] = accessible_points[node_n].y
            self.graph.nodes[node_n]["node_n"] = node_n

            if confidence:
                self.graph.nodes[node_n]["accessible"] = True
            else:
                self.graph.nodes[node_n]["uncertain"] = True

    def get_uncertain_node(self) -> Optional[dict]:
        """
            Returns a random uncertain node
        """
        if not self.uncertain:
            return None
        return choice(self.uncertain)

    def get_closest_node(self, point: Point) -> dict:
        """
            Returns the graph node closest to a point
        """
        dist = np.apply_along_axis(
            np.linalg.norm, 1, self.coordinates - point.xy
        )
        return self.graph.nodes[np.argmin(dist)]

    def plan_route(self, agent, target_node: dict) -> List[dict]:
        """
            Plans the shourtest route along the graph from the agent's current
            location to the selected node.
        """
        # get agent's start node
        start_node = self.get_closest_node(
            Point(
                agent.map.agent_trajectory["x"][-1],
                agent.map.agent_trajectory["y"][-1],
            )
        )

        # get the shortest path
        path_idx: List[int] = shortest_path(
            self.graph, start_node["node_n"], target_node["node_n"]
        )
        path: List[dict] = [self.graph.nodes[idx] for idx in path_idx]

        return path

    def draw(self, ax: plt.Axes):
        positions = dict(zip(self.graph.nodes, self.coordinates))

        nx.draw(
            self.graph,
            positions,
            node_color=[
                node["confidence"] for node in self.graph.nodes.values()
            ],
            cmap="Reds",
            vmin=-0.5,
            vmax=2,
            width=0.5,  # edge widht
            edge_color="k",  # edge color
            ax=ax,
            node_size=0.25 * (self.coordinates.max() - self.coordinates.min()),
            linewidths=0.5,  # node lw
            edgecolors="w",  # node ec
        )
