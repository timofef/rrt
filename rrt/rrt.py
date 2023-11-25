import random

import networkx as nx
import shapely
import numpy as np


class RRT:
    def __init__(self,
                 start: shapely.Point, terminal: shapely.Point,
                 x_boundary: int, y_boundary: int,
                 obstacles: list[shapely.Polygon]):
        self.graph = nx.Graph()
        self.graph.add_node(start)

        self.start_point = start
        self.goal = terminal
        self.terminal_point = None

        self.x_boundary = x_boundary
        self.y_boundary = y_boundary

        self.obstacles = obstacles

        self.success = False

        self.precision = 0

    def search(self, precision: float, step_size: float, max_nodes: int):
        random.seed(10)
        self.precision = precision
        node_count = 0
        success = False
        while not (success or node_count >= max_nodes):
            new_point = self.generate_point()
            nearest_node = self.find_nearest_node(new_point)
            new_node, step = self.get_new_node(nearest_node, new_point, step_size)
            if new_node is None:
                continue

            self.graph.add_node(new_node)
            self.graph.add_edge(nearest_node, new_node, weight=step)

            node_count += 1

            if shapely.distance(new_node, self.goal) < precision:
                self.terminal_point = new_node
                success = True

        self.success = success
        print(node_count)

    def generate_point(self):
        while True:
            x = random.uniform(0, self.x_boundary)
            y = random.uniform(0, self.y_boundary)
            point = shapely.Point(x, y)
            is_inside_obstacle = False
            for obstacle in self.obstacles:
                is_inside_obstacle = obstacle.contains(point)
                if is_inside_obstacle:
                    break
            if not is_inside_obstacle:
                break
        return point

    def find_nearest_node(self, point: shapely.Point):
        nearest = None
        min_dist = 10000.

        for node in self.graph.nodes:
            dist = shapely.distance(point, node)
            if dist < min_dist:
                min_dist = dist
                nearest = node

        return nearest

    def get_new_node(self, near: shapely.Point, rand: shapely.Point, step_size: float):
        dist = np.sqrt(np.square(rand.x - near.x) + np.square(rand.y - near.y))
        x_direction = (rand.x - near.x) / dist
        y_direction = (rand.y - near.y) / dist

        step = min(dist, step_size)
        new_node = shapely.Point(near.x + x_direction * step, near.y + y_direction * step)

        for obstacle in self.obstacles:
            crosses_obstacle = shapely.intersects(shapely.LineString([near, new_node]), obstacle)
            if crosses_obstacle:
                return None, step

        return new_node, step
