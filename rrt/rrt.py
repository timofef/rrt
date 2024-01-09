import random

import networkx as nx
import shapely
from shapely.ops import nearest_points
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

        self.history = []
        self.global_counter = 1

        self.density_limit = 0
        self.density_grid = 0
        self.grid = None
        self.last_inserted = start

        self.mode = 0

    def search(self, animated: bool, precision: float, step_size: float, max_nodes: int, bias: int, density_limit: int,
                 density_grid: int):
        random.seed(9)
        self.precision = precision
        node_count = 1
        success = False
        grid_size = self.x_boundary // density_grid
        self.density_limit = density_limit
        self.density_grid = density_grid
        self.grid = np.zeros([grid_size, grid_size], dtype=int)

        while not (success or node_count >= max_nodes):
            self.global_counter += 1
            # Обычный RRT
            if self.mode == 0:
                new_point = self.generate_point(bias)
                nearest_node = self.find_nearest_node(new_point)
                new_node, step, changed_cell, on_finish = self.get_new_node(nearest_node, new_point, step_size)
                if new_node is None:
                    continue
                node_count += 1

                if on_finish:
                    self.mode = 1

            # Финишная прямая
            elif self.mode == 1:
                if shapely.distance(self.goal, self.last_inserted) > self.precision + step_size:
                    dist = shapely.distance(self.goal, self.last_inserted)
                    x_direction = self.goal.x - self.last_inserted.x
                    y_direction = self.goal.y - self.last_inserted.y
                    step = min(dist, step_size) / dist
                    new_node = shapely.Point(self.last_inserted.x + x_direction * step, self.last_inserted.y + y_direction * step)
                else:
                    new_node = self.goal
                nearest_node = self.last_inserted
                node_count += 1

            self.graph.add_node(new_node)
            self.graph.add_edge(nearest_node, new_node, weight=step)
            self.last_inserted = new_node

            if animated:
                self.history.append(((new_node, nearest_node), changed_cell))

            if shapely.distance(new_node, self.goal) < self.precision:
                self.terminal_point = new_node
                success = True
            print("Total nodes: " + str(node_count), end='\r')
        print("Total nodes: " + str(node_count))
        print("Attempted nodes: " + str(self.global_counter))
        self.success = success

    def generate_point(self, bias: int):
        if bias != 1:
            if self.global_counter % bias == 0:
                return self.goal

        free_spaces = np.nonzero(self.density_grid > self.grid)

        while True:
            rand_index = random.randint(0, len(free_spaces[0]) - 1)
            grid_cell_x, grid_cell_y = free_spaces[0][rand_index], free_spaces[1][rand_index]
            x = (random.uniform(0, 1) + grid_cell_x) * self.density_grid
            y = (random.uniform(0, 1) + grid_cell_y) * self.density_grid

            point = shapely.Point(x, y)
            is_inside_obstacle = False
            for obstacle in self.obstacles:
                is_inside_obstacle = obstacle.contains(point)
                if is_inside_obstacle:
                    break
            if not is_inside_obstacle:
                break

        return point

    def find_nearest_node(self, point: shapely.Point) -> shapely.Point:
        nearest = nearest_points(point, shapely.MultiPoint(list(self.graph.nodes)))

        return nearest[1]

    def get_new_node(self, near: shapely.Point, rand: shapely.Point, step_size: float):
        dist = shapely.distance(near, rand)
        x_direction = rand.x - near.x
        y_direction = rand.y - near.y

        step = min(dist, step_size) / dist
        new_node = shapely.Point(near.x + x_direction * step, near.y + y_direction * step)

        for obstacle in self.obstacles:
            crosses_obstacle = shapely.intersects(shapely.LineString([near, new_node]), obstacle)
            if crosses_obstacle:
                return None, step, None, False

        on_finish = True
        for obstacle in self.obstacles:
            if shapely.intersects(shapely.LineString([self.goal, new_node]), obstacle):
                on_finish = False
                break

        changed_cell = None
        grid_x, grid_y = int((new_node.x / self.x_boundary) * len(self.grid)), int((new_node.y / self.y_boundary) * len(self.grid))
        if self.grid[grid_x][grid_y] + 1 > self.density_limit:
            return None, step, None, False
        else:
            self.grid[grid_x][grid_y] += 1
            if self.grid[grid_x][grid_y] == self.density_limit:
                changed_cell = (grid_x, grid_y)

        return new_node, step, changed_cell, on_finish
