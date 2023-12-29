import networkx
import networkx as nx
import numpy.random
import shapely
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from rrt.rrt import RRT

x_boundary = 100
y_boundary = 100


class World:
    def __init__(self, raw_world_data):
        # Границы поля
        self.x_boundary = x_boundary
        self.y_boundary = y_boundary

        # Стартовая и терминальная точка
        self.start_point = shapely.Point(raw_world_data[1]['x'], raw_world_data[1]['y'])
        self.terminal_point = shapely.Point(raw_world_data[2]['x'], raw_world_data[2]['y'])

        # Препятствия
        self.obstacles = []
        for i in range(3, len(raw_world_data)):
            vertices = []
            for j in range(len(raw_world_data[i]['points'])):
                vertices.append(shapely.Point(raw_world_data[i]['points'][j]['x'],
                                                    raw_world_data[i]['points'][j]['y']))
            self.obstacles.append(shapely.Polygon(vertices))

        self.rrt = RRT(self.start_point, self.terminal_point,
                       self.x_boundary, self.y_boundary,
                       self.obstacles)

    # Отрисовка результата
    def plot(self, animated: bool):
        fig, ax = plt.subplots()
        fig.set_size_inches(8, 8)
        ax.set_aspect(x_boundary / y_boundary)
        ax.set_xlim(0, x_boundary)
        ax.set_ylim(0, y_boundary)
        frames_num = len(self.rrt.history)+1 if self.rrt.success else len(self.rrt.history)

        if not animated:
            for i in range(len(self.rrt.grid) - 1):
                for j in range(len(self.rrt.grid) - 1):
                    if self.rrt.grid[i][j] == self.rrt.density_limit:
                        xs = [el * self.rrt.density_grid for el in [i, i+1, i+1, i]]
                        ys = [el * self.rrt.density_grid for el in [j, j, j+1, j+1]]
                        ax.fill(xs, ys, 'r', alpha=0.2, edgecolor='r')

        # Стартовая и терминальная вершины
        ax.scatter(*self.start_point.xy, fc='green')
        ax.add_patch(
            plt.Circle((self.terminal_point.x, self.terminal_point.y), self.rrt.precision, color='black', fill=False))
        ax.scatter(*self.terminal_point.xy, fc='red')

        # Препятствия
        for obstacle in self.obstacles:
            ax.fill(*obstacle.exterior.xy, fc='#999999', ec='black')

        # RRT
        if animated:
            frames = [ax.plot([], [], color='black', linewidth=0.5)[0]
                      for i in range(len(self.rrt.history))]
        else:
            edges = [e for e in self.rrt.graph.edges]
            for edge in edges:
                ax.plot([edge[0].x, edge[1].x], [edge[0].y, edge[1].y], color='black', linewidth=0.5)

        # Найденный путь
        if self.rrt.success:
            path = networkx.astar_path(self.rrt.graph, self.rrt.start_point, self.rrt.terminal_point)
            if animated:
                self.rrt.history.append(tuple([node for node in path]))
                frames.append(ax.plot([], [], color='red', linewidth=1)[0])
            else:
                for i in range(len(path) - 1):
                    ax.plot([path[i].x, path[i+1].x], [path[i].y, path[i+1].y], color='red', linewidth=1)

        if animated:
            def anim(i):
                frames[i].set_data([node.x for node in self.rrt.history[i]],
                                   [node.y for node in self.rrt.history[i]])
            ani = animation.FuncAnimation(fig, anim, frames=frames_num, interval=5000/frames_num)

        ax.set_title('Всего узлов: ' + str(self.rrt.graph.number_of_nodes()))

        plt.show()
