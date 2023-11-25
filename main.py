import sys, json
from world.world import World

precision = 5
step = 3
max_nodes = 2000


def main(path: str):
    # Считываем мир из json файла
    f = open(path)
    raw_data = json.load(f)

    # Создаем мир и RRT
    world = World(raw_data)

    # Ищем путь
    world.rrt.search(precision, step, max_nodes)

    # Рисуем найденный путь
    world.plot()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: python3 main.py <world.json>")
        exit(1)
    main(sys.argv[1])