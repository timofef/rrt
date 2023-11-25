import sys, json, argparse
from world.world import World

precision = 3
step = 10
max_nodes = 2000


def main(path: str, animated: bool):
    # Считываем мир из json файла
    f = open(path)
    raw_data = json.load(f)

    # Создаем мир и RRT
    world = World(raw_data)

    # Ищем путь
    world.rrt.search(precision, step, max_nodes)

    # Рисуем найденный путь
    world.plot(animated)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--animated", action="store_true", help="Show exploration animation")
    parser.add_argument("file", help="JSON file")
    args = parser.parse_args()

    main(args.file, args.animated)
