import sys, json, argparse

from world.world import World


def main(path: str, animated: bool, precision: float, step: float, max_nodes: int):
    print()
    print("Animation: " + str(animated))
    print("Precision: " + str(precision))
    print("Step:      " + str(step))
    print("Max nodes: " + str(max_nodes))

    # Считываем мир из json файла
    f = open(path)
    raw_data = json.load(f)

    # Создаем мир и RRT
    world = World(raw_data)

    # Ищем путь
    print()
    world.rrt.search(animated, precision, step, max_nodes)
    print()

    # Рисуем найденный путь
    world.plot(animated)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-mn", "--max_nodes", default=2000, help="Максимальное число узлов в дереве", type=int)
    parser.add_argument("-s", "--step", default=5, help="Минимальный размер шага", type=float)
    parser.add_argument("-p", "--precision", default=3, help="Погрешность конечной вершины", type=float)
    parser.add_argument("-a", "--animated", action="store_true", help="Показать анимацию")
    parser.add_argument("file", help="JSON file")
    args = parser.parse_args()

    main(args.file, args.animated, args.precision, args.step, args.max_nodes)
