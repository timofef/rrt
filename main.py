import sys, json, argparse, time

from world.world import World


def main(path: str, animated: bool, precision: float, step: float, max_nodes: int, bias: int, density_limit: int,
         density_grid: int):
    if density_limit > 0 > density_grid or density_limit > 0 > density_grid:
        print("density_limit and density_grid must be > 0")

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
    start = time.time()
    world.rrt.search(animated, precision, step, max_nodes, bias, density_limit, density_grid)
    end = time.time()
    print("Elapsed time: " + str(end-start) + "s")

    # Отрисовываем найденный путь
    world.plot(animated)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-mn", "--max_nodes", default=2000, help="Максимальное число узлов в дереве", type=int)
    parser.add_argument("-s", "--step", default=5, help="Минимальный размер шага", type=float)
    parser.add_argument("-p", "--precision", default=3, help="Погрешность конечной вершины", type=float)
    parser.add_argument("-a", "--animated", action="store_true", help="Показать анимацию")
    parser.add_argument("-b", "--bias", default=1, help="Bias", type=int)
    parser.add_argument("-dl", "--density_limit", default=-1, help="Предельная плотность, при которой можно вставить новый узел", type=float)
    parser.add_argument("-dg", "--density_grid", default=-1, help="dfghjk", type=int)
    parser.add_argument("file", help="JSON file")
    args = parser.parse_args()

    main(args.file,
         args.animated,
         args.precision,
         args.step,
         args.max_nodes,
         args.bias,
         args.density_limit,
         args.density_grid)
