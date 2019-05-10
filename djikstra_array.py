import numpy as np
import pygame
import time


DIMS = (16 * 32, 16 * 32)

def enlarge(grid):
    i, j = DIMS
    a, b = grid.shape
    x_factor = i // a
    y_factor = j // b
    x_pad = (x_factor // 2 - 10, x_factor // 2 + 10)
    y_pad = (y_factor //2 - 10, y_factor // 2 + 10)
    big_grid = np.zeros(DIMS)
    for x, y in zip(*np.where(grid > 0.)):
        val = grid[x, y]
        big_grid[x * x_factor + x_pad[0]:x * x_factor + x_pad[1],
                 y * y_factor + y_pad[0]:y * y_factor + y_pad[1]] = val
    return big_grid


def is_neighbour(p1: (int, int), p2: (int, int)) -> bool:
    ax, ay = p1
    bx, by = p2
    return abs(ax - bx) <= 1 and abs(ay - by) <= 1


def euclidean_dist(p1, p2):
    ax, ay = p1
    bx, by = p2
    return np.sqrt(abs(ax - bx)**2 + abs(ay - by)**2)


def dijkstra(a: np.ndarray, origin: (int, int)):
    all_nodes = np.ones_like(a) * 10e6
    unvisited = set([(x, y)
                     for x in range(a.shape[0])
                     for y in range(a.shape[1])])
    paths = {node: [] for node in unvisited}
    unvisited.remove(origin)
    visited = set()
    current_node, current_distance = origin, 0
    all_nodes[current_node] = 0

    while unvisited:
        # if path to current node from origin
        if is_neighbour(origin, current_node):
            dist_from_origin = abs(current_node[0] - origin[0]) + abs(current_node[1] - origin[1])
            current_distance = min(all_nodes[current_node], dist_from_origin)
            current_path = [origin, current_node]
            paths[current_node] = current_path
        else:
            current_distance = all_nodes[current_node]
            current_path = paths[current_node]

        # distance to neighbours from current node
        neighbours = list(filter(lambda x: is_neighbour(current_node, x), unvisited))

        x, y = current_node
        for neighbour in neighbours:
            nb_dist_from_current = euclidean_dist(current_node, neighbour)
            dist_thru_current = current_distance + nb_dist_from_current
            if dist_thru_current < all_nodes[neighbour]:
                all_nodes[neighbour] = dist_thru_current
                paths[neighbour] = current_path + [neighbour]
                yield neighbour, dist_thru_current

        all_nodes[x, y] = current_distance
        visited.add(current_node)
        current_node = list(sorted(unvisited, key=lambda x: all_nodes[x]))[0]
        unvisited.remove(current_node)


def run():

    pygame.init()
    display = pygame.display.set_mode(DIMS)
    clock = pygame.time.Clock()
    done = False

    a = np.arange(16 * 16, dtype=np.int16).reshape(-1, 16)
    path_gen = dijkstra(a, (0, 0))

    grid = np.zeros((16, 16))
    #grid[np.random.randint(16), np.random.randint(16)] = 1

    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        try:
            coord, dist = next(path_gen)
        except StopIteration:
            break
        grid[coord] = dist * 10
        surf = pygame.surfarray.make_surface(enlarge(grid).T)
        display.blit(surf, (0, 0))
        pygame.display.update()

        clock.tick(60)
        time.sleep(.1)


if __name__ == '__main__':
    run()
