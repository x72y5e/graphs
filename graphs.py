from collections import deque, namedtuple
import random
import asyncio


Craft = namedtuple("craft", "range velocity id".split())
GRAPH = {"london": {"new york": 5000, "paris": 250, "los angeles": 8000,
                    "singapore": 7000, "edinburgh": 500, "hong kong": 7000},
         "edinburgh": {"london": 500, "inverness": 250},
         "inverness": {"edinburgh": 250},
         "new york": {"london": 5000, "paris": 3200, "los angeles": 3500,
                      "singapore": 11000, "miami": 3000},
         "paris": {"london": 250, "new york": 3200, "madrid": 700, "lyon": 350},
         "lyon": {"paris": 350, "madrid": 500},
         "los angeles": {"london": 8000, "new york": 3500, "singapore": 8000},
         "singapore": {"london": 7000, "new york": 11000, "los angeles": 8000,
                       "hong kong": 1500},
         "madrid": {"paris": 700, "lyon": 500, "santiago": 6500},
         "hong kong": {"aukland": 4000, "sydney": 3500, "singapore": 1500, "london": 7000},
         "aukland": {"hong kong": 4000, "sydney": 2000},
         "sydney": {"aukland": 2000, "hong kong": 3500, "santiago": 6000},
         "santiago": {"sydney": 6000, "miami": 4500, "madrid": 6500},
         "miami": {"santiago": 4500, "new york": 3000}
         }


def check_graph(g):
    assert type(g) == dict
    for k, v in g.items():
        for k1, v1 in v.items():
            try:
                assert v1 == g[k1][k]
            except:
                print("graph fault: {} - {}".format(k, k1))
                return False
    return True


def get_distance(g, x, y):
# breadth-first search
    q = deque([(x, [x], 0)])
    seen = set()
    valid_paths = []

    while q:
        current, path, dist = q.popleft()
        seen.add(current)
        for node in g[current]:
            if node == y:
                valid_paths.append((path + [y], dist + g[current][y]))
            if node not in seen:
                seen.add(node)
                q.append((node, path + [node], dist + g[current][node]))
    return sorted(valid_paths, key=lambda x: x[1])[0]


def dijkstra(g: dict, origin: str) -> dict:
    all_nodes = [(node, 10e6, []) for node in g.keys() if node != origin]
    visited = set()
    current_node, current_distance, current_path = origin, 0, []
    while True:
        # distance to current node from origin
        if g[current_node].get(origin, 10e6) < current_distance:
            current_distance = g[current_node][origin]
            current_path = [origin]

        # distance to neighbours from current node
        neighbours = {node: (dist + current_distance, current_path + [current_node])
                      for node, dist in g[current_node].items()
                      if node not in visited}

        # substitute path through current node for all neighbours if less than currently assigned
        all_nodes = [(node, dist, path) if dist <= neighbours.get(node, [10e6])[0]
                     else (node, neighbours[node][0], neighbours[node][1])
                     for node, dist, path in
                     [x for x in all_nodes if x[0] != current_node]]

        all_nodes.append((current_node, current_distance, current_path))
        visited.add(current_node)
        unvisited = list(filter(lambda x: x[0] not in visited, all_nodes))
        if not unvisited:
            break
        else:
            # select node with minimum current estimated distance
            current_node, current_distance, current_path = sorted(unvisited, key=lambda x: x[1])[0]

    # add the final destination to the path when assembling the dictionary
    return {node: (distance, path + [node]) for (node, distance, path) in all_nodes}


async def traverse(g, craft, origin, destination):
    distance, path = dijkstra(g, origin)[destination]
    current = path[0]
    path = path[1:]
    v, id = craft.velocity, craft.id
    while current != destination:
        print("{} at {}".format(id, current))
        dist = g[current][path[0]]
        print("{} going to {} ({})".format(id, path[0], dist))
        await asyncio.sleep(dist / 500)
        current = path[0]
        path = path[1:]
    print("{} arrived at {}".format(id, current))
    return


async def run_async():
    tasks = [asyncio.ensure_future(traverse(g, c0, "london", "aukland")),
             asyncio.ensure_future(traverse(g, c0, "paris", "inverness"))]
    await asyncio.gather(*tasks)


if __name__ == '__main__':
    g = GRAPH
    if check_graph(g):
        for i in range(0):
            origin, destination = random.sample(g.keys(), 2)
            print("path from {} to {}: {}\n\n".format(
                #origin, destination, get_distance(g, origin, destination))
                origin, destination, dijkstra(g, origin)[destination])
            )
        c0 = Craft(9000, 500, "c0")
        loop = asyncio.get_event_loop()
        loop.run_until_complete(run_async())
        loop.close()

