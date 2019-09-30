#!/usr/bin/env python3

import heapq
from node import Node
import grid as g
import math
from random import choice


def informed_search(grid, start, goal, greedy=True, manhattan=True):
    """
        Handles initialization of data structures for grid search.
    """
    visited, unexplored, path = [], [], []
    heuristic = manhattan_distance if manhattan else euclidean_distance

    print('Search algorithm:', 'Greedy search' if greedy else 'A*')
    print('Heuristic function:', heuristic.__name__)

    start_node = Node(start, None, 0, heuristic(start, goal), greedy)

    return search(grid, start_node, goal, heuristic, unexplored, visited, greedy, path)


def search(grid, node, goal, heuristic, unexplored, visited, greedy, path):
    """
        Recursive search. Exits when goal node has been reached or
        when queue of unexplored nodes is empty.

    :return: if goal node is reached, return a list of tuples with (coordinates, path cost)
                back to the starting node and the number of nodes visited in search.
             if queue is empty without reaching goal node, return None and the number of nodes visited in search.
    """
    visited.append(node)

    if node.value == goal:
        return set_path(node, path), len(visited)
    else:
        # Add valid neighboring nodes to unexplored queue
        expand_node(grid, node, goal, heuristic, visited, unexplored, greedy)

        if not unexplored:
            return None, len(visited)
        else:
            # Search through next node in queue
            return search(grid, heapq.heappop(unexplored), goal, heuristic, unexplored, visited, greedy, path)


def step_cost(grid, pt):
    return grid[pt[0]][pt[1]]


def manhattan_distance(pt1, pt2):
    return sum([abs(d1 - d2) for d1, d2 in zip(pt1, pt2)])


def euclidean_distance(pt1, pt2):
    return math.sqrt(sum([(d1 - d2) ** 2 for d1, d2 in zip(pt1, pt2)]))


def set_path(node, path):
    """
        Recursive function to determine the path from the goal node to starting node
        by traversing the parent nodes until reaching the start node.
    """
    path.append((node.value, node.g))
    if node.parent is None:
        return path
    else:
        return set_path(node.parent, path)


def expand_node(grid, node, goal, heuristic, visited, unexplored, greedy):
    """
        Given a node, push its valid neighboring nodes to the unexplored queue.
        Nodes are valid if:
            - They have a non-zero grid value, have not already been visited, and are not already in the queue

            (Only relevant for A*)
            - Or they have the same coordinates as a node in queue but with a lower path cost
            - In which case, the node with higher path cost is removed from queue and
            the new node is pushed to the queue
    """
    def in_unexplored(coord, q):
        return coord in [x.value for x in q]

    def in_visited(coord, l):
        return coord in [x.value for x in l]

    for n in node.get_neighbors(grid):
        # Path cost of neighbor is the path cost to the parent + step cost to new coord
        path_cost = node.g + step_cost(grid, n)
        temp_node = Node(n, node, path_cost, heuristic(n, goal), greedy)

        if in_unexplored(n, unexplored):
            for duplicate in [x for x in unexplored if x.value == n and x.priority > temp_node.priority]:
                unexplored.remove(duplicate)
                heapq.heappush(unexplored, temp_node)
        elif not in_visited(n, visited):
            heapq.heappush(unexplored, temp_node)


def get_user_coords(grid, text):
    """
        Get and validate user input for starting and goal coordinates.
    """
    while True:
        try:
            coord = [int(x) for x in input('Enter a {} coordinate (r, c): '.format(text)).split(',')]
        except ValueError:
            print('Non-numeric coordinate entered')
            continue

        if step_cost(grid, coord) == 0:
            print('Invalid coordinate on grid')
        else:
            return coord


def main():
    grid = g.read_grid('grid.txt')
    # grid = g.make_grid(20, 20)
    g.print_grid(grid)
    print()

    start = get_user_coords(grid, 'start')
    end = get_user_coords(grid, 'goal')
    print('-' * 15)

    # Randomly toggles search and heuristic methods
    path, num_states = informed_search(grid, start, end,
                                       greedy=choice([True, False]),
                                       manhattan=choice([True, False]))
    print('Number of nodes expanded:', num_states)
    print('-' * 15)

    fname = 'path.txt'

    if path is None:
        print('No path found.')
    else:
        print('Path length: {} nodes'.format(len(path)))
        print('Total cost:', path[0][1])
        print()
        g.output_grid(fname, grid, start, end, [x[0] for x in path])
        print()
        print('Path: coordinate - cost')
        print('\n'.join('\t{} - {:02d}'.format(coord, cost) for coord, cost in path[::-1]))


if __name__ == '__main__':
    main()
