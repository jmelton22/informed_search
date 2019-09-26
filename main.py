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

    print('\nSearch algorithm:', end=' ')
    print('Greedy search') if greedy else print('A*')
    print('Heuristic function:', heuristic.__name__)

    start_node = Node(start, '', 0, heuristic(start, goal), greedy)

    return search(grid, start_node, goal, heuristic, unexplored, visited, greedy, path)


def search(grid, node, goal, heuristic, unexplored, visited, greedy, path):
    """
        Recursive informed search. Exits when goal node has been reached or
        when queue of unexplored nodes is empty.

    :return: if goal node is reached, return list of nodes back to the starting node.
             if queue is empty without reaching goal node, return None.
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
    return sum([abs(x - y) for x, y in zip(pt1, pt2)])


def euclidean_distance(pt1, pt2):
    return math.sqrt(sum([(x - y) ** 2 for x, y in zip(pt1, pt2)]))


def set_path(node, path):
    """
        Recursive function to determine the path from the goal node to starting node
        by traversing the parent nodes until reaching the start node.
    """
    path.append((node.g, node.value))
    if node.parent == '':
        return path
    else:
        return set_path(node.parent, path)


def expand_node(grid, node, goal, heuristic, visited, unexplored, greedy):
    """
        Given a node, add its valid neighboring nodes to the unexplored queue.
        Nodes are valid if:
            - their value in the grid is non-zero and
            - they have not already been visited and
            - they are not already in the queue
    """
    def in_unexplored(coord, q):
        return coord in [x.value for x in q]

    def in_visited(coord, l):
        return coord in [x.value for x in l]

    for n in node.get_neighbors(grid):
        path_cost = node.g + step_cost(grid, n)
        temp_node = Node(n, node, path_cost, heuristic(n, goal), greedy)

        # If a grid value is already in queue, but has higher priority
        # Remove that node and add new node (with lower priority) to queue
        if in_unexplored(n, unexplored):
            for duplicate in [x for x in unexplored if x.value == n]:
                if duplicate.priority > temp_node.priority:
                    unexplored.remove(duplicate)
                    heapq.heappush(unexplored, temp_node)
                    heapq.heapify(unexplored)
        elif not in_visited(n, visited):
            heapq.heappush(unexplored, temp_node)


def get_user_coords(grid, text):
    """
        Get and validate user input for starting and goal coordinates.
    """
    while True:
        try:
            print('Enter a {} coordinate (r, c):'.format(text), end=' ')
            coord = [int(x) for x in input().split(',')]
        except ValueError:
            print('Non-numeric coordinate entered')
            continue

        if grid[coord[0]][coord[1]] == 0:
            print('Invalid coordinate on grid')
        else:
            return coord


def main():
    grid = g.read_grid('grid.txt')
    g.print_grid(grid)
    print()

    start = get_user_coords(grid, 'start')
    end = get_user_coords(grid, 'goal')

    path, num_states = informed_search(grid, start, end,
                                       greedy=choice([True, False]),
                                       manhattan=choice([True, False]))
    print('Number of nodes expanded:', num_states)
    print('Path:')

    fname = 'path.txt'

    if path is None:
        print('No path found.')
    else:
        print('\n'.join('\t{:02d} - {}'.format(cost, coord) for cost, coord in path[::-1]))
        print()
        g.output_grid(fname, grid, start, end, [x[1] for x in path])


if __name__ == '__main__':
    main()
