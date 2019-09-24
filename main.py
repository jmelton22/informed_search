#!/usr/bin/env python3

from queue import PriorityQueue
from node import Node
import grid as g
import math


def informed_search(grid, start, goal, greedy=True, manhattan=True):
    """
        Handles initialization of data structures for grid search.
    """
    visited, path = [], []
    unexplored = PriorityQueue()
    heuristic = manhattan_distance if manhattan else euclidean_distance
    print()
    print('Heuristic function:', heuristic.__name__)

    # TODO: Update g value with correct path_cost function
    start_node = Node(start, '', step_cost(grid, start), heuristic(start, goal), greedy)

    return search(grid, start_node, goal, heuristic, unexplored, visited, greedy, path)


def search(grid, node, goal, heuristic, unexplored, visited, greedy, path):
    """
        Recursive uninformed search. Exits when goal node has been reached or
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

        if unexplored.empty():
            return None
        else:
            # Search through next node in queue
            return search(grid, unexplored.get(), goal, heuristic, unexplored, visited, greedy, path)


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
    path.append(node.value)
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
        for x in list(q.queue):
            if coord == x.value and step_cost(grid, coord) >= x.priority:
                return True
        return False

    def in_visited(coord, l):
        return coord in [x.value for x in l]

    # TODO: Update choosing when new nodes are added to the queue
    for n in node.get_neighbors(grid):
        if not in_visited(n, visited) and not in_unexplored(n, unexplored):
            unexplored.put(Node(n, node, step_cost(grid, n), heuristic(n, goal), greedy))


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
    g.print_grid(grid)  # Print formatted grid
    print()

    start = get_user_coords(grid, 'start')
    end = get_user_coords(grid, 'goal')

    path, num_states = informed_search(grid, start, end, greedy=True)
    print('Number of nodes expanded:', num_states)
    print('Path:', path[::-1])
    print()

    fname = 'path.txt'

    if path is None:
        print('No path found.')
    else:
        g.output_grid(fname, grid, start, end, path)


if __name__ == '__main__':
    main()
