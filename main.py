#!/usr/bin/env python3

from queue import Queue, LifoQueue
from node import Node
import grid as g


def informed_search(grid, start, goal, breadth=True):
    """
        Handles initialization of data structures for grid search.
    """
    visited, path = [], []

    # Queue for breadth-first search; Stack (LifoQueue) for depth-first search
    unexplored = Queue() if breadth else LifoQueue()

    return search(grid, Node(start, ''), goal, unexplored, visited, path)


def search(grid, node, goal, unexplored, visited, path):
    """
        Recursive uninformed search. Exits when goal node has been reached or
        when queue of unexplored nodes is empty.

    :return: if goal node is reached, return list of nodes back to the starting node.
             if queue is empty without reaching goal node, return None.
    """
    visited.append(node)

    if node.value == goal:
        return set_path(node, path)
    else:
        # Add valid neighboring nodes to unexplored queue
        expand_node(grid, node, visited, unexplored)

        if unexplored.empty():
            return None
        else:
            # Search through next node in queue
            return search(grid, unexplored.get(), goal, unexplored, visited, path)


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


def expand_node(grid, node, visited, unexplored):
    """
        Given a node, add its valid neighboring nodes to the unexplored queue.
        Nodes are valid if:
            - their value in the grid is 0 and
            - they have not already been visited and
            - they are not already in the queue
    """
    def in_unexplored(coord, q):
        return coord in [x.value for x in list(q.queue)]

    def in_visited(coord, l):
        return coord in [x.value for x in l]

    for n in node.get_neighbors(grid):
        if not in_visited(n, visited) and not in_unexplored(n, unexplored):
            unexplored.put(Node(n, node))


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

        if grid[coord[0]][coord[1]] != 0:
            print('Invalid coordinate on grid')
        else:
            return coord


def main():
    # grid = g.read_grid('grid.txt')
    grid = g.make_grid(10, 10)

    # Print grid with a space between columns and a newline between rows
    print('\n'.join(' '.join([str(col) for col in row]) for row in grid))
    print()

    start = get_user_coords(grid, 'start')
    end = get_user_coords(grid, 'goal')

    # Prompt user to choose breadth-first or depth-first search
    while True:
        print('Choose breadth-first (0) or depth-first (1) search:', end=' ')
        try:
            # Default to breadth-first unless 1 entered
            breadth = int(input()) != 1
            break
        except ValueError:
            print('Invalid selection. Please enter 0 or 1')

    path = uninformed_search(grid, start, end, breadth)
    fname = 'path.txt'

    if path is None:
        print('No path found.')
    else:
        g.output_grid(fname, grid, start, end, path)


if __name__ == '__main__':
    main()
