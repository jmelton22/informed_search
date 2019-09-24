#!/usr/bin/env python3


class Node:
    def __init__(self, value, parent, g, h, greedy):
        self.value = value
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h
        self.priority = self.g if greedy else self.f

    def __lt__(self, other):
        return self.priority < other.priority

    def get_neighbors(self, grid):
        # First row nodes have no 'up' neighbor
        up = [self.value[0] - 1, self.value[1]] if self.value[0] > 0 else None
        # Last col nodes have no 'right' neighbor
        right = [self.value[0], self.value[1] + 1] if self.value[1] < len(grid[0]) - 1 else None
        # Last row nodes have no 'down' neighbor
        down = [self.value[0] + 1, self.value[1]] if self.value[0] < len(grid) - 1 else None
        # First col nodes have no 'left' neighbor
        left = [self.value[0], self.value[1] - 1] if self.value[1] > 0 else None

        # Return neighboring nodes which have non-zero value in grid
        return [coord for coord in [up, right, down, left] if coord is not None and grid[coord[0]][coord[1]] != 0]
