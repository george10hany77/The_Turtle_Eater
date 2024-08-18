#!/usr/bin/env python3
import re
from sympy import li
from .AStarNode import AStarNode
import math

class PathFinding:
    DIAGONAL_COST = 14
    STRAIGHT_COST = 10

    def __init__(self, width, height):
        self.open_list = []
        self.closed_list = []
        self.grid = [[AStarNode(j, i) for j in range(width)] for i in range(height)]
        self.width = width
        self.height = height

    def add_obstacles(self, obstacles):
        if obstacles:
            for obstacle in obstacles:
                x = obstacle.get_x()
                y = obstacle.get_y()
                try:
                    self.grid[y][x].set_is_walkable(False)
                    self.closed_list.append(self.grid[y][x])
                except IndexError:
                    print("Error in add obstacles!")

    def calc_distance(self, start_node, end_node):
        x_diff = abs(start_node.get_x() - end_node.get_x())
        y_diff = abs(start_node.get_y() - end_node.get_y())
        remaining = abs(x_diff - y_diff)
        return remaining * self.STRAIGHT_COST + min(x_diff, y_diff) * self.DIAGONAL_COST

    def a_star(self, start, end) -> list[AStarNode]:
        start.set_g_cost(0)
        start.set_h_cost(self.calc_distance(start, end))
        start.set_prev_node(None)
        end.set_h_cost(0)
        self.open_list.append(start)

        while self.open_list:
            open_node = self.get_lowest_open_node()
            if open_node == end or open_node.get_h_cost() == 0:
                print("7amdella 3ala elsalama")
                return self.generate_path(open_node)

            neighbours = self.get_neighbors_3(open_node)
            for neighbour in neighbours:
                if neighbour in self.closed_list:
                    continue

                neighbour.set_h_cost(self.calc_distance(neighbour, end))
                possible_g_cost = open_node.get_g_cost() + self.calc_distance(neighbour, open_node)

                if neighbour.get_g_cost() > possible_g_cost:
                    neighbour.set_g_cost(possible_g_cost)
                    neighbour.set_prev_node(open_node)

                neighbour.set_f_cost(neighbour.get_g_cost() + neighbour.get_h_cost())

                if neighbour not in self.open_list:
                    self.open_list.append(neighbour)

            self.open_list.remove(open_node)
            self.closed_list.append(open_node)

        return None

    def generate_path(self, open_node) -> list[AStarNode]:
        path = []
        while open_node:
            path.append(open_node)
            open_node = open_node.prev_node
        path.reverse()
        self.mapList(path)
        return path

    def get_neighbors(self, open_node) -> list[AStarNode]:
        x = open_node.get_x()
        y = open_node.get_y()
        x = int(x)
        y = int(y)
        neighbours = []

        if x - 1 >= 0 and self.grid[y][x - 1].is_walkable and self.grid[y][x - 1] not in self.closed_list:
            neighbours.append(self.grid[y][x - 1])  # left
        if (x + 1 < len(self.grid[y]) and self.grid[y][x + 1].is_walkable and
                self.grid[y][x + 1] not in self.closed_list):
            neighbours.append(self.grid[y][x + 1])  # right
        if (y - 1 >= 0 and self.grid[y - 1][x].is_walkable and self.grid[y - 1][x]
                not in self.closed_list):
            neighbours.append(self.grid[y - 1][x])  # up
        if y + 1 < len(self.grid) and self.grid[y + 1][x].is_walkable and self.grid[y + 1][x] not in self.closed_list:
            neighbours.append(self.grid[y + 1][x])  # down
        if (x - 1 >= 0 and y - 1 >= 0 and self.grid[y - 1][x - 1].is_walkable and
                self.grid[y - 1][x - 1] not in self.closed_list):
            neighbours.append(self.grid[y - 1][x - 1])  # up-left
        if (x - 1 >= 0 and y + 1 < len(self.grid) and self.grid[y + 1][x - 1].is_walkable and
                self.grid[y + 1][x - 1] not in self.closed_list):
            neighbours.append(self.grid[y + 1][x - 1])  # down-left
        if (x + 1 < len(self.grid[y]) and y - 1 >= 0 and self.grid[y - 1][x + 1].is_walkable and
                self.grid[y - 1][x + 1] not in self.closed_list):
            neighbours.append(self.grid[y - 1][x + 1])  # up-right
        if (x + 1 < len(self.grid[y]) and y + 1 < len(self.grid) and self.grid[y + 1][x + 1].is_walkable and
                self.grid[y + 1][x + 1] not in self.closed_list):
            neighbours.append(self.grid[y + 1][x + 1])  # down-right

        return neighbours

    def get_neighbors_2(self, open_node) -> list[AStarNode]:
        x = open_node.get_x()
        y = open_node.get_y()
        x = int(x)
        y = int(y)
        neighbours = []
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        for dx, dy in directions:
            nx = x + dx
            ny = y + dy
            if 0 <= nx < len(self.grid[0]) and 0 <= ny < len(self.grid):
                if self.grid[ny][nx] in self.closed_list:
                    continue
                neighbours.append(self.grid[ny][nx])

        return neighbours

    def get_neighbors_3(self, open_node:AStarNode):
        x = open_node.get_x()
        y = open_node.get_y()
        x = int(x)
        y = int(y)
        neighbors = []

        if x - 1 >= 0 and self.grid[y][x - 1] not in self.closed_list:  # left
            neighbors.append(self.grid[y][x - 1])

        if x + 1 < len(self.grid[0]) and self.grid[y][x + 1] not in self.closed_list:  # right
            neighbors.append(self.grid[y][x + 1])

        if y - 1 >= 0 and self.grid[y - 1][x] not in self.closed_list:  # up
            neighbors.append(self.grid[y - 1][x])

        if y + 1 < len(self.grid) and self.grid[y + 1][x] not in self.closed_list:  # down
            neighbors.append(self.grid[y + 1][x])

        if x - 1 >= 0 and y - 1 >= 0 and self.grid[y - 1][x - 1] not in self.closed_list:  # up-left
            if not (self.grid[y][x - 1] in self.closed_list and self.grid[y - 1][x] in self.closed_list):
                neighbors.append(self.grid[y - 1][x - 1])

        if x - 1 >= 0 and y + 1 < len(self.grid) and self.grid[y + 1][x - 1] not in self.closed_list:  # down-left
            if not (self.grid[y][x - 1] in self.closed_list and self.grid[y + 1][x] in self.closed_list):
                neighbors.append(self.grid[y + 1][x - 1])

        if x + 1 < len(self.grid[0]) and y - 1 >= 0 and self.grid[y - 1][x + 1] not in self.closed_list:  # up-right
            if not (self.grid[y][x + 1] in self.closed_list and self.grid[y - 1][x] in self.closed_list):
                neighbors.append(self.grid[y - 1][x + 1])

        if x + 1 < len(self.grid[0]) and y + 1 < len(self.grid) and self.grid[y + 1][
            x + 1] not in self.closed_list:  # down-right
            if not (self.grid[y][x + 1] in self.closed_list and self.grid[y + 1][x] in self.closed_list):
                neighbors.append(self.grid[y + 1][x + 1])

        return neighbors

    def map(self, node:AStarNode): # works with turtlesim's map ONLY !!
        node.set_y(self.height-1 - node.get_y())
    
    def mapList(self, listNode:list[AStarNode]):
        for n in listNode:
            self.map(n)

    def mapList_2(self, listNode:list[AStarNode]) -> list[AStarNode]:
        res = []
        for n in listNode:
            new_node = AStarNode(n.get_x(), self.height-1 - n.get_y())
            res.append(new_node)
        return res

    def get_lowest_open_node(self) -> AStarNode:
        return min(self.open_list, key=lambda node: node.get_f_cost())

    def get_grid(self):
        return self.grid

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height
