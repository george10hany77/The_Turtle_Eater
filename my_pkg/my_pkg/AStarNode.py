#!/usr/bin/env python3
class AStarNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g_cost = float('inf')
        self.h_cost = float('inf')
        self.f_cost = float('inf')
        self.prev_node = None
        self.is_walkable = True

    def set_prev_node(self, node):
        self.prev_node = node

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y
    
    def set_x(self, x:float):
        self.x = x

    def set_y(self, y:float):
        self.y = y

    def get_g_cost(self):
        return self.g_cost

    def set_g_cost(self, cost):
        self.g_cost = cost

    def get_h_cost(self):
        return self.h_cost

    def set_h_cost(self, cost):
        self.h_cost = cost

    def get_f_cost(self):
        return self.f_cost

    def set_f_cost(self, cost):
        self.f_cost = cost

    def set_is_walkable(self, is_walkable):
        self.is_walkable = is_walkable

    def __str__(self):
        return f"({self.x}, {self.y}) - G: {self.g_cost}, H: {self.h_cost}, F: {self.f_cost}, Walkable: {self.is_walkable}"

    def __repr__(self):
        return f"({self.x}, {self.y})"