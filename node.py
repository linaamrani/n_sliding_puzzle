import numpy as np
import copy
from queue import PriorityQueue
import time
import random
from math import sqrt
from puzzle import *
import solver


class Node:
    def __init__(self, tiles=[], parent=None, g=0, h=0, f=0, heuristic='displaced', goal=[]):
        self.tiles = tiles
        self.puzzle = Puzzle(len(tiles), tiles)
        self.g = g
        self.h = h
        self.f = f
        self.parent = parent
        self.heuristic = heuristic
        self.goal = goal
        self.n = len(tiles)

    def __eq__(self, other):
        return (self.f == other.f)

    def __ne__(self, other):
        return not (self.f == other.f)

    def __lt__(self, other):
        return (self.f < other.f)

    def __gt__(self, other):
        return (self.f > other.f)

    def __le__(self, other):
        return (self.f < other.f) or (self.f == other.f)

    def __ge__(self, other):
        return (self.f > other.f) or (self.f == other.f)

    def def_goal(self):
        n = len(self.tiles)
        #goal_state = [[1, 2], [3, 0]]
        #goal_state = [[1, 2, 3, 4], [5, 6, 7, 8],[9, 10, 11, 12], [13, 14, 15, 0]]
        #goal_state = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]
        goal_state = [list(range(1 + n * i, 1 + n * (i + 1)))
                      for i in range(n)]
        goal_state[-1][-1] = 0
        self.goal = goal_state
        return goal_state

    def isGoal(self):
        t = [elem for sublist in self.tiles for elem in sublist]
        return t == [elem for sublist in self.goal for elem in sublist]

    def find_empty_tile(self):  # find the empty tile in a node's tiles
        tiles = self.tiles
        for i in range(self.n):
            for j in range(self.n):
                if tiles[i][j] == 0:
                    return i, j

    def heuristic_fun(self):
        # misplaced tiles heuristic
        # state is the current state
        # n is the size of the puzzle
        if self.heuristic == 'displaced':
            state_list = []
            tiles = self.tiles
            self.def_goal()  # est ce necessaire???
            goal_state = [elem for sublist in self.goal for elem in sublist]
            #print("GOAL STATE: ", goal_state)
            misplaced = 0
            for i in range(self.n):
                state_list = state_list+self.tiles[i]
            #print("STATE LIST/ ", state_list)
            for i in range(self.n**2):
                if state_list[i] != goal_state[i] and state_list[i] != 0:
                    misplaced += 1
            if self.tiles == initial_state.tiles:
                self.h = misplaced
                self.g = 0
                self.f = misplaced
            else:
                self.h = misplaced
                self.g = self.parent.g + 1
                self.f = self.g + misplaced
        if self.heuristic == 'manhattan':
            sum = 0
            tiles = self.tiles
            n = len(tiles)

            for i in range(n):
                for j in range(n):
                    if tiles[i][j] == 0:
                        continue
                    else:
                        x, y = solver.findGoal(tiles[i][j], tiles)
                        sum += abs(x - i) + abs(y - j)
            if self.tiles == initial_state.tiles:
                print("ici")
                self.h = sum
                self.g = 0
                self.f = self.h
            else:
                self.h = sum
                self.g = self.parent.g + 1
                self.f = self.g + self.h
        if self.heuristic == 'ucs':
            sum = 0
            tiles = self.tiles
            n = len(tiles)
            if self.tiles == initial_state.tiles:
                print("ici")
                self.h = 0
                self.g = 0
                self.f = self.g
            else:
                self.h = 0
                self.g = self.parent.g + 1
                self.f = self.g

    def genChildren(self):
        child_tiles = self.puzzle.possible_actions()
        ret = []
        for i in child_tiles:  # create children nodes
            child = Node(i, self)
            if self.heuristic == 'manhattan':
                child.heuristic_fun()
            if self.heuristic == "displaced":
                child.heuristic_fun()
            ret.append(child)
        return ret

    def isExp(self, exp):
        # return if node is in the expanded nodes list
        tiles = self.tiles
        for i in exp:
            if (tiles == i.tiles) and (i.f <= self.f):
                return i
        return None
