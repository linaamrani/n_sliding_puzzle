import numpy as np
import copy
from queue import PriorityQueue
from queue import Queue
import time
import random
from math import sqrt
from puzzle import *


class Node:
    def __init__(self, tiles, parent, heuristic, goal, g=0, h=0, f=0):
        self.tiles = tiles
        self.puzzle = Puzzle(len(tiles), tiles)
        self.g = g
        self.h = h
        self.f = f
        self.parent = parent
        self.heuristic = heuristic
        self.goal = goal
        self.n = len(tiles)

    def __eq__(self, node):
        if (self.f == node.f):
            return True
        return False

    def __ne__(self, node):
        if not (self.f == node.f):
            return True
        return False

    def __lt__(self, node):
        if (self.f < node.f):
            return True
        return False

    def __gt__(self, node):
        if (self.f > node.f):
            return True
        return False

    def __le__(self, node):
        if (self.f < node.f) or (self.f == node.f):
            return True
        return False

    def __ge__(self, node):
        if (self.f > node.f) or (self.f == node.f):
            return True
        return False

    def def_goal(self):
        n = len(self.tiles)
        #goal_state = [[1, 2], [3, 0]]
        #goal_state = [[1, 2, 3, 4], [5, 6, 7, 8],[9, 10, 11, 12], [13, 14, 15, 0]]
        #goal_state = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]
        goal_state = [list(range(1 + n * i, 1 + n * (i + 1)))
                      for i in range(n)]
        goal_state[-1][-1] = 0
        self.goal = goal_state

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
            print("MANHATTAN ")
            sum = 0
            tiles = self.tiles
            n = len(tiles)

            for i in range(n):
                for j in range(n):
                    if tiles[i][j] == 0:
                        continue
                    else:
                        x, y = findGoal(tiles[i][j], self.goal)
                        sum += abs(x - i) + abs(y - j)

            if self.tiles == initial_state.tiles:
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
                self.h = 0
                self.g = 0
                self.f = self.g
            else:
                self.h = 0
                self.g = self.parent.g + 1
                self.f = self.g

    def expand(self):
        child_tiles = self.puzzle.possible_actions()
        children = []
        for i in child_tiles:  # create children nodes
            child = Node(i, self, heuristic=self.heuristic, goal=self.goal)
            if self.heuristic == 'manhattan':
                child.heuristic_fun()
            if self.heuristic == "displaced":
                child.heuristic_fun()
            if self.heuristic == 'ucs':
                child.heuristic_fun()
            children.append(child)
        return children

    def is_in_expanded(self, exp):
        # return if node is in the expanded nodes list
        tiles = self.tiles
        for node in exp:
            if (tiles == node.tiles) and (node.f <= self.f):
                return node
        return None


def findGoal(elem, goal):  # find the correct place of a tile in goal
    # return indexes of element elem in the goal state
    #tiles = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]
    #tiles = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, '_']]
    #tiles = [[1, '_'], [3, 2]]
    for i in range(len(goal)):
        for j in range(len(goal)):
            if goal[i][j] == elem:
                return i, j


def Astar():
    incr = 0
    frontier = PriorityQueue(1000000000000000000000)
    # print(initial_state.tiles)
    frontier.put((initial_state.f, initial_state))

    exp = []
    front_size = 0
    expanded_nodes = 0
    while (1):
        #print("frontiere debut iteration: \n")
        # for (i, j) in frontier.queue:
        #    print(j.tiles)
        #   print(i)
        if frontier.qsize() == 0:
            return None
        if frontier.qsize() > front_size:
            front_size = frontier.qsize()
        x = frontier.get()
        #print("element choisi: \n")
        # print(x[1].tiles)
        #print("heuristique de l'element choisi ", x[0])
        # print(frontier.get()[1].tiles)
        # x est un tuple (valeur huristique du node, node)
        # return and delete element that is going to be expanded from frontier
        if (x[1].isGoal()):
            print("FRONTIER SIZE")
            print(front_size)
            print("EXPANDED NODES: ", expanded_nodes)
            return x[1]
        if (x[1].is_in_expanded(exp) is None):
            exp.append(x[1])
            expanded_nodes += 1
            successors = x[1].expand()
            for i in successors:
                if i.is_in_expanded(exp) is None:
                    frontier.put((i.f, i))
                # print(frontier.queue)
            #print("frontiere fin iteration")
            # for (i, j) in frontier.queue:
              #  print(j.tiles)
    return None


def UCS():
    incr = 0
    frontier = PriorityQueue(1000000000000000000000)
    # print(initial_state.tiles)
    frontier.put((initial_state.f, initial_state))

    exp = []
    front_size = 0
    expanded_nodes = 0
    while (1):
        #print("frontiere debut iteration: \n")
        # for (i, j) in frontier.queue:
        #    print(j.tiles)
        #   print(i)
        if frontier.qsize() == 0:
            return None
        if frontier.qsize() > front_size:
            front_size = frontier.qsize()
        x = frontier.get()
        #print("element choisi: \n")
        # print(x[1].tiles)
        #print("heuristique de l'element choisi ", x[0])
        # print(frontier.get()[1].tiles)
        # x est un tuple (valeur huristique du node, node)
        # return and delete element that is going to be expanded from frontier
        if (x[1].isGoal()):
            print("FRONTIER SIZE")
            print(front_size)
            print("EXPANDED NODES: ", expanded_nodes)
            return x[1]
        # if node x[1] is not expanded
        if (x[1].is_in_expanded(exp) is None):
            exp.append(x[1])
            expanded_nodes += 1
            successors = x[1].expand()
            for child in successors:
                if child.is_in_expanded(exp) is None:
                    frontier.put((child.f, child))
                # print(frontier.queue)
            #print("frontiere fin iteration")
            # for (i, j) in frontier.queue:
              #  print(j.tiles)
    return None


def BFS():
    explored = []
    front_size = 0
    expanded_nodes = 0
    frontier = Queue(100000000000)
    frontier.put(initial_state)
    while frontier:
        if frontier.qsize() == 0:
            return None
        if frontier.qsize() > front_size:
            front_size = frontier.qsize()
        x = frontier.get()
        explored.append(x)
        expanded_nodes += 1
        # if (x.is_in_expanded(explored) is None):
        # explored.append(x)
        successors = x.expand()
        print(successors)
        for child in successors:
            print(child)
            if child.is_in_expanded(explored) is None:
                if (child.isGoal()):
                    print("FRONTIER SIZE")
                    print(front_size)
                    print("EXPANDED NODES: ", expanded_nodes)
                    return child
                frontier.put(child)
    return None


def print_state(state):
    for i in range(len(state)):
        print(state[i], "\n")
    print("\n")


def print_path(sol):
    global goal, n
    print("SUCCESS\n")
    path = []
    while (sol is not None):
        path.insert(0, sol.tiles)
        sol = sol.parent
        # print_state(sol.tiles)
        # print(sol.h)
    for state in path:
        print_state(state)
    print("PATH LENGTH", len(path))


def generate_random_puzzle(n):
    liste = list(range(n**2))
    random.shuffle(liste)
    tiles = [[liste[i + j*n] for i in range(n)] for j in range(n)]
    return tiles


def solve():
    global m, initial_state, goal, n
    #tiles = [[1, 2, 4], [3, 6, 5], [7, 8, '_']]
    #tiles = [[12, 1, 2, 15], [11, 6, 5, 8], [7, 10, 9, 4], ['_', 13, 14, 3]]
    #tiles = [[1, '_'], [3, 2]]
    #tiles = [[1, 3, 5], [4, '_', 2], [7, 8, 6]]
    choice = input("Generate a random puzzle (R) or use existing file (F): ")
    # put R or F aaccording to the choice
    if choice == 'R':
        size = input("Enter a size for the puzzle: ")
        s = int(size)
        tiles = generate_random_puzzle(int(sqrt(s+1)))
    if choice == 'F':
        f = input("Name of the text file containing the puzzle : ")
        txt = "./tests/" + f
        file = open(txt, 'r')
        tiles = np.loadtxt(file).astype(int).tolist()
    n = len(tiles)
    algo = input(
        "\nWhat algorithm do you want to use to solve the puzzle, A* (A) or UCS (U) or BFS(B): ")
    if algo == 'A':
        heuristic = input(
            "\nWhat heuristic do you want to use to solve the puzzle with A*? \n The manhattan distance heuristic ('manhattan') or the displaced tiles heuristic('displaced'): ")
    if algo == 'U':
        heuristic = 'ucs'
    if algo == 'B':
        heuristic = None

    m = 2000
    # input("What is the maximum number of iterations ? \n")
    goal = [list(range(1 + n * i, 1 + n * (i + 1)))for i in range(n)]
    goal[-1][-1] = 0
    #goal = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]
    #goal = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, '_']]
    #goal = [[1, 2], [3, '_']]
    initial_state = Node(tiles=tiles, heuristic=heuristic,
                         goal=goal, parent=None)

    puzzle = Puzzle(n, tiles)
    if not puzzle.is_solvable():
        print("THIS PUZZLE IS NOT SOLVABLE")
        solve()
    if puzzle.is_solvable():
        if algo == 'A':
            print("\nSOLVING USING A* WITH HEURISTIC ", heuristic, "\n")
            start_time = time.time()
            solution = Astar()
            end_time = time.time()
        if algo == 'U':
            print("SOLVING USING UCS \n")
            start_time = time.time()
            solution = UCS()
            end_time = time.time()
        if algo == 'B':
            print("\nSOLVING USING BFS \n")
            start_time = time.time()
            solution = BFS()
            end_time = time.time()

        print("INITIAL STATE")
        print_state(initial_state.tiles)
        # solution = IDAstar()

        if (solution is None):
            print("FAILURE")
        else:
            print("PATH")
            print_path(solution)
            print("TIME: ", end_time-start_time)


solve()
