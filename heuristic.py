import numpy as np
import copy
from queue import PriorityQueue


class Node:
    def __init__(self, tiles=np.empty([]), parent=None, g=0, h=0, f=0, heuristic='displaced', goal=np.empty([])):
        self.tiles = tiles
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
        goal_state = np.array([i+1 for i in range(n**2)]).reshape(n, n)
        goal_state[-1][-1] = 0
        self.goal = goal_state

    def isGoal(self):
        t = np.concatenate(self.tiles).flat
        return t == np.concatenate(self.goal).flat

    def findBlank(self):  # find the blank in a node's tiles
        tiles = self.tiles
        for i in range(self.n):
            for j in range(self.n):
                if tiles[i][j] == 0:
                    return i, j

    def heuristic(self):
        # misplaced tiles heuristic
        # state is the current state
        # n is the size of the puzzle
        if self.heuristic == 'displaced':
            state_list = np.array()
            tiles = self.tiles
            self.def_goal()  # est ce necessaire???
            goal_state = self.goal.flatten()
            print("GOAL STATE: ", goal_state)
            misplaced = 0
            for i in range(self.n):
                state_list = np.concatenate([state_list, self.tiles[i]])
            print("STATE LIST/ ", state_list)
            for i in range(self.n**2):
                if state_list[i] != goal_state[i] and state_list[i] != 0:
                    misplaced += 1
            if self == initial_state:
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
                        x, y = findGoal(tiles[i][j], self.goal)
                        sum += abs(x - i) + abs(y - j)
            if self == initial_state:
                self.h = sum
                self.g = 0
                self.f = sum
            else:
                self.h = sum
                self.g = self.parent.g + 1
                self.f = self.g + sum

    def genChildren(self):
        tiles = self.tiles
        x, y = self.findBlank()
        newTiles = []
        if (x + 1) < self.n:  # moving blank down / moving a tile up
            new = copy.deepcopy(tiles)
            new[x][y] = new[x+1][y]
            new[x+1][y] = 0
            newTiles.append(new)
        if (x - 1) > -1:  # moving blank up / moving a tile down
            new = copy.deepcopy(tiles)
            new[x][y] = new[x-1][y]
            new[x-1][y] = 0
            newTiles.append(new)
        if (y + 1) < self.n:  # moving blank right / moving a tile left
            new = copy.deepcopy(tiles)
            new[x][y] = new[x][y+1]
            new[x][y+1] = 0
            newTiles.append(new)
        if (y - 1) > -1:  # moving blank left / moving a tile right
            new = copy.deepcopy(tiles)
            new[x][y] = new[x][y - 1]
            new[x][y-1] = 0
            newTiles.append(new)
        ret = []
        for i in newTiles:  # create children nodes
            print(newTiles)
            print("i:::")
            print(i)
            child = Node(self, i)
            if self.heuristic == 'manhattan':
                child.heuristic()
            if self.heuristic == "displaced":
                child.heuristic()
            ret.append(child)
        return ret

    def isExp(self, exp):
        # return if node is in the expanded nodes list
        tiles = self.tiles
        for i in exp:
            if (tiles == i.tiles) and (i.f <= self.f):
                return i
        return None


def getInput():
    global method, m, n, goal, start
    method = raw_input()
    m = int(raw_input())
    n = int(raw_input())
    tiles = []
    for i in range(n):
        tiles.append(raw_input().split(" "))
    start.tiles = tiles
    tiles = []
    for i in range(n):
        tiles.append(raw_input().split(" "))
    goal.tiles = tiles
    start.hgf()


def findOpt(lst):
    # in the frontier list return the optimal node and its index in the frontier
    opt = lst[0]
    index = 0
    for i in range(len(lst)):  # type: Node
        if lst[i].f < opt.f:
            opt = lst[i]
            index = i
    return opt, index


def findGoal(str, goal_state):  # find the correct place of a tile in goal
    # return indexes of element str in the goal state
    tiles = goal_state
    for i in range(n):
        for j in range(n):
            if tiles[i][j] == str:
                return i, j


def Astar():
    frontier = PriorityQueue(10000)
    print(initial_state.tiles)
    frontier.put((initial_state.h, initial_state))
    print("FRONTIER: ", frontier.queue)
    print(frontier.qsize())
    exp = []
    while (1):
        print("frontiere debut iteration: ")
        for (i, j) in frontier.queue:
            print(j.tiles)
        if frontier.qsize() == 0:
            return None

        x = frontier.get()
        print("element choisi")
        print(x[1].tiles)
        print(x[0])
        # x est un tuple (valeur huristique du node, node)
        # return and delete element that is going to be expanded from frontier
        if (x[1].isGoal().all()):

            return x[1]

        if (x[1].isExp(exp) is None):
            exp.append(x[1])
            successors = x[1].genChildren()
            for i in successors:
                if i.isExp(exp) is None:
                    frontier.put((i.h, i))
                # print(frontier.queue)
            print("frontiere fin iteration")
            for (i, j) in frontier.queue:
                print(j.tiles)

    return None


'''
def IDAstar():
    fmax = start.f
    while(1):
        if(fmax > m):
            return None
        node, newfmax = limitedSearch(start, fmax)
        if(node!=None):
            return node
        if(newfmax!=None):
            fmax = newfmax
    return None


def limitedSearch(node, limit):
    if node.f > limit:
        return None, node.f
    if node.isGoal():
        return node, limit
    successors = node.genChildren()
    min = float('inf')
    for succ in successors:
        newNode, newLimit = limitedSearch(succ, limit)
        if newNode!=None:
            return newNode, None
        if newLimit < min:
            min = newLimit
    return None, min
 '''


def printOutput(sol):
    global goal, n
    print("SUCCESS\n")
    path = []
    while (sol is not None):
        path.insert(0, sol)
        sol = sol.parent
    for i in range(len(path)):
        state = path[i]
        for j in range(n):
            for k in range(n):
                if (k < n-1):
                    print(state.tiles)[j][k],
                else:
                    print(state.tiles)[j][k]
        print("\n")


def solve():
    global m, initial_state, goal, n
    tiles = [[12, 1, 2, 15], [11, 6, 5, 8], [7, 10, 9, 4], [0, 13, 14, 3]]
    n = len(tiles)
    # input("What puzzle do you want to solve (in list of tiles form) ? \n")
    heuristic = 'displaced'
    # input("Which heuristic : manhattan or dipslaced ? \n")

    m = 2000
    # input("What is the maximum number of iterations ? \n")
    goal = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 0]]
    initial_state = Node(tiles=tiles, heuristic=heuristic, h=2,
                         goal=goal, parent=None)

    # initial_state.def_goal()
    # solution = None
    print(initial_state)
    solution = Astar()
    print(solution)
    # solution = IDAstar()

    if (solution is None):
        print("FAILURE")
    else:
        printOutput(solution)


solve()
