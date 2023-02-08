import numpy as np
import copy
from queue import PriorityQueue


class Node:
    def __init__(self, tiles=[], parent=None, g=0, h=0, f=0, heuristic='manhattan', goal=[]):
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
        goal_state = [[1, 2], [3, '_']]
        # goal_state.append('_')
        self.goal = goal_state

    def isGoal(self):
        t = [elem for sublist in self.tiles for elem in sublist]
        return t == self.goal

    def findBlank(self):  # find the blank in a node's tiles
        tiles = self.tiles
        for i in range(self.n):
            for j in range(self.n):
                if tiles[i][j] == '_':
                    return i, j

    def displaced_tiles_heuristic(self):
        # misplaced tiles heuristic
        # state is the current state
        # n is the size of the puzzle
        state_list = []
        tiles = self.tiles
        self.def_goal()  # est ce necessaire???
        goal_state = self.goal
        print("GOAL STATE: ", goal_state)
        misplaced = 0
        for i in range(self.n):
            state_list = np.concatenate([state_list, self.tiles[i]])
        for i in range(self.n):
            if state_list[i] != goal_state[i]:
                misplaced += 1
        if self == initial_state:
            self.h = misplaced
            self.g = 0
            self.f = misplaced
        else:
            self.h = misplaced
            self.g = self.parent.g + 1
            self.f = self.g + misplaced

    def manhattan_distance_heuristic(self):  # calculate the manhattan distance
        sum = 0
        tiles = self.tiles
        n = len(tiles)

        for i in range(n):
            for j in range(n):
                if tiles[i][j] == '_':
                    continue
                else:
                    x, y = findGoal(tiles[i][j])
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
            new[x+1][y] = '_'
            newTiles.append(new)
        if (x - 1) > -1:  # moving blank up / moving a tile down
            new = copy.deepcopy(tiles)
            new[x][y] = new[x-1][y]
            new[x-1][y] = '_'
            newTiles.append(new)
        if (y + 1) < self.n:  # moving blank right / moving a tile left
            new = copy.deepcopy(tiles)
            new[x][y] = new[x][y+1]
            new[x][y+1] = '_'
            newTiles.append(new)
        if (y - 1) > -1:  # moving blank left / moving a tile right
            new = copy.deepcopy(tiles)
            new[x][y] = new[x][y - 1]
            new[x][y-1] = '_'
            newTiles.append(new)
        ret = []
        for i in newTiles:  # create children nodes
            child = Node(i, self)
            if self.heuristic == 'manhattan':
                child.manhattan_distance_heuristic()
            if self.heuristic == "displaced":
                child.displaced_tiles_heuristic()
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


def findGoal(str):  # find the correct place of a tile in goal
    # return indexes of element str in the goal state
    global goal
    tiles = [[1, 2], [3, '_']]
    for i in range(n):
        for j in range(n):
            if tiles[i][j] == str:
                return i, j


def Astar():
    front = [initial_state]
    exp = []
    while(1):
        if len(front)==0:
            return None

        x, xIndex = findOpt(front)
        del front[xIndex]

        if(x.isGoal() and x.f<=m):
            return x

        if(x.isExp(exp) == None and x.f<=m):
            exp.append(x)
            successors = x.genChildren()
            for i in successors:
                front.append(i)

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
    while (sol != None):
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
    tiles = [[2, 1], [3, '_']]
    n = len(tiles)
    #input("What puzzle do you want to solve (in list of tiles form) ? \n")
    heuristic = 'manhattan'
    #input("Which heuristic : manhattan or dipslaced ? \n")

    m = 2000
    #input("What is the maximum number of iterations ? \n")
    goal = [[1, 2], [3, '_']]
    initial_state = Node(tiles=tiles, heuristic=heuristic,
                         goal=goal, parent=None)

    # initial_state.def_goal()
    #solution = None
    solution = Astar()

    #solution = IDAstar()

    if (solution == None):
        print("FAILURE")
    else:
        printOutput(solution)


solve()
