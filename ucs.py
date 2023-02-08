import heuristic2
from heuristic2 import Node
from queue import PriorityQueue


def UCS(puzzle):
    initial_state = Node(tiles=puzzle.initial_state,
                         goal=puzzle.goal_state, parent=None)
    frontier = PriorityQueue(100000)
    explored = set()
    frontier.put(initial_state.tiles)
    while (1):
        if frontier.empty():
            return "FAILURE"
        node = frontier.get()
        if node.isGoal():
            return node
        explored.add(node)
        successors = node.genChildren()
        for a in successors:
            child
