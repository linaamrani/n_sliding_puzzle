

def find_empty(puzzle):
    for i in range(puzzle.n):
        for j in range(puzzle.n):
            if puzzle.initial_state[i][j] == '_':
                return (i, j)


class NPuzzle():
    def __init__(self, initial_state, goal_state):
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.n = len(self.initial_state)
        self.empty_tile = find_empty(self)
