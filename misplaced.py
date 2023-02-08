import numpy as np
tiles = [['_', 1], [2, 3]]


def displaced_tiles_heuristic(tiles):
    # misplaced tiles heuristic
    # state is the current state
    # n is the size of the puzzle
    state_list = []
    goal = [[1, 2], [3, '_']]  # est ce necessaire???
    goal_state = [elem for sublist in goal for elem in sublist]
    print("GOAL STATE: ", goal_state)
    misplaced = 0
    for i in range(len(tiles)):
        state_list = state_list+tiles[i]
    print("STATE LIST/ ", state_list)
    for i in range(len(tiles)**2):
        if state_list[i] != goal_state[i] and state_list[i] != 0:
            misplaced += 1
    return misplaced


print(displaced_tiles_heuristic(tiles))

n = 4
array = np.array([i+1 for i in range(n**2)]).reshape(n, n)
print(array)
print(array[1][1])

array[-1][-1] = 0
print(array)

'''
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
'''
