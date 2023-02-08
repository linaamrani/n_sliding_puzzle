import copy


class Puzzle:
    def __init__(self, size, tiles):
        self.n = size
        self.tiles = tiles
        self.empty_tile = self.find_empty_tile()

    def find_empty_tile(self):
        # to find the position (indexes) of the empty tile
        for i in range(self.n):
            for j in range(self.n):
                if self.tiles[i][j] == 0:
                    return i, j
        return None

    def is_solvable(self):
        # return True if the puzzle is solvable
        # because some puzzle are nots
        # a puzzle is solvable if
        # if its size is odd, then its solvable if the number of inversions is even
        # if its size is even; its solvable if empty tile is on an even row counting from bottom
        # and the number of inversions is odd
        # or if empty tile is on an odd row counting from bottom and the number of inversions is even
        list_tiles = [elem for sublist in self.tiles for elem in sublist]
        inversions = 0
        # compute number of inversions
        for i in range(len(list_tiles)):
            for j in range(i, len(list_tiles)):
                if list_tiles[i] > list_tiles[j] and list_tiles[j] != 0:
                    inversions += 1
        size = self.n
        if size % 2 != 0:
            if inversions % 2 == 0:
                return True
            else:
                return False
        else:
            zero_pos = size - self.empty_tile[0]
            print("zero pos", zero_pos)
            print("inv_count", inversions)
            if zero_pos % 2 == 0:
                if inversions % 2 != 0:
                    return True
                return False
            else:
                if inversions % 2 == 0:
                    return True
                return False

    def possible_actions(self):
        tiles = self.tiles
        i, j = self.find_empty_tile()
        actions = []
        if (i - 1) > -1:  # moving empty tile up = moving a tile down
            new_tiles = copy.deepcopy(tiles)
            new_tiles[i][j] = new_tiles[i-1][j]
            new_tiles[i-1][j] = 0
            actions.append(new_tiles)
        if (i + 1) < self.n:  # moving empty tile down = moving a tile up
            # creates a copy of the current node
            new_tiles = copy.deepcopy(tiles)
            new_tiles[i][j] = new_tiles[i+1][j]
            new_tiles[i+1][j] = 0
            actions.append(new_tiles)
        if (j - 1) > -1:  # moving empty tile left = moving a tile right
            new_tiles = copy.deepcopy(tiles)
            new_tiles[i][j] = new_tiles[i][j - 1]
            new_tiles[i][j-1] = 0
            actions.append(new_tiles)
        if (j + 1) < self.n:  # moving empty tile right = moving a tile left
            new_tiles = copy.deepcopy(tiles)
            new_tiles[i][j] = new_tiles[i][j+1]
            new_tiles[i][j+1] = 0
            actions.append(new_tiles)
        return actions


#p = Puzzle(3, [[1, 2, 3], [4, 5, 0], [7, 8, 6]])
# print(p.is_solvable())
# print(p.find_empty_tile())
