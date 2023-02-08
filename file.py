import numpy as np
#txt = input("Name of the text file containing the puzzle :")
#f1 = open(txt, 'r')
#tiles = np.loadtxt(f1).astype(int).tolist()
# print(tiles)
import random

goal = [list(range(1 + 3 * i, 1 + 3 * (i + 1)))
        for i in range(3)]
goal[-1][-1] = 0
print(goal)


def generate_random_puzzle(n):
    liste = list(range(n**2))
    print(liste)
    random.shuffle(liste)
    tiles = [[liste[i + j*n] for i in range(n)] for j in range(n)]
    return tiles


print(generate_random_puzzle(3))
