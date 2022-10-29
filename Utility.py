import numpy as np


class Matrix:   #Matrix generator for generating NxN matrix with probability <= 0.28 as blocked (1) or unblocked (0)
    def matrix_generator(N):
        grid = np.zeros((N, N), dtype=int)
        for r in np.arange(0, N):
            for c in np.arange(0, N):
                grid[r][c] = 1 if np.random.random() <= 0.28 else 0 #Probability to decide whether the cell should be blocked or not
        grid[0][0] = 0  #Assigning top-left and bottom-right as unblocked
        grid[N - 1][N - 1] = 0
        return grid
