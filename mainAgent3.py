from Agent3 import *

N = 51  #matrix size
ghost_upper_bound = 200 #maximum no. of ghosts to test for
number_of_runs = 50 #no. of iterations

agent3 = plot_survivability_vs_ghosts_agent3(N, number_of_runs, ghost_upper_bound)  #Plot the probability of survival for Agent3 w.r.t no. of ghosts

# matrix = Matrix.matrix_generator(N)
# ghosts = 1
# is_solvable = Agent3(0, 0, N - 1, N - 1, N, matrix).solve_grid()
# print('solvable: ', is_solvable)
# if is_solvable[0] is True:
#     agent3 = Agent3(0, 0, N - 1, N - 1, N, matrix).solve_agent3(is_solvable[2], ghosts)