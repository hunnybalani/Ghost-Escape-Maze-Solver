from Agent2 import *

N = 51  #matrix size
ghost_upper_bound = 200 #maximum no. of ghosts to test for
number_of_runs = 50 #no. of iterations

agent2 = plot_survivability_vs_ghosts_agent2(N, number_of_runs, ghost_upper_bound)  #Plot the probability of survival for Agent w.r.t no. of ghosts