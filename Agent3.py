import matplotlib.pyplot as plt

from Utility import *
from AStarAlgorithm import *


class Agent3:
    N = 51

    x_path = [0, 1, 0, -1]  # The four possible directions right, down, up, left
    y_path = [1, 0, -1, 0]

    x_dir = [0, 1, 0, 0, -1]  # The four possible directions right, down, center, up, left
    y_dir = [1, 0, 0, -1, 0]

    weight = [1, 1, 0.8, 0.7, 0.7]  #Weights in all 5 mentioned directions

    def __init__(self, start_x, start_y, goal_x, goal_y, N, matrix,  ghosts=np.zeros((N, N), dtype=int)):
        self.x = start_x  #start node
        self.y = start_y
        self.goal_x = goal_x  #goal node
        self.goal_y = goal_y
        self.size = N  #matrix size
        self.matrix = matrix  #matrix
        self.discoveredGhosts = ghosts  #
        self.queue = SortedSet()  #Priority queue for nodes, hence Sorted Set is used
        self.childQueue = {}  #To keep a track of all nodes being added to priority queue along with its fn,gn,hn values
        self.map = {}  #To store the least distance of a cell from the start node
        self.parentMap = {}  #Maps Child -> Parent (with least distance from source node)

    def distance_from_goal(self, x1, y1):   #Function used to calculate distance of x1,y1 to the goal node by Manhattan Distance
        return abs(self.goal_x - x1) + abs(self.goal_y - y1)

    def distance_from_source(self, x1, y1, x1_parent, y1_parent):   #Function used to calculate the minimum distance from source node
        if self.map.get(tuple([x1, y1])) is None:   #Check if node is in the queue
            self.map[tuple([x1, y1])] = self.map.get(tuple([x1_parent, y1_parent])) + 1
            self.parentMap[(x1, y1)] = (x1_parent, y1_parent)
        else:   #If node is present in the queue, calculate distance from the previous parent (parent+1)
                #and distance without the previous parent, update with the smallest distance. Also update the parent if required
            distance_with_parent = self.map.get(tuple([x1_parent, y1_parent])) + 1
            distance_without_parent = self.map.get(tuple([x1, y1]))
            if distance_with_parent > distance_without_parent:
                self.map[tuple([x1, y1])] = distance_without_parent
            else:
                self.map[tuple([x1, y1])] = distance_with_parent
                self.parentMap[(x1, y1)] = (x1_parent, y1_parent)
        return self.map.get(tuple([x1, y1]))

    def a_star(self): #Main AStar Algorithm to calculate if a path exists or not
        visited = [[0 for x in range(self.size)] for y in range(self.size)] #Initialize a visited matrix, to keep a track of expanded node
        self.map[tuple([self.x, self.y])] = 0   #Adding source node to the map, and updating its gn value to 0
        self.queue.add((self.distance_from_goal(self.x, self.y), self.distance_from_goal(self.x, self.y), 0, self.x, self.y))   #Add the source to the queue, with fn,hn,gn

        while self.queue.__len__() > 0: #Traverse all the nodes until queue is not empty
            curr_fn, curr_hn, curr_gn, curr_x, curr_y = self.queue.pop(0)   #Pop the node with priority in the order of fn,hn,gn

            if curr_x == self.goal_x and curr_y == self.goal_y: #Check if the popped node is goal node
                return True #If yes, goal node reached

            if visited[curr_x][curr_y] == 1:    #If the popped node is already visited, move to the next node
                continue

            visited[curr_x][curr_y] = 1 #Mark the current popped node as visited

            for i in range(0, 4):   #Traverse through the neighbours of the popped node
                temp_x = curr_x + Agent3.x_path[i] #Append the direction value to get the neighbours
                temp_y = curr_y + Agent3.y_path[i]

                if 0 <= temp_x < self.size and 0 <= temp_y < self.size and visited[temp_x][temp_y] == 0 and self.matrix[temp_x][temp_y] == 0  and self.discoveredGhosts[temp_x][temp_y] == 0:   #Check if the neighbour is within the NxN matrix and is unblocked and not visited and does not have a ghost

                    gn = self.distance_from_source(temp_x, temp_y, curr_x, curr_y)  # Calculate shorted distance from source
                    hn = self.distance_from_goal(temp_x, temp_y)  # Calculate manhattan distance to goal node
                    fn = gn + hn  # Calculate total heuristic

                    if (temp_x, temp_y) in self.childQueue: #If neighbour id already in the queue, remove the neighbour and update it with new fn,hn,gn values
                        fn_old, hn_old, gn_old = self.childQueue[(temp_x, temp_y)]
                        self.queue.remove((fn_old, hn_old, gn_old, temp_x, temp_y))

                    self.childQueue[(temp_x, temp_y)] = (fn, hn, gn)    #Keep track of nodes added along with their fn,gn,hn values
                    self.queue.add((fn, hn, gn, temp_x, temp_y))    #Add neighbours into priority queue

        return False   #If all nodes are traversed and goal node not reached, no path possible

    def get_path(self): #Function to get the shortest path from source to goal node
        curr = (self.goal_x, self.goal_y)   #Start from the goal node
        count = 0
        path = []
        while curr != (self.x, self.y): #Loop until source node
            count += 1
            path.append(curr)   #Add node to path
            curr = self.parentMap[curr] #Get the parent of current node
        path.append((self.x, self.y))   #Append the source node
        path.reverse()  #Reverse the array to get path from source to goal
        return count, path  #return path and len(path)

    def solve_grid(self):   #Function to check whether a given matrix is solvable or not
        path = []
        count = 0
        solvable = self.a_star()        #Call AStar to check if a path is possible
        if solvable is True:
            count, path = self.get_path()   #If matrix is solvable we get the shortest path
        return solvable, count, path    #Return if solvable, len(path), path

    def move_ghosts(self, ghost_x, ghost_y):    #Function to move a ghost in a random direction
        ghost_direction = np.random.randint(0, 3)
        temp_ghost_x = ghost_x + Agent3.x_path[ghost_direction]
        temp_ghost_y = ghost_y + Agent3.y_path[ghost_direction]
        return temp_ghost_x, temp_ghost_y

    def move_agent(self, x, y): #Function to move the agent such that we don't land on a ghost cell
        for dir in range(4):
            temp_x = x + Agent3.x_path[dir]
            temp_y = y + Agent3.y_path[dir]
            if temp_x < 0 or temp_x >= self.size or temp_y < 0 or temp_y >= self.size or self.matrix[temp_x][temp_y] == 1 or self.discoveredGhosts[temp_x][temp_y] == 1:    #Check if the cell is in the grid and not landing on a ghost
                continue
            else:
                return temp_x, temp_y
        return False

    def solve_agent2(self, path, ghosts):   #Function to traverse as Agent2 in the Grid for a given path and no. of ghosts
        ghost_location = [0 for x in range(ghosts)]  #Initialize ghost location by randomly generating (x,y)

        for i in range(ghosts):
            location = np.random.randint(0, self.size * self.size - 1)
            ghost_y = location % self.size
            ghost_x = int(location / self.size)
            ghost_location[i] = (ghost_x, ghost_y)   #Save the locations to check if the agent dies or not

        i = 0
        while i < len(path):    #Traverse along the given path
            self.discoveredGhosts = np.zeros((self.size, self.size), dtype=int)

            for g in range(ghosts):  #Check at each step, if the agent bumped into a ghost cell
                (ghost_x, ghost_y) = ghost_location[g]
                if (ghost_x, ghost_y) == path[i]:
                    return False, 0

            if path[i] == (self.goal_x, self.goal_y):   #If goal node reached, agent survives
                return True, len(path)

            for g in range(ghosts): #After moving the agent, move all the ghosts based on given conditions
                (ghost_x, ghost_y) = ghost_location[g]

                (temp_ghost_x, temp_ghost_y) = self.move_ghosts(ghost_x, ghost_y)   #Move ghost in a random direction using function

                while temp_ghost_x < 0 or temp_ghost_x >= self.size or temp_ghost_y < 0 or temp_ghost_y >= self.size:   #Keep a check that the ghost does not move out of the NxN grid
                    (temp_ghost_x, temp_ghost_y) = self.move_ghosts(ghost_x, ghost_y)

                if self.matrix[temp_ghost_x][temp_ghost_y] == 1:    #If the next step of ghost is in a blocked cell, move the ghost with a probability of 0.5
                    (temp_ghost_x, temp_ghost_y) = (temp_ghost_x, temp_ghost_y) if np.random.random() <= 0.5 else (ghost_x, ghost_y)

                ghost_location[g] = (temp_ghost_x, temp_ghost_y)   #Update the ghost locations
                self.discoveredGhosts[temp_ghost_x][temp_ghost_y] = 1   #Update the discovered grid for Agent3

                if (temp_ghost_x, temp_ghost_y) == path[i]: #Check at each ghost step, if the ghost bumped into the agent cell
                    return False, 0

            if i == len(path) - 1:  #If you reach at the last step of path, and it is not goal, reconstruct the path taking this as source
                (start_x, start_y) = path[i]
                path.extend(Agent3(start_x, start_y, self.goal_x, self.goal_y, self.size, self.matrix).solve_grid()[2][1:])

            for g in range(ghosts): #Check for all ghosts if the next step of the path if occupied by a ghost
                if path[i + 1] == ghost_location[g]:
                    (start_x, start_y) = path[i]    #If yes, Agent3 replans the path taking the current node as source node
                    agent2_replan = Agent3(start_x, start_y, self.goal_x, self.goal_y, self.size, self.matrix, self.discoveredGhosts).solve_grid()
                    is_solvable = agent2_replan[0]  #If no solution possible at the moment, plan only the next step
                    if is_solvable is False:
                        next_step = self.move_agent(start_x, start_y)   #Move the agent using the function, such that it does not land on ghost
                        if next_step is False:  #If no such step possible Agent3 dies
                            return False, 0
                        else:   #Else append the path with the next step
                            path = path[:i + 1]
                            path.append(next_step)
                    else:   #If solution is possible change the path from the next step
                        path[i + 1:] = agent2_replan[2]

            i += 1
        return True, len(path)

    def solve_agent3(self, path, ghosts):   #Function to traverse as Agent3 in the Grid for a given path and no. of ghosts
        ghost_location = [0 for x in range(ghosts)]  #Initialize ghost location by randomly generating (x,y)

        for i in range(ghosts):
            location = np.random.randint(0, self.size * self.size - 1)
            ghost_y = location % self.size
            ghost_x = int(location / self.size)
            ghost_location[i] = (ghost_x, ghost_y)   #Save the locations to check if the agent dies or not

        i = 0
        while i < len(path):    #Traverse along the given path
            self.discoveredGhosts = np.zeros((self.size, self.size), dtype=int)

            for g in range(ghosts):  #Check at each step, if the agent bumped into a ghost cell
                (ghost_x, ghost_y) = ghost_location[g]
                if (ghost_x, ghost_y) == path[i]:
                    print('Agent3 is DEAD!')
                    return False, 0

            if path[i] == (self.goal_x, self.goal_y):      #If goal node reached, agent survives
                print('Agent3 SURVIVED!')
                return True, len(path)

            for g in range(ghosts): #After moving the agent, move all the ghosts based on given conditions
                (ghost_x, ghost_y) = ghost_location[g]

                (temp_ghost_x, temp_ghost_y) = self.move_ghosts(ghost_x, ghost_y)   #Move ghost in a random direction using function

                while temp_ghost_x < 0 or temp_ghost_x >= self.size or temp_ghost_y < 0 or temp_ghost_y >= self.size:  #Keep a check that the ghost does not move out of the NxN grid
                    (temp_ghost_x, temp_ghost_y) = self.move_ghosts(ghost_x, ghost_y)

                if self.matrix[temp_ghost_x][temp_ghost_y] == 1:    #If the next step of ghost is in a blocked cell, move the ghost with a probability of 0.5
                    (temp_ghost_x, temp_ghost_y) = (temp_ghost_x, temp_ghost_y) if np.random.random() <= 0.5 else (ghost_x, ghost_y)

                ghost_location[g] = (temp_ghost_x, temp_ghost_y)   #Update the ghost locations
                self.discoveredGhosts[temp_ghost_x][temp_ghost_y] = 1  #Update the discovered grid for Agent2

                if (temp_ghost_x, temp_ghost_y) == path[i]: #Check at each ghost step, if the ghost bumped into the agent cell
                    print('Agent3 is DEAD!')
                    return False, 0

            survivability = [0 for s in range(5)]   #Initialize the survivability array
            for dir in range(0, 5): #Traverse in all possible 5 directions
                (start_x, start_y) = path[i]
                start_x += Agent3.x_dir[dir]
                start_y += Agent3.y_dir[dir]

                if 0 <= start_x < self.size and 0 <= start_y < self.size and self.matrix[start_x][start_y] == 1:    #Check if the new step is within the grid NxN
                    survivability[dir] = 0  #If the matrix is blocked in that direction assign survivability = 0
                    continue

                if 0 <= start_x < self.size and 0 <= start_y < self.size:    #Check if the new step is within the grid NxN
                    for n in range(20): #Iterate 20 times in each direction and calculate if the agent survives
                        is_solvable = Agent3(start_x, start_y, self.goal_x, self.goal_y, self.size, self.matrix).solve_grid()
                        if is_solvable[0] is True:
                            solve_agent2 = Agent3(start_x, start_y, self.goal_x, self.goal_y, self.size, self.matrix, self.discoveredGhosts).solve_agent2(is_solvable[2], ghosts)
                            survivability[dir] += solve_agent2[0]

            # print('path:', path[i])
            # print('survive:',survivability)
            optimum_survivability = max(survivability)  #Find the maximum survival
            optimum_path_survivability = [0 for o in range(5)]
            possible_directions = 0
            for dir in range(0, 5):
                if survivability[dir] == optimum_survivability:
                    optimum_path_survivability[dir] = 1 #Assign 1 to the direction with max survival
                    possible_directions += 1    #Count the number of maximum survivals

            if possible_directions > 1: #If there is a tie between survivals, assign weights and find the best direction
                best_path = [0 for o in range(5)]
                for dir in range(0, 5):
                    if optimum_path_survivability[dir] == 1:
                        best_path[dir] = survivability[dir] * Agent3.weight[dir]    #Assign weights sich that down = right > center > left = up

                # print('best:',best_path)
                best_dir = best_path.index(max(best_path))  #Find the index of the best direction
                # print('best dir:', best_dir)
                (start_x, start_y) = path[i]
                start_x += Agent3.x_dir[best_dir]
                start_y += Agent3.y_dir[best_dir]
                path[i + 1:] = Agent3(start_x, start_y, self.goal_x, self.goal_y, self.size, self.matrix).solve_grid()[2]   #Replan the path taking the new direction
            else:
                best_dir = optimum_path_survivability.index(1)  #if there is no tie, just take the index of max survival
                (start_x, start_y) = path[i]
                start_x += Agent3.x_dir[best_dir]
                start_y += Agent3.y_dir[best_dir]
                path[i + 1:] = Agent3(start_x, start_y, self.goal_x, self.goal_y, self.size, self.matrix).solve_grid()[2]  #Replan the path taking the new direction

            i += 1
        print('Agent3 SURVIVED!')  #If all the path is traversed, agent survives
        return True, len(path)


def plot_survivability_vs_ghosts_agent3(N, number_of_runs, ghost_upper_bound):  #Function to plot the graph of probability of survival vs ghosts for Agent4
    survived = [0 for x in range(ghost_upper_bound + 1)]   #Initialize a matrix to keep a track of survival
    for iter in range(1, number_of_runs + 1):
        for i in range(5, ghost_upper_bound + 1, 5): #For each iteration, generate a matrix and check if agent survived for ghosts = 0 to 200
            ghosts = i
            print('ghosts: ', ghosts)
            matrix = Matrix.matrix_generator(N)
            is_solvable = Agent3(0, 0, N - 1, N - 1, N, matrix).solve_grid()
            print('solvable: ', is_solvable)
            if is_solvable[0] is True:   #Check if the matrix is solvable
                    survived[i] += Agent3(0, 0, N - 1, N - 1, N, matrix).solve_agent3(is_solvable[2], ghosts)[0]  #If yes, then traverse agent3 along the given solution path
    print(survived)

    for i in range(len(survived)):
        survived[i] = survived[i] / number_of_runs #Get the probability by no. of times survived/no. of runs
    number_of_ghosts = [i for i in range(ghost_upper_bound + 1)]
    plt.ylim(0, number_of_runs / number_of_runs)
    plt.xlim(0, ghost_upper_bound)
    plt.xticks(np.arange(0, ghost_upper_bound, 5))
    plt.title('AGENT 3 SURVIVABILITY', pad=30, fontweight="bold")
    plt.xlabel('NUMBER OF GHOSTS', labelpad=30, fontweight="bold")
    plt.ylabel('PROBABILITY OF SURVIVAL', labelpad=30, fontweight="bold")
    plt.rcParams["figure.figsize"] = (75, 30)
    plt.rcParams['font.size'] = 30
    plt.plot(number_of_ghosts, survived, color='crimson', linewidth=2)
    plt.show()

    print('Minimum:', min(survived), 'Minimum when ghosts:', survived.index(min(survived))) #Check for which ghost the probability was minimum
    print('Maximum:', max(survived), 'Maximum when ghosts:', survived.index(max(survived))) #Check for which ghost the probability was maximum
