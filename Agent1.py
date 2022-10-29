import matplotlib.pyplot as plt

from Utility import *
from AStarAlgorithm import *


class Agent1:
    x_path = [0, 1, 0, -1]  # The four possible directions right, down, up, left
    y_path = [1, 0, -1, 0]

    def __init__(self, start_x, start_y, goal_x, goal_y, N, matrix):
        self.x = start_x  #start node
        self.y = start_y
        self.goal_x = goal_x  #goal node
        self.goal_y = goal_y
        self.size = N  #matrix size
        self.matrix = matrix  #matrix
        self.queue = SortedSet()  #Priority queue for nodes, hence Sorted Set is used
        self.childQueue = {}  # o keep a track of all nodes being added to priority queue along with its fn,gn,hn values
        self.map = {}  #To store the least distance of a cell from the start node
        self.parentMap = {}  #Maps Child -> Parent (with least distance from source node)

    def distance_from_goal(self, x1,y1):  #Function used to calculate distance of x1,y1 to the goal node by Manhattan Distance
        return abs(self.goal_x - x1) + abs(self.goal_y - y1)

    def distance_from_source(self, x1, y1, x1_parent,y1_parent):  #Function used to calculate the minimum distance from source node
        if self.map.get(tuple([x1, y1])) is None:  #Check if node is in the queue
            self.map[tuple([x1, y1])] = self.map.get(tuple([x1_parent, y1_parent])) + 1
            self.parentMap[(x1, y1)] = (x1_parent, y1_parent)
        else:  #If node is present in the queue, calculate distance from the previous parent (parent+1)
            #and distance without the previous parent, update with the smallest distance. Also update the parent if required
            distance_with_parent = self.map.get(tuple([x1_parent, y1_parent])) + 1
            distance_without_parent = self.map.get(tuple([x1, y1]))
            if distance_with_parent > distance_without_parent:
                self.map[tuple([x1, y1])] = distance_without_parent
            else:
                self.map[tuple([x1, y1])] = distance_with_parent
                self.parentMap[(x1, y1)] = (x1_parent, y1_parent)
        return self.map.get(tuple([x1, y1]))

    def a_star(self):  #Main AStar Algorithm to calculate if a path exists or not
        visited = [[0 for x in range(self.size)] for y in range(self.size)]  #Initialize a visited matrix, to keep a track of expanded node
        self.map[tuple([self.x, self.y])] = 0  #Adding source node to the map, and updating its gn value to 0
        self.queue.add((self.distance_from_goal(self.x, self.y), self.distance_from_goal(self.x, self.y), 0, self.x, self.y))  # Add the source to the queue, with fn,hn,gn

        while self.queue.__len__() > 0:  #Traverse all the nodes until queue is not empty
            curr_fn, curr_hn, curr_gn, curr_x, curr_y = self.queue.pop(0)  #Pop the node with priority in the order of fn,hn,gn

            if curr_x == self.goal_x and curr_y == self.goal_y:  #Check if the popped node is goal node
                return True  #If yes, goal node reached

            if visited[curr_x][curr_y] == 1:  #If the popped node is already visited, move to the next node
                continue

            visited[curr_x][curr_y] = 1  #Mark the current popped node as visited

            for i in range(0, 4):  #Traverse through the neighbours of the popped node
                temp_x = curr_x + Agent1.x_path[i]  #Append the direction value to get the neighbours
                temp_y = curr_y + Agent1.y_path[i]

                if 0 <= temp_x < self.size and 0 <= temp_y < self.size and visited[temp_x][temp_y] == 0 and self.matrix[temp_x][temp_y] == 0:  #Check if the neighbour is within the NxN matrix and is unblocked and not visited

                    gn = self.distance_from_source(temp_x, temp_y, curr_x,curr_y)  #Calculate shorted distance from source
                    hn = self.distance_from_goal(temp_x, temp_y)  #Calculate manhattan distance to goal node
                    fn = gn + hn  #Calculate total heuristic

                    if (temp_x,temp_y) in self.childQueue:  #If neighbour id already in the queue, remove the neighbour and update it with new fn,hn,gn values
                        fn_old, hn_old, gn_old = self.childQueue[(temp_x, temp_y)]
                        self.queue.remove((fn_old, hn_old, gn_old, temp_x, temp_y))

                    self.childQueue[(temp_x, temp_y)] = (fn, hn, gn)  #Keep track of nodes added along with their fn,gn,hn values
                    self.queue.add((fn, hn, gn, temp_x, temp_y))  #Add neighbours into priority queue

        return False  #If all nodes are traversed and goal node not reached, no path possible

    def get_path(self):  #Function to get the shortest path from source to goal node
        curr = (self.goal_x, self.goal_y)  #Start from the goal node
        count = 0
        path = []
        while curr != (self.x, self.y):  #Loop until source node
            count += 1
            path.append(curr)  #Add node to path
            curr = self.parentMap[curr]  #Get the parent of current node
        path.append((self.x, self.y))  #Append the source node
        path.reverse()  #Reverse the array to get path from source to goal
        return count, path  #return path and len(path)

    def solve_grid(self):  #Function to check whether a given matrix is solvable or not
        path = []
        count = 0
        solvable = self.a_star()  #Call AStar to check if a path is possible
        if solvable is True:
            count, path = self.get_path()  #If matrix is solvable we get the shortest path
        return solvable, count, path  #Return if solvable, len(path), path

    def move_ghosts(self, ghost_x, ghost_y):    #Function to move a ghost in a random direction
        ghost_direction = np.random.randint(0, 3)
        temp_ghost_x = ghost_x + Agent1.x_path[ghost_direction]
        temp_ghost_y = ghost_y + Agent1.y_path[ghost_direction]
        return temp_ghost_x, temp_ghost_y

    def solve_agent1(self, path, ghosts):   #Function to traverse as Agent1 in the Grid for a given path and no. of ghosts
        ghost_location = [0 for x in range(ghosts)] #Initialize ghost location by randomly generating (x,y)

        for i in range(ghosts):
            location = np.random.randint(0, self.size * self.size - 1)
            ghost_y = location % self.size
            ghost_x = int(location / self.size)
            ghost_location[i] = (ghost_x, ghost_y)  #Save the locations to check if the agent dies or not

        for i in range(len(path)):  #Check at each step, if the agent bumped into a ghost cell
            for g in range(ghosts):
                (ghost_x, ghost_y) = ghost_location[g]
                if (ghost_x, ghost_y) == path[i]:
                    print('Agent1 is DEAD!')
                    return False

            if i == len(path) - 1:  #If you reached the last step, the agent survives
                print('Agent1 SURVIVED!')
                return True

            for g in range(ghosts): #After moving the agent, move all the ghosts based on given conditions
                (ghost_x, ghost_y) = ghost_location[g]
                (temp_ghost_x, temp_ghost_y) = self.move_ghosts(ghost_x, ghost_y)   #Move ghost in a random direction using function

                while temp_ghost_x < 0 or temp_ghost_x >= self.size or temp_ghost_y < 0 or temp_ghost_y >= self.size:   #Keep a check that the ghost does not move out of the NxN grid
                    (temp_ghost_x, temp_ghost_y) = self.move_ghosts(ghost_x, ghost_y)

                if self.matrix[temp_ghost_x][temp_ghost_y] == 1:    #If the next step of ghost is in a blocked cell, move the ghost with a probability of 0.5
                    (temp_ghost_x, temp_ghost_y) = (temp_ghost_x, temp_ghost_y) if np.random.random() <= 0.5 else (ghost_x, ghost_y)
                ghost_location[g] = (temp_ghost_x, temp_ghost_y)    #Update the ghost locations

                if (temp_ghost_x, temp_ghost_y) == path[i]: #Check at each ghost step, if the ghost bumped into the agent cell
                    print('Agent1 is DEAD!')
                    return False
        print('Agent1 SURVIVED!')    #If traversed through the entire path, the agent survives
        return True


def plot_survivability_vs_ghosts_agent1(N, number_of_runs, ghost_upper_bound):  #Function to plot the graph of probability of survival vs ghosts for Agent1
    survived = [0 for x in range(ghost_upper_bound + 1)]    #Initialize a matrix to keep a track of survival
    for iter in range(1, number_of_runs + 1):
        matrix = Matrix.matrix_generator(N) #For each iteration, generate a matrix and check if agent survived for ghosts = 0 to 200
        is_solvable = Agent1(0, 0, N - 1, N - 1, N, matrix).solve_grid()    #Check if the matrix is solvable
        print('solvable: ', is_solvable)
        if is_solvable[0] is True:  #If yes, then traverse agent1 along the given solution path
            for i in range(ghost_upper_bound + 1):
                ghosts = i
                print('ghosts: ', ghosts)
                survived[i] += Agent1(0, 0, N - 1, N - 1, N, matrix).solve_agent1(is_solvable[2], ghosts)   #Count the number of times agent survived
    print(survived)

    for i in range(len(survived)):
        survived[i] = survived[i] / number_of_runs  #Get the probability by no. of times survived/no. of runs
    number_of_ghosts = [i for i in range(ghost_upper_bound + 1)]
    plt.ylim(0, number_of_runs / number_of_runs)
    plt.xlim(0, ghost_upper_bound)
    plt.xticks(np.arange(0, ghost_upper_bound, 5))
    plt.title('AGENT 1 SURVIVABILITY', pad=30, fontweight="bold")
    plt.xlabel('NUMBER OF GHOSTS', labelpad=30, fontweight="bold")
    plt.ylabel('PROBABILITY OF SURVIVAL', labelpad=30, fontweight="bold")
    plt.rcParams["figure.figsize"] = (75, 30)
    plt.rcParams['font.size'] = 30
    plt.plot(number_of_ghosts, survived, color='crimson', linewidth=2)
    plt.show()

    print('Minimum:', min(survived), 'Minimum when ghosts:', survived.index(min(survived))) #Check for which ghost the probability was minimum
    print('Maximum:', max(survived), 'Maximum when ghosts:', survived.index(max(survived))) #Check for which ghost the probability was maximum
