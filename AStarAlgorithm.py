from sortedcontainers import SortedSet


class AStarSearch:
    x_path = [0, 1, 0, -1]  #The four possible directions right, down, up, left
    y_path = [1, 0, -1, 0]

    def __init__(self, start_x, start_y, goal_x, goal_y, N, matrix):
        self.x = start_x    #start node
        self.y = start_y
        self.goal_x = goal_x    #goal node
        self.goal_y = goal_y
        self.size = N   #matrix size
        self.matrix = matrix    #matrix
        self.queue = SortedSet()    #Priority queue for nodes, hence Sorted Set is used
        self.childQueue = {}    #To keep a track of all nodes being added to priority queue along with its fn,gn,hn values
        self.map = {}   #To store the least distance of a cell from the start node
        self.parentMap = {} #Maps Child -> Parent (with least distance from source node)

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
                temp_x = curr_x + AStarSearch.x_path[i] #Append the direction value to get the neighbours
                temp_y = curr_y + AStarSearch.y_path[i]

                if 0 <= temp_x < self.size and 0 <= temp_y < self.size and visited[temp_x][temp_y] == 0 and self.matrix[temp_x][temp_y] == 0:   #Check if the neighbour is within the NxN matrix and is unblocked and not visited

                    gn = self.distance_from_source(temp_x, temp_y, curr_x, curr_y)  #Calculate shorted distance from source
                    hn = self.distance_from_goal(temp_x, temp_y)    #Calculate manhattan distance to goal node
                    fn = gn + hn    #Calculate total heuristic

                    if (temp_x, temp_y) in self.childQueue: #If neighbour id already in the queue, remove the neighbour and update it with new fn,hn,gn values
                        fn_old, hn_old, gn_old = self.childQueue[(temp_x, temp_y)]
                        self.queue.remove((fn_old, hn_old, gn_old, temp_x, temp_y))

                    self.childQueue[(temp_x, temp_y)] = (fn, hn, gn)    #Keep track of nodes added along with their fn,gn,hn values
                    self.queue.add((fn, hn, gn, temp_x, temp_y))    #Add neighbours into priority queue

        return False    #If all nodes are traversed and goal node not reached, no path possible

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
