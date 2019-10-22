from __future__ import print_function
import matplotlib.pyplot as plt
import arcl_youbot_planner.arm_planner.prmstar
import math
import numpy as np

class AStarGraph(object):
	#Define a class board like grid with two barriers
 
    def __init__(self, graph_vertex, graph_neighbor, graph_dict):
        self.graph_vertex = graph_vertex
        self.graph_neighbor = graph_neighbor
        self.graph_dict = graph_dict
	
    def heuristic(self, start, goal):
        start_arr = np.asarray(start)
        goal_arr = np.asarray(goal)

		#start, goal: numpy array with size (5, 1)
        temp = (start_arr - goal_arr)*(start_arr - goal_arr)
        return math.sqrt(temp.sum())
 
    def get_vertex_neighbours(self, pos):
        if pos in self.graph_dict:
            index = self.graph_dict[pos]
            print("current index:"+ str(index))
        else:
            print("not in the graph dict")
            return 
        neighbor_array = self.graph_neighbor[:, index]
        neighbor_index = np.where(neighbor_array == 1)
        neighbor_index = np.asarray(neighbor_index)
        print(neighbor_index)
        print(neighbor_index.shape)
        print(self.graph_vertex.shape)
        neighbor_index = neighbor_index.reshape([neighbor_index.shape[1]])
        neighbors_numpy_array = self.graph_vertex[:, neighbor_index]
        neighbors = []
        i = 0
        print(neighbors_numpy_array.shape)
        while i < neighbors_numpy_array.shape[1]:
            neighbors.append(tuple(neighbors_numpy_array[:,i]))
            i += 1
        return neighbors
    
    def move_cost(self, a, b):
        a = np.asarray(a)
        b = np.asarray(b)
        temp = (a - b)*(a - b)
        return math.sqrt(temp.sum())
 
def AStarSearch(start, end, graph):

    G = {} #Actual movement cost to each position from the start position
    F = {} #Estimated movement cost of start to end going via this position
 
	#Initialize starting values

    G[start] = 0 
    F[start] = graph.heuristic(start, end)

    closedVertices = set()
    openVertices = set([start])
    cameFrom = {}


    while len(openVertices) > 0:
        current = None
        currentFscore = None
        for pos in openVertices:
            if current is None or F[pos] < currentFscore:
				currentFscore = F[pos]
				current = pos
        print(openVertices)
        #Check if we have reached the goal
        if current == end:
            print(current)
            print(end)
        	#Retrace our route backward
            path = [current]
            while current in cameFrom:
                current = cameFrom[current]
                path.append(current)
            path.reverse()
            print("find the solution")
            print(path)
            print(len(path))
            #print(cameFrom)
            return path, F[end] #Done!

        openVertices.remove(current)
        closedVertices.add(current)

		#Update scores for vertices near the current position
        for neighbour in graph.get_vertex_neighbours(current):
            #print(current)
            #print(neighbour)
            if neighbour in closedVertices: 
				continue #We have already processed this node exhaustively
            candidateG = G[current] + graph.move_cost(current, neighbour)

            if neighbour not in openVertices:
				openVertices.add(neighbour) #Discovered a new vertex
            elif candidateG >= G[neighbour]:
				continue #This G score is worse than previously found
 
			#Adopt this G score
            cameFrom[neighbour] = current
            G[neighbour] = candidateG
            H = graph.heuristic(neighbour, end)
            F[neighbour] = G[neighbour] + H
    #print("G:")
    #print(G)
    #print("F")
    #print(F)
    #print(cameFrom)
    raise RuntimeError("A* failed to find a solution")
    
