import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx
from enum import Enum
from queue import PriorityQueue
import numpy as np
from scipy.spatial import Voronoi
from bresenham import bresenham
import numpy.linalg as LA

def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min + 1)))
    east_size = int(np.ceil((east_max - east_min + 1)))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Initialize an empty list for Voronoi points
    points = []

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
            
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)
    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        # Then you can test each pair p1 and p2 for collision using Bresenham
        # (need to convert to integer if using prebuilt Python package)
        edge_points = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        in_collision = False
        for ep in edge_points:
                if (ep[0] < 0) or (ep[1] < 0) or ep[0] >= grid.shape[0] or ep[1] >= grid.shape[1]:
                    in_collision = True
                    break
                #check collision
                if grid[ep[0], ep[1]] == 1:
                    in_collision = True
                    break

        # If the edge does not hit an obstacle
        # add it to the list
        if not in_collision:
            edges.append(((p1[0], p1[1]), (p2[0], p2[1])))

    return grid, edges, int(north_min), int(east_min)

def create_graph(edges):
    G = nx.Graph()
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        dist = LA.norm(np.array(p2) - np.array(p1))
        G.add_edge(p1, p2, weight=dist)
    return G

def closest_point(graph, point):
    current_point = (point[0], point[1])
    closest_point = None
    min_dist = 100000
    for n in graph.nodes:
        dist = LA.norm(np.array(n) - np.array(current_point))
        if dist < min_dist:
            closest_point = n
            min_dist = dist
    return closest_point

def a_star(graph, heuristic, start, goal):
    
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            found = True
            break
            
        else:
            for node in graph[current_node]:
                cost = graph.edges[current_node, node]['weight']
                new_cost = current_cost + cost + heuristic(node, goal)

                if node not in visited:                
                    visited.add(node)               
                    queue.put((new_cost, node))
                    
                    branch[node] = (new_cost, current_node)
             
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

def prune_path(path):
    pruned_path = [p for p in path]
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i+=1

    return pruned_path

def collinearity_check(p1, p2, p3, epsilon=1e-1): 
    collinear = False
    # TODO: Add a third dimension of z=1 to each point
    p1_3d = np.append(p1, 1) 
    p2_3d = np.append(p2, 1)
    p3_3d = np.append(p3, 1)
    # TODO: Create the matrix out of three points
    mat = np.vstack((p1_3d, p2_3d, p3_3d))
    # TODO: Calculate the determinant of the matrix. 
    det = np.linalg.det(mat)
    # TODO: Set collinear to True if the determinant is less than epsilon
    if np.abs(det) < epsilon:
        collinear = True
    return collinear

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))
