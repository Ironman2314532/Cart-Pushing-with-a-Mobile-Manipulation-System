import pygame
import numpy as np
from scipy import spatial
from math import atan2,cos,sin,radians,sqrt

class Node:
    def __init__(self, x, y):
        self.x = x        # coordinate
        self.y = y        # coordinate
        self.parent = None    # parent node / edge
        self.cost = 0.0       # cost to parent / edge weight

class RRT_star:
    def __init__(self,obs,start,goal):
        self.obs=obs
        self.path=[]
        self.reduced_path=[]
        self.vertices=[]
        self.found=False
        self.row_size=1600
        self.col_size=800
        self.start=Node(start[0],start[1])
        self.goal = Node(goal[0], goal[1])

        self.vertices.append(self.start)
    

    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        return np.sqrt((node1.x-node2.x)**2 + (node1.y-node2.y)**2)
    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if there are obstacles
            False if the new node is valid to be connected
        '''
        # Check obstacle between nodes
        # get all the points in between
        points_between = zip(np.linspace(node1.x, node2.x, dtype=int), 
                             np.linspace(node1.y, node2.y, dtype=int))
        # check if any of these are obstacles
        for point in points_between:
            for ob in self.obs:
                if ob.collidepoint(point):
                    return True
        return False
    

    def get_new_point(self):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        # select goal
        
        
        point = [np.random.randint(0, self.row_size-1), np.random.randint(0, self.col_size-1)]
        return point
    
    def get_nearest_node(self, point):
        '''Find the nearest node from the new point in self.vertices
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        # Use kdtree to find the neighbors within neighbor size
        samples = [[v.x, v.y] for v in self.vertices]
        kdtree = spatial.cKDTree(samples)
        coord, ind = kdtree.query(point)
        return self.vertices[ind]
    
    def extend(self, new_point, extend_dis=50):
        '''Extend a new node to the current tree structure
        arguments:
            new_point - the new sampled point in the map
            extend_dis - extension distance for each step

        return:
            a new node if this node is valid and added, None if not.

        Extend towards the new point and check feasibility.
        Create and add a new node if feasible.
        '''
        # Get nearest node
        nearest_node = self.get_nearest_node(new_point)

        # Calculate new node location
        slope = np.arctan2(new_point[1]-nearest_node.y, new_point[0]-nearest_node.x)
        new_row = nearest_node.x + extend_dis*np.cos(slope)
        new_col = nearest_node.y + extend_dis*np.sin(slope)
        new_node = Node(int(new_row), int(new_col))

        # Check boundary and collision
        if (0 <= new_row < self.row_size) and (0 <= new_col < self.col_size) and \
           not self.check_collision(nearest_node, new_node):
            # If pass, add the new node
            new_node.parent = nearest_node
            new_node.cost = extend_dis
            self.vertices.append(new_node)

            # Check if goal is close
            if not self.found:
                d = self.dis(new_node, self.goal)
                if d < extend_dis:
                    self.goal.cost = d
                    self.goal.parent = new_node
                    self.vertices.append(self.goal)
                    self.found = True

            return new_node
        else:
            return None

    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that is within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        # Use kdtree to find the neighbors within neighbor size
        samples = [[v.x, v.y] for v in self.vertices]
        kdtree = spatial.cKDTree(samples)
        ind = kdtree.query_ball_point([new_node.x, new_node.y], neighbor_size)
        neighbors = [self.vertices[i] for i in ind]
        # Remove the new_node itself
        neighbors.remove(new_node)
        return neighbors

    def path_cost(self, start_node, end_node):
        '''Compute path cost starting from start node to end node
        arguments:
            start_node - path start node
            end_node - path end node

        return:
            cost - path cost
        '''
        cost = 0
        curr_node = end_node
        while start_node.x != curr_node.x or start_node.y != curr_node.y:
            # Keep tracing back until finding the start_node 
            # or no path exists
            parent = curr_node.parent
            if parent is None:
                print("Invalid Path")
                return 0
            cost += curr_node.cost
            curr_node = parent
        
        return cost
    
    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        # If no neighbors, skip
        if neighbors == []:
            return

        # Compute the distance from the new node to the neighbor nodes
        distances = [self.dis(new_node, node) for node in neighbors]

        # Rewire the new node
        # compute the least potential cost
        costs = [d + self.path_cost(self.start, neighbors[i]) for i, d in enumerate(distances)]
        indices = np.argsort(np.array(costs))
        # check collision and connect the best node to the new node
        for i in indices:
            if not self.check_collision(new_node, neighbors[i]):
                new_node.parent = neighbors[i]
                new_node.cost = distances[i]
                break

        # Rewire new_node's neighbors
        for i, node in enumerate(neighbors):
            # new cost
            new_cost = self.path_cost(self.start, new_node) + distances[i]
            # if new cost is lower
            # and there is no obstacles in between
            if self.path_cost(self.start, node) > new_cost and \
               not self.check_collision(node, new_node):
                node.parent = new_node
                node.cost = distances[i]

    def reduce_path(self):
        '''Reduce the path by removing nodes that are on the same line
        '''
        self.reduced_path.append((self.path[0].x, self.path[0].y))
        last_node=Node(self.reduced_path[-1][0],self.reduced_path[-1][1])
        for i in range(1, len(self.path)-1):
            if self.check_collision(last_node, self.path[i+1]):
                self.reduced_path.append((self.path[i].x, self.path[i].y))
                last_node=Node(self.reduced_path[-1][0],self.reduced_path[-1][1])
                if self.check_collision(self.path[i], self.path[-1])==False:
                    break
        self.reduced_path.append((self.path[-1].x, self.path[-1].y))

    def RRT(self, n_pts=5000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Start searching       
        for i in range(n_pts):
            # Extend a new node until all the points are sampled
            # or find the path
            new_point = self.get_new_point()
            new_node = self.extend(new_point, 50)
            if self.found:
                break

        # Output
        if self.found:
            cur = self.goal
            self.path.append(cur)
            while cur.y != self.start.y or cur.x != self.start.x:
                cur = cur.parent
                self.path.append(cur)
            self.reduce_path()
        if not self.found:
            print("No path found")
        

    def RRT_star(self, n_pts=1000, neighbor_size=100):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
    
        # Start searching       
        for i in range(n_pts):
            # Extend a new node
            new_point = self.get_new_point()
            new_node = self.extend(new_point, 50)
            # Rewire
            if new_node is not None:
                neighbors = self.get_neighbors(new_node, neighbor_size)
                self.rewire(new_node, neighbors)

        # Output
        if self.found:
            cur = self.goal
            self.path.append(cur)
            while cur.y != self.start.y or cur.x != self.start.x:
                cur = cur.parent
                self.path.append(cur)
            self.reduce_path()
        else:
            print("No path found")
