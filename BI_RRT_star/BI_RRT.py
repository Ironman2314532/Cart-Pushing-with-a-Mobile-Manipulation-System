import pygame
import numpy as np
from scipy import spatial
from math import atan2,cos,sin,radians,sqrt,degrees

class Node:
    def __init__(self, x, y):
        self.x = x        # coordinate
        self.y = y        # coordinate
        self.parent = None    # parent node / edge
        self.cost = 0.0       # cost to parent / edge weight

class BI_RRT_star:
    def __init__(self,obs,start,goal):
        self.obs=[]
        self.path=[]
        self.reduced_path=[]
        self.final_path=[]
        self.tree1_vertices=[]
        self.tree2_vertices=[]
        self.found=False
        self.row_size=1600
        self.col_size=800
        self.start=Node(start[0],start[1])
        self.goal = Node(goal[0], goal[1])
        self.start_pose=start
        self.goal_pose=goal

        self.tree1_vertices.append(self.start)
        self.tree2_vertices.append(self.goal)

        self.tree1_goal=None
        self.tree2_goal=None

        for ob in obs:
            self.obs.append(ob.inflate(30,30))

        self.best_path_cost=None
        self.best_path_cost_tree1=None
        self.best_path_cost_tree2=None
        self.best_dis_cost=None

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
    
    def get_new_point(self,tree_number):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        # select goal
        
        if self.found==False:
            point = [np.random.randint(0, self.row_size-1), np.random.randint(0, self.col_size-1)]
        
        else:
            point=self.get_new_point_in_ellipsoid(tree_number)
        return point
    
    def get_nearest_node(self, point,tree_number):
        '''Find the nearest node from the new point in self.vertices
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        # Use kdtree to find the neighbors within neighbor size
        if tree_number==1:
            samples = [[v.x, v.y] for v in self.tree1_vertices]
            kdtree = spatial.cKDTree(samples)
            coord, ind = kdtree.query(point)
            return self.tree1_vertices[ind]
        else:
            samples = [[v.x, v.y] for v in self.tree2_vertices]
            kdtree = spatial.cKDTree(samples)
            coord, ind = kdtree.query(point)
            return self.tree2_vertices[ind]
        
    def extend(self, new_point, extend_dis=50,tree_number=1):
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
        nearest_node = self.get_nearest_node(new_point,tree_number)

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
            if tree_number==1:
                self.tree1_vertices.append(new_node)
            else:
                self.tree2_vertices.append(new_node)

            

            return new_node
        else:
            return None
    
    def get_neighbors(self, new_node, neighbor_size,tree_number):
        '''Get the neighbors that is within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        # Use kdtree to find the neighbors within neighbor size
        if tree_number==1:
            samples = [[v.x, v.y] for v in self.tree1_vertices]
            kdtree = spatial.cKDTree(samples)
            ind = kdtree.query_ball_point([new_node.x, new_node.y], neighbor_size)
            neighbors = [self.tree1_vertices[i] for i in ind]
            # Remove the new_node itself
            neighbors.remove(new_node)
            return neighbors

        else:
            samples = [[v.x, v.y] for v in self.tree2_vertices]
            kdtree = spatial.cKDTree(samples)
            ind = kdtree.query_ball_point([new_node.x, new_node.y], neighbor_size)
            neighbors = [self.tree2_vertices[i] for i in ind]
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


    def get_new_point_in_ellipsoid(self, tree_number):
        '''Choose the goal or generate a random point in an ellipsoid
           defined by start, goal and current best length of path
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point
            c_best - the length of the current best path

        return:
            point - the new point
        '''
        
        
        #### TODO ####
        # Generate a random point in an ellipsoid
        
            # Compute the distance between start and goal - c_min
        if tree_number==1:
            c_best=self.best_path_cost_tree1+self.best_dis_cost
            c_min=self.dis(self.start,self.tree2_goal)
            start_node=self.start
            goal_node=self.tree2_goal
        else:
            c_best=self.best_path_cost_tree2+self.best_dis_cost
            c_min=self.dis(self.goal,self.tree1_goal)
            start_node=self.goal
            goal_node=self.tree1_goal
        # Calculate center of the ellipsoid - x_center
        x_center=Node((goal_node.x+start_node.x)/2,(goal_node.y+start_node.y)/2)
        # Compute rotation matrix from elipse to world frame - C
        theta=atan2((goal_node.x-start_node.x),(goal_node.y-start_node.y))
        R=np.array([[cos(theta),sin(theta),0],[-sin(theta),cos(theta),0],[0,0,1]])
        # Compute diagonal matrix - L
        L = np.diag([ (sqrt(c_best**2 - c_min**2))/2, c_best/2,c_best/2])
        # Cast a sample from a unit ball - x_ball
        r=np.random.random(1)
        angle=np.random.randint(0,360)
        x=r*cos(radians(angle))
        y=r*sin(radians(angle))
        x_ball=np.array([x[0],y[0],0]).T
        # Map ball sample to the ellipsoid - x_rand
        x_rand = np.matmul(R, np.matmul(L, x_ball)) + np.array([x_center.x, x_center.y, 0]).T
        point = [x_rand[0], x_rand[1]]
        #### TODO END ####

        return point

    def rewire(self, new_node, neighbors,tree_number):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''

        if tree_number==1:
            start_node=self.start
        else:
            start_node=self.goal
        # If no neighbors, skip
        if neighbors == []:
            return

        # Compute the distance from the new node to the neighbor nodes
        distances = [self.dis(new_node, node) for node in neighbors]

        # Rewire the new node
        # compute the least potential cost
        costs = [d + self.path_cost(start_node, neighbors[i]) for i, d in enumerate(distances)]
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
            new_cost = self.path_cost(start_node, new_node) + distances[i]
            # if new cost is lower
            # and there is no obstacles in between
            if self.path_cost(start_node, node) > new_cost and \
               not self.check_collision(node, new_node):
                node.parent = new_node
                node.cost = distances[i]


    def reduce_path(self):
        '''Reduce the path by removing nodes that are on the same line
        '''
        #Uncommenr for shorter path
        #Comment for Safe path
        # obs=self.obs
        # self.obs=[]
        # for ob in obs:
        #    self.obs.append(ob.inflate(-30,-30))
        self.reduced_path.append((self.path[0].x, self.path[0].y))
        last_node=Node(self.reduced_path[-1][0],self.reduced_path[-1][1])
        for i in range(1, len(self.path)-1):
            if self.check_collision(last_node, self.path[i+1]):
                self.reduced_path.append((self.path[i].x, self.path[i].y))
                last_node=Node(self.reduced_path[-1][0],self.reduced_path[-1][1])
                if self.check_collision(self.path[i], self.path[-1])==False:
                    break
        self.reduced_path.append((self.path[-1].x, self.path[-1].y))

        




    def check_if_trees_connect(self,node,tree_number=1):
        '''Check if the node is within the tree2 nodes
        arguments:
            node - the node to be checked
            tree2_nodes - the tree2 nodes

        return:
            True if the node is within the tree2 nodes, False if not
        '''
        # Use kdtree to find the neighbors within neighbor size
        
        nearest_node = self.get_nearest_node([node.x, node.y],tree_number)
        if self.check_collision(nearest_node, node)==False:
            return nearest_node
        else:
            return None
    
    def compute_final_path(self):
        '''Compute the final path
        '''
        self.final_path.append(self.start_pose)
        for i in range(1,len(self.reduced_path)-1):
            angle=-atan2((self.reduced_path[i+1][1]-self.reduced_path[i][1]),(self.reduced_path[i+1][0]-self.reduced_path[i][0]))
            self.final_path.append((self.reduced_path[i][0],self.reduced_path[i][1],degrees(angle),0))
        self.final_path.append(self.goal_pose)

    def BIRRT_star(self, n_pts=10000, neighbor_size=100):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        n_pts_changed=False
        # Start searching
        tree_tracker=0
        i=0      
        while i <n_pts:
            i=i+1
            # Extend a new node
            if tree_tracker%2==0:
                new_point = self.get_new_point(1)
                new_node = self.extend(new_point, 50,1)
                tree_tracker+=1
                # Rewire
                if new_node is not None:
                    neighbors = self.get_neighbors(new_node, neighbor_size,1)
                    self.rewire(new_node, neighbors,1)
                    
                    other_tree_node=self.check_if_trees_connect(new_node,2)
                    if other_tree_node is not None:
                        tree_1_cost=self.path_cost(self.start,new_node)
                        tree_2_cost=self.path_cost(self.goal,other_tree_node)
                        dis_cost=self.dis(new_node,other_tree_node)
                        if self.best_path_cost is None or tree_1_cost+tree_2_cost+dis_cost<self.best_path_cost:
                            self.tree1_goal=new_node
                            self.tree2_goal=other_tree_node
                            self.best_path_cost=tree_1_cost+tree_2_cost+dis_cost
                            self.best_path_cost_tree1=tree_1_cost
                            self.best_path_cost_tree2=tree_2_cost
                            self.best_dis_cost=dis_cost
                            self.found=True
                            if n_pts_changed==False:
                                n_pts=i+2000
                                n_pts_changed=True
            else:
                new_point = self.get_new_point(2)
                new_node = self.extend(new_point, 50,2)
                tree_tracker+=1
                # Rewire
                if new_node is not None:
                    neighbors = self.get_neighbors(new_node, neighbor_size,2)
                    self.rewire(new_node, neighbors,2)
                    
                    other_tree_node=self.check_if_trees_connect(new_node,1)
                    if other_tree_node is not None:
                        tree_1_cost=self.path_cost(self.start,other_tree_node)
                        tree_2_cost=self.path_cost(self.goal,new_node)
                        dis_cost=self.dis(new_node,other_tree_node)
                        if self.best_path_cost is None or tree_1_cost+tree_2_cost+dis_cost<self.best_path_cost:
                            self.tree1_goal=other_tree_node
                            self.tree2_goal=new_node
                            self.best_path_cost=tree_1_cost+tree_2_cost+dis_cost
                            self.best_path_cost_tree1=tree_1_cost
                            self.best_path_cost_tree2=tree_2_cost
                            self.best_dis_cost=dis_cost
                            self.found=True
                            if n_pts_changed==False:
                                n_pts=i+2000
                                n_pts_changed=True
            

        # Output
        if self.found:
            cur = self.tree1_goal
            self.path.append(cur)
            while cur.y != self.start.y or cur.x != self.start.x:
                cur = cur.parent
                self.path.append(cur)
            self.path.reverse()
            cur = self.tree2_goal
            self.path.append(cur)
            while cur.y != self.goal.y or cur.x != self.goal.x:
                cur = cur.parent
                self.path.append(cur)
            self.reduce_path()
            self.compute_final_path()
            
        else:
            print("No path found")
