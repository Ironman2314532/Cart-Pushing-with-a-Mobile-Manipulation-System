import networkx as nx
import pygame
from math import cos,sin,radians,tan,sqrt

class Node:
    def __init__(self,x,y,angle,cart_angle,left_wheel_vel,right_wheel_vel,path,g,h,f):
        self.x=x
        self.y=y
        self.angle=angle
        self.cart_angle=cart_angle
        self.left_wheel_vel=left_wheel_vel
        self.right_wheel_vel=right_wheel_vel
        self.g=g
        self.h=h
        self.f=f
        self.path=path
        

class A_star:
    def __init__(self,obs,robot,cart,start,goal):
        self.obs=obs
        self.robot=robot
        self.cart=cart
        self.start=start
        self.goal=goal
        self.path=[]
        self.nodes_to_visit=[]
        self.visited=[]

        self.dt=0.002
        self.step_time=2
        self.steps=int(self.step_time/self.dt)

    def move_sim(self,x,y,angle,left_wheel_vel,right_wheel_vel,del_time):
        x+=(self.robot.wheelradius/2)*(left_wheel_vel+right_wheel_vel)*cos(radians(angle))*del_time
        y-=(self.robot.wheelradius/2)*(left_wheel_vel+right_wheel_vel)*sin(radians(angle))*del_time
        angle+=(self.robot.wheelradius/self.robot.track)*(right_wheel_vel-left_wheel_vel)*del_time
        return x,y,angle
    
    def get_center(self,x,y,angle):
        x=x+(self.robot.center_to_axel*cos(radians(angle)))
        y=y-(self.robot.center_to_axel*sin(radians(angle)))
        return x,y

    
    def check_collison(self,center,angle,image):
        rotated_image = pygame.transform.rotate(image, angle)
        new_rect = rotated_image.get_rect(center=center)
        collison=False
        for ob in self.obs:
            if new_rect.colliderect(ob):
                collison=True
                break
        return collison
    
    def get_cart_center(self,x,y,angle,cart_angle):
        x,y=self.get_center(x,y,angle)
        x=x+self.robot.center_to_cart*cos(radians(angle))+(self.cart.cart_length/2)*cos(radians(angle+cart_angle))
        y=y-self.robot.center_to_cart*sin(radians(angle))-(self.cart.cart_length/2)*sin(radians(angle+cart_angle))
        return x,y
    
    def heuristic(self,x,y):
        return (sqrt((x-self.goal[0])**2+(y-self.goal[1])**2)/(self.robot.max_wheel_vel*self.robot.wheelradius)) 

    
    def get_children(self,node):       
        for del_right_wheel_vel in [0,3,-3]:
            for del_left_wheel_vel in [0,3,-3]:
                for del_cart_angle in [0,10,-10]:
                    x_=node.x
                    y_=node.y
                    angle_=node.angle
                    left_wheel_vel_=node.left_wheel_vel+del_left_wheel_vel
                    right_wheel_vel_=node.right_wheel_vel+del_right_wheel_vel
                    
                    collison=False
                    cart_angle_=node.cart_angle+del_cart_angle
                    if left_wheel_vel_>=self.robot.max_wheel_vel or left_wheel_vel_<=-self.robot.max_wheel_vel or right_wheel_vel_>=self.robot.max_wheel_vel or right_wheel_vel_<=-self.robot.max_wheel_vel or cart_angle_>=45 or cart_angle_<=-45:
                        continue
                    for t in range(self.steps):
                        x_,y_,angle_= self.move_sim(x_,y_,angle_,left_wheel_vel_,right_wheel_vel_,self.dt)
                        if t%100==0:
                            collison = self.check_collison(self.get_center(x_,y_,angle_),angle_,self.robot.img)
                            if collison==True:
                                break
                            collison = self.check_collison(self.get_cart_center(x_,y_,angle_,cart_angle_),angle_+cart_angle_,self.cart.img)
                            if collison==True:
                                break
                    if collison==True:
                        continue
                    visited=False
                    for i in range(-2,2):
                        for j in range(-2,2):
                            for k in range(0,1):
                                if (round(x_,0)+i,round(y_,0)+j,round(angle_%360,0)+k) in self.visited:
                                    visited=True
                                    break
                            if visited==True:
                                break
                        if visited==True:
                           break

                    #if (round(x_,0),round(y_,0),angle_%360) in self.visited:
                    #    visited=True

                    if visited==False and x_>=0 and y_>=0 and x_<1600 and y_<800:
                        path_n=node.path.copy()
                        path_n.append([x_,y_,angle_,cart_angle_,left_wheel_vel_,right_wheel_vel_])
                        g_n=node.g+self.step_time
                        h_n=self.heuristic(x_,y_)
                        f_n=g_n+h_n
                        node_n=Node(x_,y_,angle_,cart_angle_,left_wheel_vel_,right_wheel_vel_,path_n,g_n,h_n,f_n)
                        self.nodes_to_visit.append(node_n)
                        self.visited.append(((round(x_,0),round(y_,0),round(angle_%360,0))))
                        

                    
    def get_next_node(self):
        min_f=None
        min_node=None
        for node in self.nodes_to_visit:
            if min_f==None or node.f<min_f:
                min_f=node.f
                min_node=node
                self.nodes_to_visit.remove(min_node)
    
        return min_node                    

    def generate_path(self):
        h=self.heuristic(self.start[0],self.start[1])
        node=Node(self.start[0],self.start[1],self.start[2],self.start[3],0,0,[(self.start[0],self.start[1],self.start[2],self.start[3],0,0)],0,h,h)
        self.nodes_to_visit.append(node)
        self.visited.append((round(self.start[0],0),round(self.start[1],0),self.start[2]))
        found_goal=False
        while len(self.nodes_to_visit)>0 and found_goal==False:
            node=self.get_next_node()
            if sqrt((self.goal[0]-node.x)**2+(self.goal[1]-node.y)**2)<25 and abs(self.goal[2]-(node.angle%360))<10 and abs(self.goal[3]-(node.cart_angle%360))<10:
                found_goal=True
                self.path=node.path
                break
            self.get_children(node)

    def check_dyna_coll(self,dy_obs,point):
        x=point[0]
        y=point[1]
        angle=point[2]
        image=self.robot.img
        center=self.get_center(x,y,angle)
        rotated_image = pygame.transform.rotate(image, angle)
        new_rect = rotated_image.get_rect(center=center)
        collison=False
        for ob in dy_obs:
            if new_rect.colliderect(ob):
                collison=True
                break
        cart_angle=point[3]
        center=self.get_cart_center(x,y,angle,cart_angle)
        rotated_image = pygame.transform.rotate(self.cart.img, angle+cart_angle)
        new_rect = rotated_image.get_rect(center=center)
        for ob in dy_obs:
            if new_rect.colliderect(ob):
                collison=True
                break
        return collison
           
        

