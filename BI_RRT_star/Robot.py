import pygame, sys
from pygame.locals import *
from math import sin,cos,radians,tan,floor

class Robot:
    def __init__(self):
        #robot parameters
        self.wheelradius=5
        self.width=25
        self.length=25
        self.center_to_cart=20
        self.center_to_axel=10
        self.track=2.5

        #configuration parameters
        self.x=0
        self.y=0
        self.angle=0
        self.cart_angle=0

        #Control parameters
        self.left_wheel_vel=0
        self.right_wheel_vel=0

        #Limiting parameters
        self.max_wheel_vel=10
        self.max_cart_angle=45
        
        #Image
        self.img=pygame.transform.scale(pygame.image.load('robot.png'),(self.length,self.width))
        
    #Kinematic model
    def move(self,del_time):
        self.x+=(self.wheelradius/2)*(self.left_wheel_vel+self.right_wheel_vel)*cos(radians(self.angle))*del_time
        self.y-=(self.wheelradius/2)*(self.left_wheel_vel+self.right_wheel_vel)*sin(radians(self.angle))*del_time
        self.angle+=(self.wheelradius/self.track)*(self.right_wheel_vel-self.left_wheel_vel)*del_time
    #Get center of robot
    def get_center(self):
        x=self.x+self.center_to_axel*cos(radians(self.angle))
        y=self.y-self.center_to_axel*sin(radians(self.angle))
        return x,y
    #Get center of cart
    def get_cart_center(self,cart_length):
        x,y=self.get_center()
        x=x+self.center_to_cart*cos(radians(self.angle))+(cart_length/2)*cos(radians(self.angle+self.cart_angle))
        y=y-self.center_to_cart*sin(radians(self.angle))-(cart_length/2)*sin(radians(self.angle+self.cart_angle))
        return x,y
    
    def initial_config(self,start):
        self.x=start[0]
        self.y=start[1]
        self.angle=start[2]
        self.cart_angle=start[3]

