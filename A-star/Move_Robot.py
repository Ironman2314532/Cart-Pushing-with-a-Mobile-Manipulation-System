import pygame, sys
from pygame.locals import *
import random
from math import sin,cos,radians,tan,floor
import time


from Robot import Robot
from Cart import Cart
from utils import blit_rotate_center
from A_star import A_star
from maps import hospital_map_1

pygame.init()
FPS = 60
clock = pygame.time.Clock()
 
# Predefined some colors
BLUE  = (0, 0, 255)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
 
# Screen information
SCREEN_WIDTH =1600
SCREEN_HEIGHT = 800

#env=pygame.transform.scale(pygame.image.load('environment.png'),(1600,800))
DISPLAYSURF = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))
DISPLAYSURF.fill(WHITE)
pygame.display.set_caption("Game")

#Load map
map=hospital_map_1()
obs=map.obs
dyobs=map.dyobs

robot=Robot()
robot.initial_config(map.start)
cart=Cart()

start=map.start
goals=map.goals

curr_dyobs=[]
for j in range(len(goals)):
    #A-star
    a_star=A_star(obs,robot,cart,start,goals[j])
    a_star.generate_path()
    print(len(a_star.path))
    # print(a_star.path)
    # input("Press Enter to continue...")


    #Simulation
    old_time=time.time()
    start_time=old_time
    while True:
        clock.tick(FPS)
        DISPLAYSURF.fill(WHITE)
        for i in obs:
            pygame.draw.rect(DISPLAYSURF,(0,0,0),i)
        for i in goals:
            pygame.draw.circle(DISPLAYSURF,(122,122,122),(i[0],i[1]),25)
        pygame.draw.circle(DISPLAYSURF,(255,0,0),(goals[j][0],goals[j][1]),25)
        cart_rect=blit_rotate_center(DISPLAYSURF,cart.img,robot.get_cart_center(cart.cart_length),robot.cart_angle+robot.angle)
        robot_rect=blit_rotate_center(DISPLAYSURF,robot.img,robot.get_center(),robot.angle) 
        for i in a_star.path:
            pygame.draw.circle(DISPLAYSURF,(0,255,0),(i[0],i[1]),3)
        
        keys = pygame.key.get_pressed()
    
 
        if keys[pygame.K_1]:
            if dyobs[0] not in curr_dyobs:
                curr_dyobs.append(dyobs[0])
        if keys[pygame.K_2]:
            if dyobs[0]  in curr_dyobs:
                curr_dyobs.remove(dyobs[0])
            if dyobs[0]  in obs:
                curr_dyobs.remove(obs[0])

        if keys[pygame.K_3]:
            if dyobs[1] not in curr_dyobs:
                curr_dyobs.append(dyobs[1])
        if keys[pygame.K_4]:
            if dyobs[1]  in curr_dyobs:
                curr_dyobs.remove(dyobs[1])
            if dyobs[1]  in obs:
                curr_dyobs.remove(obs[1])
        if keys[pygame.K_5]:
            if dyobs[2] not in curr_dyobs:
                curr_dyobs.append(dyobs[2])
        if keys[pygame.K_6]:
            if dyobs[2]  in curr_dyobs:
                curr_dyobs.remove(dyobs[2])
            if dyobs[2]  in obs:
                curr_dyobs.remove(obs[2])
        for i in curr_dyobs:
            pygame.draw.rect(DISPLAYSURF,(0,0,0),i)

        pygame.display.update()
        for event in pygame.event.get():              
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        
        t=time.time()
        del_t=t-old_time
        i=floor((t-start_time)/a_star.step_time)
        try:
            robot.left_wheel_vel=a_star.path[i][4]
            robot.right_wheel_vel=a_star.path[i][5]
            robot.cart_angle=a_star.path[i][3]
        except:
            robot.left_wheel_vel=0
            robot.right_wheel_vel=0

        robot.move(del_t)
        old_time=t
        if i==len(a_star.path):
            start=[robot.x,robot.y,robot.angle%360,robot.cart_angle]
            #start=[goals[j][0],goals[j][1],goals[j][2],goals[j][3]]
            #robot.x=goals[j][0]
            #robot.y=goals[j][1]
            #robot.angle=goals[j][2]
            #robot.cart_angle=goals[j][3]
            break
        try:
            if a_star.check_dyna_coll(curr_dyobs,a_star.path[i]) or a_star.check_dyna_coll(curr_dyobs,a_star.path[i+1]):
                robot.left_wheel_vel=0
                robot.right_wheel_vel=0
                start=[robot.x,robot.y,robot.angle%360,robot.cart_angle]
                for k in curr_dyobs:
                    obs.append(k)
                a_star=A_star(obs,robot,cart,start,goals[j])
                a_star.generate_path()
                print(len(a_star.path))
                old_time=time.time()
                start_time=old_time
        
        except:
            pass


    
