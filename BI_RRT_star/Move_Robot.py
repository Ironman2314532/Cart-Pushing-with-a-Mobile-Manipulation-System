import pygame, sys
from pygame.locals import *
import random
from math import sin,cos,radians,tan,floor
import time
from multiprocessing import Process
import multiprocessing

from Robot import Robot
from Cart import Cart
from utils import blit_rotate_center,distance
from ADA_star import ADA_star
from A_star import A_star
from BI_RRT import BI_RRT_star
from maps import hospital_map_1


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



def f(a_start,a_goal,left_wheel_vel,right_wheel_vel,return_path):
    print(a_goal)
    a_star=A_star(obs,robot,cart,a_start,a_goal,left_wheel_vel,right_wheel_vel)
    a_star.generate_path()
    print(len(a_star.path))
    return_path.append(a_star.path)

if __name__ == '__main__':
    DISPLAYSURF = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))
    DISPLAYSURF.fill(WHITE)
    pygame.display.set_caption("Game")
    
    for k in range(len(goals)):
        bi_rrt_star=BI_RRT_star(obs,start,goals[k])
        bi_rrt_star.BIRRT_star()
        ada_star_goals=bi_rrt_star.final_path
        print(ada_star_goals)
        for j in range(0,len(ada_star_goals)):
            manager = multiprocessing.Manager()
            return_path = manager.list()

            abc=time.time()
            #if j !=1 and j!=len(ada_star_goals)-1 and distance(ada_star_goals[j][0],ada_star_goals[j][1],ada_star_goals[len(ada_star_goals)-1][0],ada_star_goals[len(ada_star_goals)-1][1])>100:
            ada_star=ADA_star(obs,robot,cart,start,ada_star_goals[j])
            ada_star.generate_path()
            print(len(ada_star.path))

            pos=int(len(ada_star.path)/2)
            a_star_start=ada_star.path[pos][0:4]
            print(a_star_start)
            robot.left_wheel_vel=ada_star.path[pos][4]
            robot.right_wheel_vel=ada_star.path[pos][5]
            

            p=Process(target=f,args=(a_star_start,ada_star_goals[j],robot.left_wheel_vel,robot.right_wheel_vel,return_path))
            p.start()
                
            # else:
            #     ada_star=A_star(obs,robot,cart,start,ada_star_goals[j],0,0)
            #     ada_star.generate_path()
            #     print(len(ada_star.path))
            #     pos=0
            
                
            
            print(time.time()-abc)
            
            print(len(ada_star.path))
            # input("Press Enter to continue...")
            robot.left_wheel_vel=0
            robot.right_wheel_vel=0

            optimal_computed=False
            passed_joinpoint=False
            #Simulation
            old_time=time.time()
            start_time=old_time
            pygame.init()
            while True:
                if optimal_computed==False and passed_joinpoint==False:
                    try:
                        ada_star.path=ada_star.path[:pos]+return_path[0]
                        optimal_computed=True
                    except:
                        pass
                    
                clock.tick(FPS)
                DISPLAYSURF.fill(WHITE)
                for i in obs:
                    pygame.draw.rect(DISPLAYSURF,(0,0,0),i)
                for i in goals:
                    pygame.draw.circle(DISPLAYSURF,(122,122,122),(i[0],i[1]),25)
                pygame.draw.circle(DISPLAYSURF,(255,0,0),(goals[k][0],goals[k][1]),25)
                cart_rect=blit_rotate_center(DISPLAYSURF,cart.img,robot.get_cart_center(cart.cart_length),robot.cart_angle+robot.angle)
                robot_rect=blit_rotate_center(DISPLAYSURF,robot.img,robot.get_center(),robot.angle) 
                for i in ada_star.path:
                    pygame.draw.circle(DISPLAYSURF,(0,255,0),(i[0],i[1]),3)
                
                if optimal_computed :
                    for i in return_path[0]:
                        pygame.draw.circle(DISPLAYSURF,(0,0,255),(i[0],i[1]),3)
                
                for i in range(len(ada_star_goals)-1):
                    pygame.draw.line(DISPLAYSURF,(122,122,0),(ada_star_goals[i][0],ada_star_goals[i][1]),(ada_star_goals[i+1][0],ada_star_goals[i+1][1]),3)
                
                for i in ada_star_goals:
                    pygame.draw.circle(DISPLAYSURF,(0,122,122),(i[0],i[1]),10)

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
                i=floor((t-start_time)/ada_star.step_time)
                if i>pos:
                    passed_joinpoint=True

                try:
                    robot.left_wheel_vel=ada_star.path[i][4]
                    robot.right_wheel_vel=ada_star.path[i][5]
                    robot.cart_angle=ada_star.path[i][3]
                except:
                    robot.left_wheel_vel=0
                    robot.right_wheel_vel=0

                robot.move(del_t)
                old_time=t
                if i==len(ada_star.path):
                    start=[robot.x,robot.y,robot.angle%360,robot.cart_angle]
                    break
                try:
                    if ada_star.check_dyna_coll(curr_dyobs,ada_star.path[i]) or ada_star.check_dyna_coll(curr_dyobs,ada_star.path[i+1]):
                        
                        for idx,path_point in enumerate(ada_star.path[i+1:]):
                            if ada_star.check_dyna_coll(curr_dyobs,path_point)== False:
                                break
                        print('collison')
                        replan_goal=ada_star.path[i+idx+2][0:4]
                        
                        print(replan_goal)
                        
                        robot.left_wheel_vel=0
                        robot.right_wheel_vel=0
                        start=[robot.x,robot.y,robot.angle%360,robot.cart_angle]
                        for k in curr_dyobs:
                            obs.append(k)
                        a_star_replan=A_star(obs,robot,cart,start,replan_goal)
                        a_star_replan.generate_path()
                        ada_star.path=a_star_replan.path+ada_star.path[i+idx+3:]
                        print(len(ada_star.path))
                        if optimal_computed==False:
                            pos=pos-i-idx-2+len(a_star_replan.path)-1
                        old_time=time.time()
                        start_time=old_time
                
                except:
                    pass


        
