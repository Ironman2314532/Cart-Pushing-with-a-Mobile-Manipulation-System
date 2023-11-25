import pygame, sys
from pygame.locals import *


class test_map:
    def __init__(self):
            self.obs=[]
            border1=pygame.Rect(0,0,10,800)
            self.obs.append(border1)
            border2=pygame.Rect(0,0,1500,10)
            self.obs.append(border2)
            border3=pygame.Rect(0,790,1500,10)
            self.obs.append(border3)
            border4=pygame.Rect(1590,0,10,800)
            self.obs.append(border4)
            obs1=pygame.Rect(300,0,50,500)
            self.obs.append(obs1)
            obs2=pygame.Rect(1000,199,50,600)
            self.obs.append(obs2)
            obs3=pygame.Rect(300,650,50,150)
            self.obs.append(obs3)

            self.start=(70,70,0,0)
            self.goal=(700,400,90,0)



class easy_map_1:
    def __init__(self):
            self.obs=[]
            border1=pygame.Rect(0,0,10,800)
            self.obs.append(border1)
            border2=pygame.Rect(0,0,1600,10)
            self.obs.append(border2)
            border3=pygame.Rect(0,790,1600,10)
            self.obs.append(border3)
            border4=pygame.Rect(1590,0,10,800)
            self.obs.append(border4)

            
            obs1=pygame.Rect(300,0,50,300)
            self.obs.append(obs1)
        
            obs2=pygame.Rect(300,500,50,300)
            self.obs.append(obs2)

            obs3=pygame.Rect(400,10,150,250)
            self.obs.append(obs3)

            obs4=pygame.Rect(700,10,150,250)
            self.obs.append(obs4)

            obs5=pygame.Rect(1000,10,150,250)
            self.obs.append(obs5)


            obs6=pygame.Rect(1300,10,150,250)
            self.obs.append(obs6)

            obs7=pygame.Rect(400,550,150,250)
            self.obs.append(obs7)

            obs8=pygame.Rect(700,550,150,250)
            self.obs.append(obs8)

            obs9=pygame.Rect(1000,550,150,250)
            self.obs.append(obs9)

            obs10=pygame.Rect(1300,550,150,250)
            self.obs.append(obs10)

            self.start=(70,700,0,0)
            self.goals=[(1525,200,90,0),(625,600,270,0)]

class hospital_map_1:
    def __init__(self):
            self.obs=[]
            border1=pygame.Rect(0,0,10,800)
            self.obs.append(border1)
            border2=pygame.Rect(0,0,1600,10)
            self.obs.append(border2)
            border3=pygame.Rect(0,790,1600,10)
            self.obs.append(border3)
            border4=pygame.Rect(1590,0,10,800)
            self.obs.append(border4)

            
            obs1=pygame.Rect(700,0,25,160)
            self.obs.append(obs1)
        
            obs2=pygame.Rect(700,260,25,280)
            self.obs.append(obs2)

            obs3=pygame.Rect(700,640,25,160)
            self.obs.append(obs3)

            obs4=pygame.Rect(0,400,700,25)
            self.obs.append(obs4)


            obs5=pygame.Rect(900,0,25,160)
            self.obs.append(obs5)

            obs6=pygame.Rect(900,260,25,280)
            self.obs.append(obs6)

            obs7=pygame.Rect(900,640,25,160)
            self.obs.append(obs7)

            obs8=pygame.Rect(900,400,700,25)
            self.obs.append(obs8)

            #room2
            obs9=pygame.Rect(1000,10,80,110)
            self.obs.append(obs9)

            obs10=pygame.Rect(1150,10,80,110)
            self.obs.append(obs10)

            obs11=pygame.Rect(1290,10,80,110)
            self.obs.append(obs11)

            obs12=pygame.Rect(1430,10,80,110)
            self.obs.append(obs12)

            

            obs13=pygame.Rect(1000,290,80,110)
            self.obs.append(obs13)

            obs14=pygame.Rect(1150,290,80,110)
            self.obs.append(obs14)

            obs15=pygame.Rect(1290,290,80,110)
            self.obs.append(obs15)

            obs16=pygame.Rect(1430,290,80,110)
            self.obs.append(obs16)

            #room3
            obs17=pygame.Rect(150,500,350,230)
            self.obs.append(obs17)

            #room1
            obs18=pygame.Rect(10,10,200,230)
            self.obs.append(obs18)

            obs19=pygame.Rect(210,300,400,100)
            self.obs.append(obs19)

            obs20=pygame.Rect(300,80,300,150)
            self.obs.append(obs20)

            #room4
            obs21=pygame.Rect(1050,500,10,150)
            self.obs.append(obs21)

            obs22=pygame.Rect(1050,490,420,10)
            self.obs.append(obs22)

            obs23=pygame.Rect(1460,490,10,210)
            self.obs.append(obs23)

            obs24=pygame.Rect(925,700,545,10)
            self.obs.append(obs24)

            #dynamic obs
            self.dyobs=[]
            dyobs1=pygame.Rect(800,400,50,50)
            self.dyobs.append(dyobs1)

            dyobs2=pygame.Rect(1100,150,50,50)
            self.dyobs.append(dyobs2)

            dyobs3=pygame.Rect(550,240,50,50)
            self.dyobs.append(dyobs3)


            self.start=(70,600,90,0)
            
            self.goals=[(800,100,90,0)]

            