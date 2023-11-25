import pygame, sys
from pygame.locals import *
from math import sqrt

def blit_rotate_center(win, image, center, angle):
    rotated_image = pygame.transform.rotate(image, angle)
    new_rect = rotated_image.get_rect(center=center)
    win.blit(rotated_image, new_rect.topleft)
    return new_rect

def distance(x1,y1,x2,y2):
    return sqrt((x1-x2)**2+(y1-y2)**2)


