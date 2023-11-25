import pygame, sys
from pygame.locals import *

def blit_rotate_center(win, image, center, angle):
    rotated_image = pygame.transform.rotate(image, angle)
    new_rect = rotated_image.get_rect(center=center)
    win.blit(rotated_image, new_rect.topleft)
    return new_rect


