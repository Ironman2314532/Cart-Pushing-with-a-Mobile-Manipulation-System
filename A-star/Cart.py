import pygame, sys
from pygame.locals import *
from math import sin,cos,radians,tan,floor

class Cart:
    def __init__(self):
        self.cart_width=20
        self.cart_length=40
        self.img=pygame.transform.scale(pygame.image.load('cart.jpg'),(self.cart_length,self.cart_width))