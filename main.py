import math
import os
import random
from re import A
import sys

import pygame
import pymunk
import pymunk.pygame_util
from pygame.locals import *
from PyQt5 import uic
from PyQt5.QtWidgets import *



def add_ball(space):
    mass = 2.7
    radius = 20
    moment = pymunk.moment_for_circle(mass, 0, radius) # 1
    body = pymunk.Body(mass, moment) # 2
    x = random.randint(299, 301)
    body.position = x, 550 # 3
    shape = pymunk.Circle(body, radius) # 4
    shape.friction = 1
    shape.elasticity = 0.7
    space.add(body, shape) # 5
    return shape


def add_L(space):
    rotation_center_body = pymunk.Body(body_type = pymunk.Body.STATIC) # 1
    rotation_center_body.position = (300, 300)

    body = pymunk.Body(10, 10000) # 2
    body.position = (300, 300)
    l1 = pymunk.Segment(body, (-150, 0), (255.0, 0.0), 5.0)
    l2 = pymunk.Segment(body, (-150.0, 0), (-150.0, 50.0), 5.0)

    rotation_center_joint = pymunk.PinJoint(body, rotation_center_body, (0,0), (0,0)) # 3

    space.add(l1, l2, body, rotation_center_joint)
    return l1,l2


def to_pygame(p):
    """Small hack to convert pymunk to pygame coordinates"""
    return int(p.x), int(-p.y + HEIGHT)


def create_box(space: pymunk.Space, pos, w, h, mass = 5.0, friction = 0.4, elasticity = 0.7, body_type = pymunk.Body.DYNAMIC) -> None:
    box_points = [(-w/2, -h/2), (-w/2, h/2), (w/2, h/2), (w/2, -h/2)]
    moment = pymunk.moment_for_poly(mass, box_points)
    body = pymunk.Body(mass, moment, body_type)
    body.position = pymunk.Vec2d(pos)
    poly = pymunk.Poly(body, box_points)
    poly.friction = friction
    poly.elasticity = elasticity
    space.add(body, poly)


def create_frame(space: pymunk.Space, pos, params, mass = 1000.0, friction = 0.8, elasticity = 0.4, body_type = pymunk.Body.DYNAMIC) -> None:
    ## Load constants
    A = params['A']
    B = params['B']
    C = params['C']
    D = params['D']
    E = params['E']
    F = params['F']

    ## Add collision filter 
    collision_filter = pymunk.ShapeFilter(group=1)

    ## Create base frame
    box_points = [(175, 0), (175, 174), (130, 174), (130, 24), (-145 + E, 24), (-145 + E, 124), (-175 + E, 124), (-175 + E, 24), (-175, 24), (-175, 0)]
    moment_frame = pymunk.moment_for_poly(mass, box_points)
    body_frame = pymunk.Body(mass, moment_frame, body_type)
    body_frame.position = pymunk.Vec2d(pos)
    frames = [[(175, 0), (175, 24), (-175, 24), (-175, 0)], [(175, 24), (175, 174), (130, 174), (130, 24)], [(-145 + E, 24), (-145 + E, 124), (-175 + E, 124), (-175 + E, 24)], [(130, 146), (130, 186)]]
    for frame in frames:
        poly = pymunk.Poly(body_frame, frame)
        poly.friction = friction
        poly.elasticity = elasticity
        space.add(poly)
    else:
        space.add(body_frame)
    servo_points = [(-175 + E, 84), (-175 + E, 114), (-175 + E - 11, 114), (-175 + E - 11, 84)]
    poly_servo = pymunk.Poly(body_frame, servo_points)
    poly_servo.filter = collision_filter # Disable collision for arms
    space.add(poly_servo)

    ## Create railplate
    railplate_components = [[(250, 0), (250, 5), (-250, 5), (-250, 0)], [(-200, 0), (-200, 30)], [(150, 0), (150, 30)]]  # 500mm * 5mm, two stopper
    moment_railplate = pymunk.moment_for_poly(10, railplate_components[0])         # need to calibrate mass
    body_railplate = pymunk.Body(10, moment_railplate, pymunk.Body.DYNAMIC)        # need to calibrate mass
    body_railplate.position = pymunk.Vec2d((pos[0] - (120 - B), pos[1] + 186)) # 186mm == base(24mm) + rail support(150mm) + hinge(12mm)
    for component in railplate_components:
        poly = pymunk.Poly(body_railplate, component)
        poly.friction = friction
        poly.elasticity = elasticity
        space.add(poly)
    else:
        space.add(body_railplate)
    rotation_center_joint = pymunk.PinJoint(body_frame, body_railplate, (130, 186), (250 - B, 0))
    space.add(rotation_center_joint)

    ## Create servo & arms
    body_lowerArm = pymunk.Body()
    poly_lowerArm = pymunk.Segment(body_lowerArm, (-D, 0), (0, 0), 2)
    poly_lowerArm.mass = 10  # need to calibrate mass
    body_lowerArm.position = pymunk.Vec2d((pos[0] - (175 - E + 5.5), pos[1] + 104))
    poly_lowerArm.filter = collision_filter # Disable collision for arms
    space.add(body_lowerArm, poly_lowerArm)
    joint_servo_lowerArm = pymunk.PinJoint(body_frame, body_lowerArm, (-175 + E - 5.5, 104), (0, 0))
    space.add(joint_servo_lowerArm)
    motor_servo = pymunk.SimpleMotor(body_frame, body_lowerArm, 0)
    space.add(motor_servo)
    motor_servo.rate = 1

    

#def create_railplate(space: pymunk.Space, pos, E = 30, mass = 1000.0, friction = 0.8, elasticity = 0.4, body_type = pymunk.Body.DYNAMIC) -> None:



# Constants
MAIN_DIR = os.path.split(os.path.abspath(__file__))[0]                       ## .py path
UI = uic.loadUiType(os.path.join(MAIN_DIR, 'IED_Framework_Simulator.ui'))[0] ## Qt Designer .ui file import

WIDTH, HEIGHT = 640, 480   ## Window size
FPS = 60                   ## FPS



class EnvironmentSetup(object):
    '''
    This class setups an environment for framework simulation.
    '''
    def __init__(self, space: pymunk.Space, params = {'A': 0, 'B': 100, 'C': 0, 'D': 50, 'E': 30, 'F': 0}) -> None:
        ## Base land
        land_body = pymunk.Body(body_type = pymunk.Body.STATIC)
        land_body.position = (320, 10)
        land = pymunk.Segment(land_body, (-320, 0), (320, 0), 15)
        land.friction = 10
        land.elasticity = 0.0
        space.add(land)

        create_frame(space, (320, 28), params)
        


        rotation_center_body = pymunk.Body(body_type = pymunk.Body.STATIC) # 1
        rotation_center_body.position = (300, 300)

        body = pymunk.Body(10, 10000) # 2
        body.position = (300, 300)
        l1 = pymunk.Segment(body, (-150, 0), (255.0, 0.0), 5.0)
        l2 = pymunk.Segment(body, (-150.0, 0), (-150.0, 50.0), 5.0)

        rotation_center_joint = pymunk.PinJoint(body, rotation_center_body, (0,0), (0,0)) # 3

        #space.add(l1, l2, body, rotation_center_joint)


class GUI(QMainWindow, UI):
    '''
    This class manages GUI components of simulator.
    '''
    def __init__(self) -> None:
        ## UI init
        super().__init__()
        self.setupUi(self)
        ## UI connect
        pass


class Simulator(object):
    '''
    This class manages execution of main simulation.
    '''
    def __init__(self) -> None:
        # Pygame initialize
        pygame.init()
        screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption('IED_Framework_Simulator')
        clock = pygame.time.Clock()
        
        # Pymunk initialize
        space = pymunk.Space()
        space.gravity = (0.0, -9806.65) ## -9.8m/s^2
        draw_options = pymunk.pygame_util.DrawOptions(screen)

        # Framework environment setup
        EnvironmentSetup(space)


        #lines = add_L(space)
        balls = []
        

        ticks_to_next_ball = 50
        ball_delay = ticks_to_next_ball
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    sys.exit(0)
                elif event.type == KEYDOWN and event.key == K_ESCAPE:
                    sys.exit(0)

            
            ball_delay -= 1
            if ball_delay <= 0:
                ball_delay = ticks_to_next_ball
                ball_shape = add_ball(space)
                balls.append(ball_shape)
            
            for _ in range(50):
                space.step(1/10000.0)

            screen.fill((255,255,255))

            balls_to_remove = []
            for ball in balls:
                if ball.body.position.y < 0: # 1
                    balls_to_remove.append(ball) # 2

            for ball in balls_to_remove:
                space.remove(ball, ball.body) # 3
                balls.remove(ball) # 4
            
            space.debug_draw(draw_options)

            pygame.display.flip()
            clock.tick(FPS)




if __name__ == '__main__':
    Simulator()

    # TODO: Multiprocessing
    # app = QApplication(sys.argv) 
    # controlWindow = GUI()
    # controlWindow.show()
    # app.exec_()

    
