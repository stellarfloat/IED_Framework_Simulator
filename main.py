import math
import os
import random
import sys
import time

import pygame
import pymunk
import pymunk.pygame_util
from pygame.locals import *
from PyQt5 import uic
from PyQt5.QtWidgets import *

'''
units

mass: grams
1px = 1mm
'''


def to_pygame(p):
    """Small hack to convert pymunk to pygame coordinates"""
    return int(p.x), int(-p.y + HEIGHT)


def degree_to_radian(deg):
    return deg * math.pi / 180


def circle_intersection(c1: tuple, r1, c2: tuple, r2) -> tuple:
    '''Calculates coordinates of intersection of two circles'''
    d = math.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(r1**2 - a**2)
    x2 = c1[0] + a * (c2[0] - c1[0]) / d
    y2 = c1[1] + a * (c2[1] - c1[1]) / d
    x3_a = x2 + h * (c2[1] - c1[1]) / d
    y3_a = y2 - h * (c2[0] - c1[0]) / d
    x3_b = x2 - h * (c2[1] - c1[1]) / d
    y3_b = x2 + h * (c2[1] - c1[1]) / d

    return ((x3_a, y3_a), (x3_b, y3_b))


def calc_joint_coords(A, B, C, D, E, pos = (0, 0)) -> list:
    '''Calculates coordinates of upperArm_railplate / lowerArm_upperArm joint.'''
    coords_upperArm_railplate = (pos[0] + (-370 + B + A), pos[1] + (186))
    coords_servo_lowerArm = (pos[0] + (-175 + E - 5.5), pos[1] + (104))
    d = math.dist(coords_upperArm_railplate, coords_servo_lowerArm)
    if d > C + D:
        raise ValueError('Impossible configuration.')
    else:
        try:
            coords_cicle_intersection = circle_intersection(coords_upperArm_railplate, C, coords_servo_lowerArm, D)
        except ValueError:
            raise ValueError('Impossible configuration.')
        else:    
            coords_lowerArm_upperArm = min(coords_cicle_intersection, key=lambda x: x[0])
    return [coords_upperArm_railplate, coords_lowerArm_upperArm]




# Constants
MAIN_DIR = os.path.split(os.path.abspath(__file__))[0]                       ## .py path
UI = uic.loadUiType(os.path.join(MAIN_DIR, 'IED_Framework_Simulator.ui'))[0] ## Qt Designer .ui file import

WIDTH, HEIGHT = 640, 480   ## Window size
FPS = 60                   ## FPS



class EnvironmentSetup(object):
    '''
    This class setups an environment for framework simulation.
    '''
    def __init__(self, space: pymunk.Space, params = {'A': 40, 'B': 132, 'C': 76, 'D': 60, 'E': 37, 'F': 0}, objects = []) -> None:
        print('EnvironmentSetup called') # debug
        ## Track objects
        self.objects = objects
        print(f'bodies: {space._bodies}\nbodies len: {len(space._bodies)}') # debug
        print(f'shapes: {space._shapes}\nshapes len: {len(space._shapes)}') # debug
        print(f'constraints: {space._constraints}\nconstraints len: {len(space._constraints)}') # debug
        ## Reset Environment
        try:
            #space.remove(self.objects)
            print('self.objects len (before setup)', len(self.objects), '\n') # debug
            remove_counter = 0 # debug
            for obj in self.objects:
                print(f'remove -> {obj}')
                space.remove(obj)
                #self.objects.remove(obj)
                remove_counter += 1
            else:
                self.objects = []
                print(f'Removed {remove_counter} items.') # debug
                print('self.objects (after removal)', self.objects, '\n') # debug
        except AttributeError as e:
            print(f'AttributeError: {e}')
        
        ## Base land
        land_body = pymunk.Body(body_type = pymunk.Body.STATIC)
        land_body.position = (320, 10)
        land = pymunk.Segment(land_body, (-320, 0), (320, 0), 15)
        land.friction = 10
        land.elasticity = 0.0
        space.add(land)
        self.objects += [land] #[land_body, land]

        ## Temporary constants
        pos = (320, 26)
        mass = 1000.0
        friction = 0.8
        elasticity = 0.2
        body_type = pymunk.Body.DYNAMIC

        ## Load constants
        A = params['A']
        B = params['B']
        C = params['C']
        D = params['D']
        E = params['E']
        F = params['F']

        ## Calculate joint coordinates
        joint_coords = calc_joint_coords(A, B, C, D, E, pos)

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
            self.objects += [poly]
        else:
            space.add(body_frame)
            self.objects += [body_frame]
        servo_points = [(-175 + E, 84), (-175 + E, 114), (-175 + E - 11, 114), (-175 + E - 11, 84)]
        poly_servo = pymunk.Poly(body_frame, servo_points)
        poly_servo.filter = collision_filter # Disable collision for arms
        space.add(poly_servo)
        self.objects += [poly_servo]

        ## Create railplate
        railplate_components = [[(250, 0), (250, 5), (-250, 5), (-250, 0)], [(-200, 0), (-200, 30)], [(150, 0), (150, 30)]]  # 500mm * 5mm, two stopper
        moment_railplate = pymunk.moment_for_poly(100, railplate_components[0])       # TODO: need to calibrate mass
        body_railplate = pymunk.Body(10, moment_railplate, pymunk.Body.DYNAMIC)       # TODO: need to calibrate mass
        body_railplate.position = pymunk.Vec2d((pos[0] - (120 - B), pos[1] + 186))    # 186mm == base(24mm) + rail support(150mm) + hinge(12mm)
        for component in railplate_components:
            poly = pymunk.Poly(body_railplate, component)
            poly.friction = friction
            poly.elasticity = 0.8 #elasticity
            poly.filter = collision_filter # Disable collision between railplate and upperArm
            space.add(poly)
            self.objects += [poly]
        else:
            space.add(body_railplate)
            self.objects += [body_railplate]
        rotation_center_joint = pymunk.PinJoint(body_frame, body_railplate, (130, 186), (250 - B, 0))
        space.add(rotation_center_joint)
        self.objects += [rotation_center_joint]

        ## Create servo & arms
        ### lowerArm
        pos_servo_lowerArm = (pos[0] - (175 - E + 5.5), pos[1] + 104)
        body_lowerArm = pymunk.Body()
        poly_lowerArm = pymunk.Segment(body_lowerArm, joint_coords[1], pos_servo_lowerArm, 2)
        poly_lowerArm.mass = 10  # TODO:  need to calibrate mass
        poly_lowerArm.filter = collision_filter # Disable collision for arms
        space.add(body_lowerArm, poly_lowerArm)
        self.objects += [body_lowerArm, poly_lowerArm]
        ### servo-lowerArm joint
        joint_servo_lowerArm = pymunk.PinJoint(body_frame, body_lowerArm, (-175 + E - 5.5, 104), pos_servo_lowerArm)
        space.add(joint_servo_lowerArm)
        self.objects += [joint_servo_lowerArm]
        ### servo
        motor_servo = pymunk.SimpleMotor(body_frame, body_lowerArm, 0)
        space.add(motor_servo)
        self.objects += [motor_servo]
        motor_servo.max_force = 183850000 # calibrated to MG90S(1.875kg/cm)
        motor_servo.rate = degree_to_radian(0)
        self.motor_servo = motor_servo
        ### upperArm
        body_upperArm = pymunk.Body()
        poly_upperArm = pymunk.Segment(body_upperArm, joint_coords[0], joint_coords[1], 2)
        poly_upperArm.mass = 10  # TODO: need to calibrate mass)
        poly_upperArm.filter = collision_filter # Disable collision for arms
        space.add(body_upperArm, poly_upperArm)
        self.objects += [body_upperArm, poly_upperArm]
        ### lowerArm-upperArm joint
        joint_lowerArm_upperArm = pymunk.PinJoint(body_lowerArm, body_upperArm, joint_coords[1], joint_coords[1])
        space.add(joint_lowerArm_upperArm)
        self.objects += [joint_lowerArm_upperArm]
        ### upperArm-railplate joint
        joint_upperArm_railplate = pymunk.PinJoint(body_upperArm, body_railplate, joint_coords[0], (-250 + A, 0))
        space.add(joint_upperArm_railplate)
        self.objects += [joint_upperArm_railplate]

        print('self.objects len (after setup)', len(self.objects), '\n') # debug


    def add_ball(self, space: pymunk.Space, pos, radius = 20, mass = 2.7, friction = 0.5, elasticity = 0.9):
        moment = pymunk.moment_for_circle(mass, 0, radius)
        body = pymunk.Body(mass, moment)
        body.position = pymunk.Vec2d(pos)
        shape = pymunk.Circle(body, radius)
        shape.friction = friction
        shape.elasticity = elasticity
        space.add(body, shape)
        return shape

    


class GUI(QMainWindow, UI):
    '''
    This class manages GUI components of simulator.
    '''
    def __init__(self) -> None:
        ## UI init
        super().__init__()
        self.setupUi(self)
        ## UI connect
        self.button_apply.clicked.connect(self.test)
        self.move(1285, 262)

        
    def test(self):
        print('button')

    def closeEvent(self, event):
        sys.exit(0)



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
        self.space = space
        space.gravity = (0.0, -9806.65) ## -9.8m/s^2
        draw_options = pymunk.pygame_util.DrawOptions(screen)

        # Framework environment setup
        setup = EnvironmentSetup(space)
        setup.add_ball(space, (320, 236))

        balls = []

        ticks_to_next_ball = 50
        ball_delay = ticks_to_next_ball
        test1 = False
        test2 = False
        lap = time.time()
        servo_stall = False
        servo_impulse_prev = 0

        running = True
        while running:
            # ball_delay -= 1
            # if ball_delay <= 0:
            #     ball_delay = ticks_to_next_ball
            #     ball_shape = setup.add_ball(space, (320, 400))
            #     balls.append(ball_shape)
            
            for _ in range(50):
                space.step(1/10000.0)

            setup.motor_servo.rate = 5 * ((round(time.time()) % 2) * 2 - 1)
            
            # Servo stall detection
            if math.isclose(setup.motor_servo.impulse, 18385.0, rel_tol=10e-2):
                if servo_stall == True and servo_impulse_prev == setup.motor_servo.impulse:
                    print(f'Servo Stall | t = {round(time.time() - lap, 2)}')
                    servo_stall = False
            else:
                servo_stall = True
            
            servo_impulse_prev = setup.motor_servo.impulse
            '''
            if round(time.time() - lap) > 1:
                if test1 == False:
                    print('##########Test 1##########')
                    setup = EnvironmentSetup(space, params = {'A': 0, 'B': 132, 'C': 76, 'D': 60, 'E': 37, 'F': 0}, objects = setup.objects)
                    test1 = True
                    print('Test 1 OK')
            if round(time.time() - lap) > 2:
                if test2 == False:
                    print('##########Test 2##########')
                    setup = EnvironmentSetup(space, params = {'A': 80, 'B': 132, 'C': 76, 'D': 60, 'E': 37, 'F': 0}, objects = setup.objects)
                    test2 = True
                    print('Test 2 OK')
            '''
            
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


            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                    sys.exit(0)
                elif event.type == KEYDOWN and event.key == K_ESCAPE:
                    running = False
                    sys.exit(0)





if __name__ == '__main__':
    app = QApplication(sys.argv) 
    controlWindow = GUI()
    controlWindow.show()
    Simulator()
    app.exec_()
    
    
    # TODO: Multiprocessing
    # app = QApplication(sys.argv) 
    # controlWindow = GUI()
    # controlWindow.show()
    # app.exec_()

    
