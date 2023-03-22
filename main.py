import pygame
import time
import numpy as np
import math

pygame.init()

def r_matrix(angle):
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    return np.array([[cos_a, -sin_a],
                     [sin_a, cos_a]])


def inPolygon(point, poly):
    #Rough bounding box check
    x = [i[0] for i in poly]
    y = [i[1] for i in poly]
    if not (point[0] > min(x) and point[0] < max(x) and point[1] > min(y) and point[1] < max(y)):
        return False
    
    # Ray casting algorithm
    intersections = 0
    for i, _ in enumerate(poly):
        u = poly[i-1]
        v = poly[i]
        if not ((u[1] > point[1] and v[1] > point[1]) or (u[1] < point[1] and v[1] < point[1])):
            s = u[0] + (v[0]-u[0]) * ((point[1]-u[1])/(v[1]-u[1]))
            if point[0] > s:
                intersections += 1

    if intersections % 2 == 0:
        return False
    else:
        return True


def getCentroidPoly(poly):
    A = 0

    for i, _ in enumerate(poly):
        u = poly[i-1]
        v = poly[i]
        A += u[0] * v[1] - v[0] * u[1]
    A = A/2

    centroid = np.array([0.0, 0.0])
    for i, _ in enumerate(poly):
        u = poly[i-1]
        v = poly[i]
        centroid[0] += (u[0] + v[0]) * (u[0] * v[1] - v[0] * u[1])
        centroid[1] += (u[1] + v[1]) * (u[0] * v[1] - v[0] * u[1])
    centroid[0] = centroid[0] / (6 * A)
    centroid[1] = centroid[1] / (6 * A)

    return centroid


def getRotInertiaPoly(poly):
    I = 0
    for i, _ in enumerate(poly):
        u = poly[i-1]
        v = poly[i]
        I += (u[0] * v[1] - v[0] * u[1]) * (v[0]**2 + v[0] * u[0] + u[0]**2 + v[1]**2 + v[1] * u[1] + u[1]**2)
    I = I / 12

    return I


class Polygon():
    def __init__(self, poly, colour, mass, bounciness=0.5, vel=[0, 0], acc=[0, 0], a_vel=0, a_acc=0, **kwargs):
        global phys_objects
        self.id = len(phys_objects)
        phys_objects.append(self)

        self.poly = np.array([[float(i[0]), float(i[1])] for i in poly])
        self.colour = colour
        self.mass= mass
        self.bounciness = bounciness
        self.vel = np.array([float(vel[0]), float(vel[1])])
        self.acc = np.array([float(acc[0]), float(acc[1])])
        self.a_vel = float(a_vel)
        self.a_acc = float(a_acc)

        self.centroid = getCentroidPoly(poly)
        self.rot_inertia = getRotInertiaPoly(poly)


    def rotate(self, radians):
        self.poly = (self.poly - self.centroid).dot(r_matrix(radians * (180/math.pi))) + self.centroid


    def update(self):
        self.vel += self.acc * delta_time
        self.a_vel += self.a_acc * delta_time

        self.centroid += self.vel * delta_time
        self.poly += self.vel * delta_time
        self.rotate(self.a_vel)

        self.render()


    def render(self):
        pygame.draw.polygon(screen, self.colour, self.poly)
        pygame.draw.circle(screen, (0, 0, 0), self.centroid, 2)


phys_objects = []

Polygon([[10, 170], [115, 90], [130, 115], [115, 130]], (255, 0, 0), 10, vel=[15, 0], acc=[0, 9.81], a_acc=0.000001)
Polygon([[210, 170], [315, 90], [330, 115], [315, 130]], (255, 0, 0), 10, vel=[15, 0], acc=[-5, 9.81], a_acc=-0.000001)

# Rendering
s_width = 1920/2
s_height = 1080/2
screen = pygame.display.set_mode((s_width, s_height))

pygame.display.set_caption('Physics Engine')

fps = 60

t1 = time.perf_counter()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))

    t0 = time.perf_counter()
    delta_time = t0 - t1
    t1 = time.perf_counter()

    for obj in phys_objects:
        obj.update()

    pygame.display.update()

pygame.quit()
