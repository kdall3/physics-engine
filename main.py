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


def absVect(vect):
    return math.sqrt(vect[0]**2 + vect[1]**2)


def getPolyBoundingBox(poly):
    x = [i[0] for i in poly]
    y = [i[1] for i in poly]
    return ((min(x), min(y)), (max(x), max(y)))


def checkPointInBox(point, box):
    if (point[0] > box[0][0] and point[0] < box[1][0]) and (point[1] > box[0][1] and point[0] < box[1][1]):
        return True
    else:
        return False


def checkPointInPolygon(point, poly):
    # Rough check
    bounding_box = getPolyBoundingBox(poly)
    if not checkPointInBox(point, bounding_box):
        return False
    
    # Ray casting algorithm
    intersections = 0
    for i, _ in enumerate(poly):
        p1 = poly[i-1]
        p2 = poly[i]
        if not ((p1[1] > point[1] and p2[1] > point[1]) or (p1[1] < point[1] and p2[1] < point[1])):
            s = p1[0] + (p2[0]-p1[0]) * ((point[1]-p1[1])/(p2[1]-p1[1]))
            if point[0] > s:
                intersections += 1

    if intersections % 2 == 0:
        return False
    else:
        return True


def checkBoxCollision(box1, box2):
    vertices = [box1[0], (box1[1][0], box1[0][1]), (box1[0][0], box1[1][1]), box1[1]]

    for vert in vertices:
        if checkPointInBox(vert, box2):
            return True
        
    return False


def getVerticesInPolygon(poly1, poly2):
    # Rough check
    box1 = getPolyBoundingBox(poly1)
    box2 = getPolyBoundingBox(poly2)

    if not checkBoxCollision(box1, box2):
        return []
    
    collided_vertices = []
    for vert in poly1:
        if checkPointInPolygon(vert, poly2):
            collided_vertices.append(vert)
    
    return collided_vertices


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
        p1 = poly[i-1]
        p2 = poly[i]
        I += (p1[0] * p2[1] - p2[0] * p1[1]) * (p2[0]**2 + p2[0] * p1[0] + p1[0]**2 + p2[1]**2 + p2[1] * p1[1] + p1[1]**2)
    I = I / 12

    return I


def distPointToPoly(point, poly):
    distances = []
    for i, _ in enumerate(poly):
        p1 = poly[i-1]
        p2 = poly[i]

        r = np.dot(p2-p1, point-p1) / (absVect(p2-p1))**2
        if r > 1:
            distances.append(absVect(point-p2))
        elif r < 0:
            distances.append(absVect(point-p1))
        else:
            distances.append(math.sqrt(absVect(point-p1)**2 - (r * absVect(p2-p1))**2))
    
    return min(distances)

class Simulation:
    def __init__(self, screen_dimensions, collision_accuracy=0.001):
        self.collision_accuracy = collision_accuracy
        self.screen_dimensions = screen_dimensions
        self.screen = pygame.display.set_mode(screen_dimensions)
        self.game_objects = []
        self.t = time.perf_counter()
        self.last_state = self.game_objects


    def step(self, time):
        for i, obj in enumerate(self.game_objects):
            if 'update' in dir(obj):
                self.game_objects[i] = obj.update(self, time)


    def handleCollisions(self):
        for obj in self.game_objects:
            for target in self.game_objects:
                if obj != target:
                    collided_verts = getVerticesInPolygon(obj.poly, target.poly)
                    if len(collided_verts) > 0:
                        collided_vert = collided_verts[0]
                        collided_vert_index = np.where(obj.poly==collided_vert)[0][0]

                        # Binary search algorithm
                        step_increment = 0.5
                        impact_time = 0

                        obj_initial = self.last_state[obj.id]
                        target_initial = self.last_state[target.id]
                        obj = obj_initial
                        target = target_initial

                        while distPointToPoly(obj.poly[collided_vert_index], target.poly) > self.collision_accuracy: # Needs to only end when the vertex is outside of the poly
                            print(distPointToPoly(obj.poly[collided_vert_index], target.poly))

                            if checkPointInPolygon(obj.poly[collided_vert_index], target.poly):
                                impact_time -= step_increment
                            else:
                                impact_time += step_increment
                            
                            step_increment = step_increment / 2

                            obj = obj_initial  # weird python object referencing thing
                            target = target_initial

                            obj.update(self, impact_time * self.delta_time)
                            target.update(self, impact_time * self.delta_time)
                            self.render()
                        

    def update(self):
        self.delta_time = time.perf_counter() - self.t

        self.step(self.delta_time)
        self.handleCollisions()
        
        self.t = time.perf_counter()

        self.last_state = self.game_objects
                

    def render(self):
        for obj in self.game_objects:
            if 'render' in dir(obj):
                obj.render(self)


class Polygon():
    def __init__(self, simulation, poly, colour, mass, bounciness=0.5, vel=[0, 0], acc=[0, 0], a_vel=0, a_acc=0):
        self.id = len(simulation.game_objects)
        simulation.game_objects.append(self)

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


    def update(self, simulation, delta_time):
        self.vel += self.acc * delta_time
        self.a_vel += self.a_acc * delta_time

        self.centroid += self.vel * delta_time
        self.poly += self.vel * delta_time
        self.rotate(self.a_vel * delta_time)

        return self


    def render(self, simulation):
        pygame.draw.polygon(simulation.screen, self.colour, self.poly)
        pygame.draw.circle(simulation.screen, (0, 0, 0), self.centroid, 2)


simulation = Simulation((400, 400))

Polygon(simulation, [[10, 170], [115, 90], [130, 115], [115, 130]], (255, 0, 0), 10, vel=[15, 0], acc=[8, 9.81], a_acc=0.01)
Polygon(simulation, [[210, 170], [315, 90], [330, 115], [315, 130]], (0, 255, 0), 10, vel=[-40, 0], acc=[-8, 9.81], a_acc=-0.01)

# Rendering
s_width = 1920/2
s_height = 1080/2
screen = pygame.display.set_mode((s_width, s_height))

pygame.display.set_caption('Physics Engine')

fps = 60

t1 = time.perf_counter()

count = 0

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))

    simulation.update()
    simulation.render()

    if count == 100:
        count = 0
    
    verts = getVerticesInPolygon(simulation.game_objects[1].poly, simulation.game_objects[0].poly) + getVerticesInPolygon(simulation.game_objects[0].poly, simulation.game_objects[1].poly)
    for vert in verts:
        pygame.draw.circle(simulation.screen, (0, 0, 255), vert, 2)

    pygame.display.update()

    count += 1

pygame.quit()
