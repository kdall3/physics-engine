import pygame
import time
import numpy as np
import math
import copy

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


def getClosestPolyEdge(point, poly):
    distances = []
    edges = []
    for i, _ in enumerate(poly):
        p1 = poly[i-1]
        p2 = poly[i]
        
        r = np.dot(p2-p1, point-p1) / (absVect(p2-p1))**2
        if r > 0 and r < 1:
            edges.append([p1, p2])
            distances.append(math.sqrt(absVect(point-p1)**2 - (r * absVect(p2-p1))**2))

    return edges[distances.index(min(distances))]



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
            if 'step' in dir(obj):
                self.game_objects[i] = obj.step(time)
    

    def saveState(self):
        self.last_state = []
        for obj in self.game_objects:
            self.last_state.append(copy.deepcopy(obj))


    def handleCollisions(self):
        for obj_A in self.game_objects:
            for obj_B in self.game_objects:
                if obj_A != obj_B:
                    collided_verts = getVerticesInPolygon(obj_A.poly, obj_B.poly)
                    if len(collided_verts) > 0:
                        collided_vert = collided_verts[0]
                        collided_vert_index = np.where(obj_A.poly==collided_vert)[0][0]

                        # Binary search algorithm
                        step_increment = 0.5
                        impact_time = 0

                        obj_A_initial = self.last_state[obj_A.id]
                        obj_B_initial = self.last_state[obj_B.id]
                        obj_A = copy.deepcopy(obj_A_initial)
                        obj_B = copy.deepcopy(obj_B_initial)

                        while distPointToPoly(obj_A.poly[collided_vert_index], obj_B.poly) > self.collision_accuracy or checkPointInPolygon(obj_A.poly[collided_vert_index], obj_B.poly): # Needs to only end when the vertex is outside of the poly
                            time.sleep(0.1)

                            if checkPointInPolygon(obj_A.poly[collided_vert_index], obj_B.poly):
                                if impact_time == 0:
                                    print("Cannot backtrack")
                                    break
                                impact_time -= step_increment
                            else:
                                impact_time += step_increment
                            
                            step_increment = step_increment / 2

                            obj_A = copy.deepcopy(obj_A_initial)  # weird python object referencing thing
                            obj_B = copy.deepcopy(obj_B_initial)

                            obj_A.step(impact_time * self.delta_time)
                            obj_B.step(impact_time * self.delta_time)

                        print(checkPointInPolygon(obj_A.poly[collided_vert_index], obj_B.poly))
                        print(impact_time)
                        print(distPointToPoly(obj_A.poly[collided_vert_index], obj_B.poly))
                        
                        # Update the rest of the objects
                        for other_obj in self.game_objects:
                            if other_obj.id != obj_A.id and other_obj.id != obj_B.id:
                                other_obj.step(impact_time)
                        
                        # Calculate the results of the collision
                        p = obj_A.poly[collided_vert_index]

                        w_a1 = obj_A.a_vel
                        w_b1 = obj_B.a_vel
                        r_ap = p - obj_A.centroid
                        r_bp = p - obj_B.centroid

                        b_edge = getClosestPolyEdge(p, obj_B.poly)
                        m_n = -1 / (b_edge[1][1] - b_edge[0][1] / b_edge[1][0] - b_edge[0][0])
                        n = np.array([math.sqrt(1 / (1 + m_n**2)), math.sqrt(1 / (1 + 1 / m_n**2))])

                        e = (obj_A.elasticity + obj_B.elasticity)/2

                        v_ab1 = obj_A.vel + np.array([-w_a1 * r_ap[1], w_a1 * r_ap[0]]) - obj_B.vel - np.array([-w_b1 * r_bp[1], w_b1 * r_bp[0]])
                        
                        j = (-(1+e) * np.dot(v_ab1, n)) / (1/obj_A.mass + 1/obj_B.mass + (np.cross(r_ap, n)**2) / obj_A.rot_inertia + (np.cross(r_bp, n)**2) / obj_B.rot_inertia)

                        obj_A.vel = obj_A.vel + j * n / obj_A.mass
                        obj_B.vel = obj_B.vel + j * n / obj_B.mass

                        obj_A.a_vel = obj_A.a_vel + np.cross(r_ap, j * n) / obj_A.rot_inertia
                        obj_B.a_vel = obj_B.a_vel + np.cross(r_bp, j * n) / obj_B.rot_inertia

                        print(obj_A.vel)


    def update(self):
        self.delta_time = time.perf_counter() - self.t

        self.step(self.delta_time)
        self.handleCollisions()
        
        self.t = time.perf_counter()

        self.saveState()


    def render(self):
        for obj in self.game_objects:
            if 'render' in dir(obj):
                obj.render(self)


class Polygon():
    def __init__(self, simulation, poly, colour, mass, elasticity=1, vel=[0, 0], acc=[0, 0], a_vel=0, a_acc=0):
        self.id = len(simulation.game_objects)
        simulation.game_objects.append(self)

        self.poly = np.array([[float(i[0]), float(i[1])] for i in poly])
        self.colour = colour
        self.mass= mass
        self.elasticity = elasticity
        self.vel = np.array([float(vel[0]), float(vel[1])])
        self.acc = np.array([float(acc[0]), float(acc[1])])
        self.a_vel = float(a_vel)
        self.a_acc = float(a_acc)

        self.centroid = getCentroidPoly(poly)
        self.rot_inertia = getRotInertiaPoly(poly)


    def rotate(self, radians):
        self.poly = (self.poly - self.centroid).dot(r_matrix(radians * (180/math.pi))) + self.centroid


    def step(self, delta_time):
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

Polygon(simulation, [[10, 170], [115, 90], [130, 115], [115, 130]], (255, 0, 0), 10, vel=[100, 0], acc=[0, 0], a_acc=0)
Polygon(simulation, [[210, 170], [315, 90], [330, 115], [315, 130]], (0, 255, 0), 10, vel=[-100, 0], acc=[0, 0], a_acc=0)

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