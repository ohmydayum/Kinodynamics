import random
import numpy as np
from scene_parser import parse_scene_file_path
from arr2_epec_seg_ex import Point_2, Polygon_2, intersection, Segment_2


def log(o):
    print("=== object type:", type(o))
    print("=== object toString:", o)


def are_close_enough(p1, p2, epsilon=1e-3):
    return np.linalg.norm(np.asarray(p1)-np.asarray(p2)) < epsilon


def get_scene_limits(obstacles):
    x_min, x_max, y_min, y_max = 4*[None]
    for obstacle in obstacles:
        for p in obstacle:
            if not x_min or p.x().to_double() < x_min:
                x_min = p.x().to_double()
            if not x_max or p.x().to_double() > x_max:
                x_max = p.x().to_double()
            if not y_min or p.y().to_double() < y_min:
                y_min = p.y().to_double()
            if not y_max or p.y().to_double() > y_min:
                y_max = p.y().to_double()
    return x_min, x_max, y_min, y_max


def get_random_point(x_min, x_max, y_min, y_max):
    return [random.uniform(x_min, x_max), random.uniform(y_min, y_max)]


class Edge(object):
    def __init__(self, previous_edge, steering, target):
        self.previous_edge = previous_edge
        self.steering = steering
        self.target = target


# TODO
def find_closest_edge(point, edges):
    best_edge = None
    best_distance = -1
    for e in edges:
        p = e.target
        current_distance = np.linalg.norm(np.asarray(p)-np.asarray(point))
        if current_distance < best_distance or not best_edge:
            best_edge = e
            best_distance = current_distance
    return best_edge


def get_random_steering():
    return [random.uniform(0, 1), random.uniform(0, 2*np.pi)]


# TODO
def get_new_state(x_near, u_rand, delta_t):
    x = x_near[0] + delta_t * u_rand[0] * np.cos(u_rand[1])
    y = x_near[1] + delta_t * u_rand[0] * np.sin(u_rand[1])
    return [x, y]


def point2_list_to_polygon_2(lst):
    l = []
    for point in lst:
        l.append(point)
    p = Polygon_2(l)
    if p.is_clockwise_oriented():
        p.reverse_orientation()
    return p


# TODO
def is_path_valid(source, target, obstacles):
    movement_segment = Segment_2(Point_2(source[0], source[1]), Point_2(target[0], target[1]))
    for obstacle in obstacles:
        for edge in obstacle.edges():
            if not intersection(movement_segment, edge).empty():
                return False
    return True


def point2_to_list(p):
    return [p.x().to_double(), p.y().to_double()]


def generate_path(path, robots, obstacles, destinations):
    x_min, x_max, y_min, y_max = get_scene_limits(obstacles)
    K = 20000
    delta_t = 1
    robot_initial_position = point2_to_list(robots[0][0])
    destination = point2_to_list(destinations[0])
    obstacles_polygons = [point2_list_to_polygon_2(obstacle) for obstacle in obstacles]
    init_edge = Edge(previous_edge=None, steering=[0, 0], target=robot_initial_position)
    edges = [init_edge]
    for i in range(K):
        print("#", i, "/", K)
        last_target = edges[-1].target
        if are_close_enough(last_target, destination, epsilon=1):
            break
        x_rand = get_random_point(x_min, x_max, y_min, y_max)
        edge_near = find_closest_edge(x_rand, edges)
        x_near = edge_near.target
        u_rand = get_random_steering()
        x_new = get_new_state(x_near, u_rand, delta_t)
        if is_path_valid(x_near, x_new, obstacles_polygons):
            current_edge = Edge(previous_edge=edge_near, steering=u_rand, target=x_new)
            edges.append(current_edge)
    current_edge = edges[-1]
    while current_edge:
        x = current_edge.target[0]
        y = current_edge.target[1]
        path.append([Point_2(x, y)])
        current_edge = current_edge.previous_edge
    path.reverse()