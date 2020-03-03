import random
import sys
import time
import cProfile

import numpy as np
from scene_parser import parse_scene_file_path
from arr2_epec_seg_ex import Point_2, Polygon_2, intersection, Segment_2, K_neighbor_search, Euclidean_distance, FT, \
    Gmpq, Kd_tree, Point_d

from scipy.spatial import cKDTree


def log(o):
    print("=== object type:", type(o))
    print("=== object toString:", o)


def get_scene_limits(obstacles):
    x_min, x_max, y_min, y_max = 4 * [None]
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


def get_random_point(limits):
    return [random.uniform(x_min, x_max) for (x_min, x_max) in limits]


class Edge(object):
    def __init__(self, previous_edge, steering, target, state, sampled_edges):
        self.sampled_edges = sampled_edges
        self.previous_edge = previous_edge
        self.steering = steering
        self.target = target
        self.state = state


def get_closest_k_neighbours(point, tree, k):
    distances, indexes = tree.query(x=point, k=k)
    if type(indexes) is list:
        return [list(tree.data[i]) for i in indexes]
    else:
        return [list(tree.data[indexes])]


def point_d_to_list(p_d):
    return [p_d[i] for i in range(p_d.dimension())]


def find_closest_edge(point, edges, tree):
    neighbour_state = get_closest_k_neighbours(point, tree, 1)[0]
    return find_edge_by_state(neighbour_state, edges)


def get_random_steering():
    return [random.uniform(0, 1), random.uniform(0, 2 * np.pi)]


def get_new_state(state, steering, dt, options, reverse=False):
    if reverse:
        a_x = -options["thrust"] / options["mass"] * steering[0] * np.cos(steering[1])
        a_y = -options["g"] / options["mass"] - options["thrust"] / options["mass"] * steering[0] * np.sin(steering[1])

        v_x = state[2] - dt * a_x
        v_y = state[3] - dt * a_y

        x = state[0] - dt * v_x - dt ** 2 * a_x
        y = state[1] - dt * v_y - dt ** 2 * a_y
    else:
        a_x = options["thrust"] / options["mass"] * steering[0] * np.cos(steering[1])
        a_y = options["g"] / options["mass"] + options["thrust"] / options["mass"] * steering[0] * np.sin(steering[1])

        v_x = state[2] + dt * a_x
        v_y = state[3] + dt * a_y

        x = state[0] + dt * v_x + dt ** 2 * a_x
        y = state[1] + dt * v_y + dt ** 2 * a_y

    return [x, y, v_x, v_y]


def point2_list_to_polygon_2(lst):
    l = []
    for point in lst:
        l.append(point)
    p = Polygon_2(l)
    if p.is_clockwise_oriented():
        p.reverse_orientation()
    return p


def is_path_valid(source, target, obstacles):
    movement_segment = Segment_2(Point_2(source[0], source[1]), Point_2(target[0], target[1]))
    for obstacle in obstacles:
        for edge in obstacle.edges():
            if not intersection(movement_segment, edge).empty():
                return False
    return True


def point2_to_list(p):
    return [p.x().to_double(), p.y().to_double()]


def list_to_point_d(l):
    return Point_d(len(l), [FT(e) for e in l])


def sample_edges_on_path(previous_edge, u, options, n, reverse=False):
    states = [previous_edge.state]
    dt_percent = random.uniform(0,1)
    for i in range(1, n):
        state = get_new_state(states[-1], u, dt_percent*options["dt"]/n, options, reverse)
        states.append(state)
    sampled_edges = []
    for s in states:
        target_new = s[:2]
        current_edge = Edge(previous_edge=previous_edge, steering=u, target=target_new, state=s,
                            sampled_edges=[])
        sampled_edges.append(current_edge)
        previous_edge = current_edge
    return sampled_edges


def find_distance(point, tree):
    distances, indexes = tree.query(x=point, k=1)
    return distances, list(tree.data[indexes])


def find_edge_by_state(state, edges):
    for edge in edges:
        if edge.state == state:
            return edge
    raise Exception("something went terribly wrong")


def generate_path(path, robots, obstacles, destinations, edges, options):
    print(options)
    t_start = time.time()
    x_min, x_max, y_min, y_max = get_scene_limits(obstacles)
    max_random_speed = 1e1
    max_random_acceleration = 1e1
    random_coordinates_limits = [(x_min, x_max), (y_min, y_max),
                                 (-max_random_speed, max_random_speed),
                                 (-max_random_acceleration, max_random_acceleration)]
    desired_speed = [0, 0]
    destination_position = point2_to_list(destinations[0])
    destination_state = destination_position + desired_speed
    obstacles_polygons = [point2_list_to_polygon_2(obstacle) for obstacle in obstacles]
    K = int(options["K"])
    if K == 0:
        K = +np.inf
    robot_initial_position = point2_to_list(robots[0][0])
    robot_initial_speed = [0, 0]
    initial_state = robot_initial_position + robot_initial_speed
    init_edge = Edge(previous_edge=None, steering=[0, 0], target=robot_initial_position, state=initial_state, sampled_edges=[])
    dest_edge = Edge(previous_edge=None, steering=[0, 0], target=destination_position, state=destination_state, sampled_edges=[])
    init_edges =[init_edge]
    dest_edges =[dest_edge]
    joint_dest_edge = dest_edge
    joint_init_edge = init_edge
    initial_states_tree = cKDTree([initial_state])
    destination_states_tree = cKDTree([destination_state])
    number_sampling_points = 5
    i = 0
    while i < K:
        i += 1
        if 0 == i % 100:
            print("#", i, "/", K)
        state_rand = get_random_point(random_coordinates_limits)
        initial_states_tree = expand_RRT(init_edges, obstacles_polygons, options, state_rand, initial_states_tree, number_sampling_points)
        current_init_state = initial_states_tree.data[-1]
        distance_to_dest_tree, closest_dest_state = find_distance(current_init_state, destination_states_tree)
        if distance_to_dest_tree < options["epsilon"] and is_path_valid(current_init_state, closest_dest_state, obstacles_polygons):
            joint_dest_edge = find_edge_by_state(closest_dest_state, dest_edges)
            joint_init_edge = init_edges[-1]
            break
        if options['two-sided']:
            destination_states_tree = expand_RRT(dest_edges, obstacles_polygons, options, state_rand, destination_states_tree, number_sampling_points, reverse=True)
            current_dest_state = destination_states_tree.data[-1]
            distance_to_init_tree, closest_init_state = find_distance(current_dest_state, initial_states_tree)
            if distance_to_init_tree < options["epsilon"] and is_path_valid(current_dest_state, closest_init_state, obstacles_polygons):
                joint_init_edge = find_edge_by_state(closest_init_state, init_edges)
                joint_dest_edge = dest_edges[-1]
                break

    path_dest = get_path(joint_dest_edge)
    path_init = get_path(joint_init_edge)
    path_init.reverse()
    path.extend(path_init)
    path.extend(path_dest)

    init_edges.pop(0)
    edges.extend(init_edges)
    dest_edges.pop(0)
    edges.extend(dest_edges)

    t_end = time.time()
    print("Running time:", (t_end - t_start))


def get_path(first_edge):
    path = []
    current_edge = first_edge
    while current_edge:
        for e in current_edge.sampled_edges[::-1]:
            x = e.target[0]
            y = e.target[1]
            path.append(Point_2(x, y))
        current_edge = current_edge.previous_edge
    return path


def expand_RRT(current_edges, obstacles_polygons, options, state_rand, states_tree, number_sampling_points, reverse = False):
    u_rand = get_random_steering()
    edge_near = find_closest_edge(state_rand, current_edges, states_tree)
    sampled_edges_along_path = sample_edges_on_path(edge_near, u_rand, options, n=number_sampling_points, reverse=reverse)
    for i in range(number_sampling_points - 1):
        if not is_path_valid(sampled_edges_along_path[i].state, sampled_edges_along_path[i + 1].state, obstacles_polygons):
            return states_tree
    state_new = sampled_edges_along_path[-1].state
    current_edge = Edge(previous_edge=edge_near, steering=u_rand, state=state_new, target=state_new[:2], sampled_edges=sampled_edges_along_path)
    states_tree = cKDTree(list(states_tree.data) + [state_new])
    current_edges.append(current_edge)
    return states_tree


if __name__ == "__main__":
    robots_count, robots_goals, robots, obstacles = parse_scene_file_path(sys.argv[1])
    path = []
    edges = []
    options = {
        "mass": 1,
        "dt": 1,
        "thrust": 1,
        "g": -0.5,
        "two-sided": True,
        "K": 10000,
        "epsilon": 10,
    }
    cProfile.run("generate_path(path, robots, obstacles, robots_goals, edges, options)")
