# IMPORTS
import numpy as np
import pandas as pd
import sympy as sp
import matplotlib.pyplot as plt
import itertools as it
import math
import scipy.stats as stats
import pickle

# - GLOBAL VARIABLES -
N_POINTS = [10, 15, 20, 25, 50, 100, 150, 200]
# N_POINTS = [15, 25]
H_DRONE = [20, 40]  # m
RAY_DRONE = 50  # m
E = [2500000, 5000000, 10000000]  # J
S = [16000, 32000]  # MB
HEIGHTS = (-5, 5)  # m
WEIGHTS = (100, 1000)  # MB
ZIPF_PARAM = [0, 0.8, -0.8]
ALFA = 200  # J/m
BETA = 700  # J/s
GAMMA = 9  # MB/s
MAX_REWARD = 10
ITERATIONS = 13
MAX_DOWNLOAD_E = (WEIGHTS[1] / GAMMA) * BETA
# MAX_DISTANCE = (E[0] - MAX_DOWNLOAD_E) / alfa
MAX_DISTANCE = 500

# - INPUT FUNCTIONS -

# - START -
np.random.seed(0)


def build_circle(sensor, height):
    global RAY_DRONE

    ray = math.sqrt(pow(RAY_DRONE, 2) - pow((height - sensor.z), 2))
    sensor_2D = sp.Point(sensor.x, sensor.y, evaluate=False)
    # circle = sp.Circle(sensor_2D, ray)
    return [sensor_2D, ray], sensor_2D


def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    # non intersecting
    if d > r0 + r1:
        return None, None
    # One circle within other
    if d < abs(r0 - r1):
        return None, None
    # coincident circles
    if d == 0 and r0 == r1:
        return sp.Point(x0, y0, evaluate=False), sp.Point(x1, y1, evaluate=False)
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(r0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return sp.Point(x3, y3, evaluate=False), sp.Point(x4, y4, evaluate=False)


def generate_rewards(n_points, theta):
    global MAX_REWARD

    x = np.arange(1, MAX_REWARD + 1)
    prob = x ** (-theta)
    prob = prob / prob.sum()
    bounded_zipf = stats.rv_discrete(name='bounded_zipf', values=(x, prob))
    rewards = bounded_zipf.rvs(size=n_points)

    return rewards


def generate_storages(n_points, theta):
    global WEIGHTS
    if theta < 0:
        theta = abs(theta)
        x = np.arange(WEIGHTS[0], WEIGHTS[1] + 1)
        prob = x ** (-theta)
        prob = prob / prob.sum()
        bounded_zipf = stats.rv_discrete(name='bounded_zipf', values=(x, prob))
        weights = bounded_zipf.rvs(size=n_points)
        weights = [WEIGHTS[1] - i for i in weights]
    else:
        x = np.arange(WEIGHTS[0], WEIGHTS[1] + 1)
        prob = x ** (-theta)
        prob = prob / prob.sum()
        bounded_zipf = stats.rv_discrete(name='bounded_zipf', values=(x, prob))
        weights = bounded_zipf.rvs(size=n_points)

    return weights


def get_point_inside_circle(p, circle):
    if math.sqrt((p.x - circle[0].x) ** 2 + (p.y - circle[0].y) ** 2) <= circle[1]:
        return True
    else:
        return False


def generate_3D_points(n_points, theta, height, plot=False):
    list_3D_points = []
    list_circles = []
    list_2D_points = []
    list_intersection_point = []

    global H_DRONE, RAY_DRONE, MAX_DISTANCE, HEIGHTS, ALFA, BETA, GAMMA

    xs = ((-MAX_DISTANCE) - MAX_DISTANCE) * np.random.random_sample((n_points,)) + MAX_DISTANCE
    ys = ((-MAX_DISTANCE) - MAX_DISTANCE) * np.random.random_sample((n_points,)) + MAX_DISTANCE
    zs = (HEIGHTS[0] - HEIGHTS[1]) * np.random.random_sample((n_points,)) + HEIGHTS[1]

    rewards = generate_rewards(n_points, 0)
    weights = generate_storages(n_points, theta)

    rewards = np.insert(rewards, 0, 0)
    weights = np.insert(weights, 0, 0)

    for i in range(n_points):
        point = sp.Point3D(xs[i], ys[i], zs[i], evaluate=False)
        circle, point_2D = build_circle(point, height)
        list_3D_points.append([point, {i + 1}])
        list_circles.append([circle, i + 1])
        list_2D_points.append([point_2D, {i + 1}])

    if plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(xs, ys, zs)
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        plt.show()
    # EVERY PAIRS
    for c1, c2 in it.combinations(list_circles, 2):
        i1, i2 = get_intersections(c1[0][0].x, c1[0][0].y, c1[0][1], c2[0][0].x, c2[0][0].y, c2[0][1])
        # print("CIRCLES: ", c1[1], " ",c2[1])
        if i1 is not None and i1 != i2:
            # print("SECANTE")
            list_intersection_point.append([i1, {c1[1], c2[1]}])
            list_intersection_point.append([i2, {c1[1], c2[1]}])
        if i1 is not None and i1 == i2:
            # print("TANGENTE")
            list_intersection_point.append([i1, {c1[1], c2[1]}])
    # print("INTERSEZIONI TROVATE: ", len(list_intersection_point))
    for c1 in list_intersection_point:
        others = set()
        # print("-HEY")
        for c2 in list_circles:
            if get_point_inside_circle(c1[0], c2[0]):
                others.add(c2[1])
                # print("--CIAO")
        c1[1] = c1[1].union(others)

    waypoints = [[sp.Point(0, 0, evaluate=False), {0}]]
    waypoints = waypoints + list_2D_points + list_intersection_point

    adj_distances = get_adj_matrix_distance(waypoints, ALFA)
    hovering_cost = get_hovering_cost(weights, BETA, GAMMA)
    return [waypoints, list_2D_points, list_intersection_point, rewards, weights, adj_distances, hovering_cost]


def get_distance(p1, p2, alfa):
    return sp.N(p1.distance(p2)) * alfa


def get_hovering_cost(weights, beta, gamma):
    N = len(weights)
    hovering = np.zeros(N)
    for i in range(N):
        hovering[i] = (weights[i] / gamma) * beta
    return hovering


def get_adj_matrix_distance(waypoints, alfa):
    N = len(waypoints)
    matrix = np.zeros((N, N))
    for i in range(N):
        for j in range(N):
            matrix[i][j] = get_distance(waypoints[i][0], waypoints[j][0], alfa)
    return matrix


def generate_problem_instance(dump=False):
    global N_POINTS, ZIPF_PARAM, H_DRONE
    instances = []
    full_instances = []
    for n_point in N_POINTS:
        for theta in ZIPF_PARAM:
            for h in H_DRONE:
                instances = []
                for i in range(ITERATIONS):
                    instances.append(generate_3D_points(n_point, theta, h))
                full_instances.append(instances)
                if dump:
                    name = "problems/problem_n" + str(n_point) + "_t" + str(theta) + "_h" + str(h) + ".dat"
                    outputFile = open(name, 'wb')
                    pickle.dump(instances, outputFile)
                    print("DUMP OF ", name, " DONE.")
    return full_instances

# - END -
