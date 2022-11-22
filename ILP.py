from docplex.mp.model import Model
import numpy as np


def get_i_in_j(wps, N):
    list_i_in_j = []
    for i in range(N):
        i_in_j = set()
        for j in range(len(wps)):
            if i in wps[j]:
                i_in_j.add(j)
        list_i_in_j.append(i_in_j)
    return list_i_in_j


def opt_ilp_cplex(wps, E, S, rewards, weights, distance, hovering, debug):
    set_wps = [p[1] for p in wps]

    set_wps = np.array(set_wps)

    weights = np.array(weights)
    rewards = np.array(rewards)

    N = len(rewards)
    M = len(wps)

    sensor_in_wps = get_i_in_j(set_wps, N)  # list where for each index i we know in which set j the sensor is contained
    # for e in range(len(sensor_in_wps)):
    #     print("i: ", e, " -> ", sensor_in_wps[e])
    # for e in range(len(set_wps)):
    #     print("j: ", e, " -> ", set_wps[e])
    model = Model()
    model.verbose = 0

    # x variables
    A = [(i, j) for i in range(N) for j in range(M)]
    x = model.binary_var_dict(A, name='x')
    # y variables
    B = [(i, j) for i in range(M) for j in range(M)]
    y = model.binary_var_dict(B, name='y')
    # dummy variables
    C = [i for i in range(1, M)]
    u = model.integer_var_dict(C, name='u', lb=1, ub=M - 1)

    # objective function
    model.maximize(model.sum(model.sum(rewards[i] * x[i, j] for j in range(M) for i in set_wps[j])))

    # for each waypoint j we take exactly once the sensor i
    model.add_constraints(model.sum(x[i, j] for j in sensor_in_wps[i]) <= 1 for i in range(N))

    # for the depot we must have exactly one edge which enters and exactly one which exits from it
    model.add_constraint((model.sum(y[0, j] for j in range(1, M)) == model.sum(y[l, 0] for l in range(1, M))))

    # at least one edge must enter and exit to/from the depot
    model.add_constraint(model.sum(y[0, j] for j in range(1, M)) == 1)
    model.add_constraint(model.sum(y[j, 0] for j in range(1, M)) == 1)

    # for each visited waypoint we must have one edge for enter and one for exit
    model.add_constraints(
        (model.sum(y[k, j] for j in range(1, M)) == model.sum(y[l, k] for l in range(1, M))) <= 1 for k in range(1, M))

    # self loop not feasible for waypoints different from the depot
    model.add_constraints((y[j, j] == 0) for j in range(1, M))

    # at least one edge must enter and exit to/from a selected waypoint
    model.add_constraints(
        model.sum(y[l, j] for l in range(M)) == model.max(x[i, j] for i in set_wps[j]) for j in range(1, M))
    model.add_constraints(
        model.sum(y[j, l] for l in range(M)) == model.max(x[i, j] for i in set_wps[j]) for j in range(1, M))
    # precedences
    model.add_constraints((u[l] - u[j] + 1) <= (M - 1) * (1 - y[l, j]) for j in range(1, M) for l in range(1, M))

    # Energy Budget
    model.add_constraint(model.sum(
        model.sum(hovering[i] * x[i, j] for i in set_wps[j]) + model.sum(distance[l, j] * y[l, j] for l in range(M)) for
        j in range(M)) <= E)
    # Storage Budget
    model.add_constraint(model.sum(model.sum(weights[i] * x[i, j] for j in sensor_in_wps[i]) for i in range(1, N)) <= S)

    sol = model.solve(log_output=False)

    total_profit = 0
    storage = 0
    energy = 0

    if debug:
        print()
    for i in range(N):
        profit = 0
        selected_x = [(i, j) for j in range(M) if sol.get_value(x[i, j]) >= 0.99]
        if debug:
            print("Waypoints: {}".format(selected_x))
        for j in range(M):
            energy = energy + (sol.get_value(x[i, j]) * hovering[i])
            profit = profit + (sol.get_value(x[i, j]) * rewards[i])
            storage = storage + (sol.get_value(x[i, j]) * weights[i])
        # print("DRONE ",i," weight: ",weight," reward: ",profit)
        if debug:
            print("  S: %d" % storage)
            print("  P: %d" % profit)
        total_profit = total_profit + profit
        if debug:
            print()
    for l in range(M):
        selected_y = [(l, j) for j in range(M) if sol.get_value(y[l, j]) >= 0.99]
        if debug:
            print("Edges: {}".format(selected_y))
        for j in range(M):
            energy = energy + (sol.get_value(y[l, j]) * distance[l, j])
    if debug:
        print("PROFIT: ", total_profit, " STORAGE: ", storage, " ENERGY: ", energy / 1000000)

    # Add other shit if you want other fields to be returned
    output = total_profit

    return output
