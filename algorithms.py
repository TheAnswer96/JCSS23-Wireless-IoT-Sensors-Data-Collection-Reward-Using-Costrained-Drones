# IMPORTS
import numpy as np
import networkx as nx
import copy
from utils import *

# - RSEO BLOCK -

# - START -

def RSEO(wps, reward, weight, distance, hovering, E, S, debug=False):
    set_wps = [(wps[p][1], p) for p in range(len(wps))]

    set_wps = np.array(set_wps)
    weights = np.array(weight)
    rewards = np.array(reward)
    N = len(rewards)

    if debug:
        print("RSEO with Energy: ", E, " Storage: ", S, " #Sensors: ", N)
        print()
    knapsack = APX_1_2_KP_algorithm(weights, rewards, S)
    if debug:
        print("KNAPSACK SOL: ")
        print("STORAGE: ", knapsack[2], " REWARD: ", knapsack[1])
        for item in knapsack[0]:
            print("- element: ", item, " r: ", rewards[item], " w: ", weights[item])
    cover_index, conver_set = set_cover(knapsack[0], set_wps)
    if debug:
        print("MIN SET COVER SOL: ")
        for item in cover_index:
            print("- element: ", wps[item][0], " sensors covered: ", wps[item][1])
    G = generate_graph(cover_index, distance)
    tsp = nx.algorithms.approximation.christofides(G, weight="weight")
    tsp_0 = tsp_elaboration(tsp, 0)
    energy = compute_weight_tsp(tsp_0, distance) + compute_hovering_tsp(set_wps, tsp_0, hovering)
    if debug:
        print("TSP: ", tsp_0)
        print("ENERGY: ", energy, " J, REWARD: ", knapsack[1], ", STORAGE: ", knapsack[2], " MB")
    info_wp_tsp = get_list_tsp_reward(tsp_0, set_wps, rewards)
    info_wp_tsp.sort(key=lambda x: x[1])
    while energy > E and len(tsp_0) > 3:
        node = info_wp_tsp.pop(0)
        tsp_0.remove(node[0])
        energy = compute_weight_tsp(tsp_0, distance) + compute_hovering_tsp(set_wps, tsp_0, hovering)
    total_profit, total_storage = get_tsp_reward_storage(tsp_0, set_wps, rewards, weights)
    if debug:
        print("TSP: ", tsp_0)
        print("ENERGY: ", energy, " J, REWARD: ", total_profit, ", STORAGE: ", total_storage, " MB")
    return [total_profit, total_storage, energy, tsp_0]


# - END -

# - MRE BLOCK -

# - START -

def MRE(wps, reward, weight, distance, hovering, E, S, debug=False):
    wpss_set = [(wps[p][1], p) for p in range(len(wps))]
    wps_copy = copy.deepcopy(wps)
    set_wps = [(wps_copy[p][1], p) for p in range(len(wps))]
    print(set_wps)
    # set_wps = np.array(set_wps)
    weights = np.array(weight)
    rewards = np.array(reward)

    mre_sol_wps = [0]
    effective_sol_sensors = [0]
    mre_sol_sensors = set()
    last = 0

    total_energy = 0
    total_profit = 0
    total_storage = 0

    r, h, s = get_composite_reward_hovering_storage(rewards, hovering, weights, set_wps)
    if debug:
        for i in range(len(r)):
            print("WAYPOINT: ", i, " r: ", r[i], " s: ", s[i], " h: ", h[i], " set: ", set_wps[i][0])
            for e in set_wps[i][0]:
                print("sensor: ", e, " r: ", rewards[e], " s: ", weights[e], " h: ", hovering[e])
    while len(set_wps) > 1:
        ratios = get_ratios_mre(r, h, distance, set_wps, last)
        if debug:
            print("RATIOS: \n")
            print(ratios)
        ratios.sort(reverse=True, key=takeFirst)
        selected = ratios.pop(0)
        if debug:
            print("POPED: ", selected)
            input()
        for i in range(len(set_wps)):
            if set_wps[i][1] == selected[1]:
                index = i
        p_reward, p_hover, p_weight = get_composite_reward_hovering_storage_single(rewards, hovering, weights,
                                                                                   set_wps[index])
        temporary_sol = mre_sol_wps + [selected[1], 0]
        if (compute_weight_tsp(temporary_sol, distance) + compute_hovering_tsp(wpss_set, temporary_sol,
                                                                               hovering)) <= E and total_storage + p_weight <= S:
            mre_sol_wps.append(selected[1])
            effective_sol_sensors.append(get_sensor_from_selection(selected[1],set_wps))
            mre_sol_sensors.add(selected[0])
            total_storage = total_storage + p_weight
            total_profit = total_profit + p_reward
            set_wps = update_sets(index, set_wps)
            last = selected[1]
        else:
            set_wps.remove(set_wps[index])
        r, h, s = get_composite_reward_hovering_storage(rewards, hovering, weights, set_wps)
        if debug:
            for i in range(len(r)):
                print("WAYPOINT: ", i, " r: ", r[i], " s: ", s[i], " h: ", h[i], " set: ", set_wps[i][0], " dist: ",
                      distance[last][i])
                for e in set_wps[i][0]:
                    print("sensor: ", e, " r: ", rewards[e], " s: ", weights[e], " h: ", hovering[e])
            print("\nreward: ", total_profit, " storage: ", total_storage, " energy: ", total_energy, " e: ",
                  mre_sol_wps)
            input()

    total_energy = compute_weight_tsp(mre_sol_wps + [0], distance) + compute_hovering_tsp(wpss_set, mre_sol_wps,
                                                                                          hovering)
    mre_sol_wps.append(0)
    effective_sol_sensors.append(0)
    return [total_profit, total_storage, total_energy, mre_sol_wps, effective_sol_sensors]


# - ENDS -

# - MRS BLOCK  -

# - START -

def MRS(wps, reward, weight, distance, hovering, E, S, debug=False):
    wpss_set = [(wps[p][1], p) for p in range(len(wps))]
    wps_copy = copy.deepcopy(wps)
    set_wps = [(wps_copy[p][1], p) for p in range(len(wps))]
    # set_wps = np.array(set_wps)
    weights = np.array(weight)
    rewards = np.array(reward)
    N = len(rewards)

    effective_sol_sensors = [0]
    mre_sol_wps = [0]
    mre_sol_sensors = set()
    last = 0

    total_energy = 0
    total_profit = 0
    total_storage = 0

    r, h, s = get_composite_reward_hovering_storage(rewards, hovering, weights, set_wps)
    if debug:
        for i in range(len(r)):
            print("WAYPOINT: ", i, " r: ", r[i], " s: ", s[i], " h: ", h[i], " set: ", set_wps[i][0])
            for e in set_wps[i][0]:
                print("sensor: ", e, " r: ", rewards[e], " s: ", weights[e], " h: ", hovering[e])
    while len(set_wps) > 1:
        ratios = get_ratios_mrs(r, distance, s, set_wps, last)
        if debug:
            print("RATIOS: \n")
            print(ratios)
        ratios.sort(reverse=True, key=takeFirst)
        selected = ratios.pop(0)
        if debug:
            print("POPED: ", selected)
            input()
        for i in range(len(set_wps)):
            if set_wps[i][1] == selected[1]:
                index = i
        p_reward, p_hover, p_weight = get_composite_reward_hovering_storage_single(rewards, hovering, weights,
                                                                                   set_wps[index])
        temporary_sol = mre_sol_wps + [selected[1], 0]
        if (compute_weight_tsp(temporary_sol, distance) + compute_hovering_tsp(wpss_set, temporary_sol,
                                                                               hovering)) <= E and total_storage + p_weight <= S:
            mre_sol_wps.append(selected[1])
            mre_sol_sensors.add(selected[0])
            effective_sol_sensors.append(get_sensor_from_selection(selected[1], set_wps))
            total_storage = total_storage + p_weight
            total_profit = total_profit + p_reward
            set_wps = update_sets(index, set_wps)
        else:
            set_wps.remove(set_wps[index])
        r, h, s = get_composite_reward_hovering_storage(rewards, hovering, weights, set_wps)
        if debug:
            for i in range(len(r)):
                print("WAYPOINT: ", i, " r: ", r[i], " s: ", s[i], " h: ", h[i], " set: ", set_wps[i][0], " dist: ",
                      distance[last][i])
                for e in set_wps[i][0]:
                    print("sensor: ", e, " r: ", rewards[e], " s: ", weights[e], " h: ", hovering[e])
            print("\nreward: ", total_profit, " storage: ", total_storage, " energy: ", total_energy, " e: ",
                  mre_sol_wps)
            input()

    total_energy = compute_weight_tsp(mre_sol_wps + [0], distance) + compute_hovering_tsp(wpss_set, mre_sol_wps,
                                                                                          hovering)
    mre_sol_wps.append(0)
    effective_sol_sensors.append(0)
    return [total_profit, total_storage, total_energy, mre_sol_wps, effective_sol_sensors]


# - END -

# - multiMRE BLOCK  -

# - START -

def multiMRE(wps, reward, weight, distance, hovering, E, S, nod, debug=False):
    wps_copy = copy.deepcopy(wps)
    wps_copy = wps_copy[0:len(weight)]
    set_wps = [(wps_copy[p][1], p) for p in range(len(weight))]
    print(wps_copy)
    print(set_wps)
    sol = []
    for i in range(nod):
        sol.append([])

    total_energy = 0
    total_profit = 0
    total_storage = 0

    sensors_sel = []
    for drone in range(nod):
        # wps_copy.remove
        elements = del_drone_selection(wps_copy, sensors_sel)
        single_sol = MRE(elements, reward, weight, distance, hovering, E, S, debug)
        print("\n#drone: ", drone, " reward: ", single_sol[0])
        print("#drone: ", drone, " storage: ", single_sol[1])
        print("#drone: ", drone, " energy: ", single_sol[2])
        print("#drone: ", drone, " selection: ", single_sol[3])
        total_profit = total_profit + single_sol[0]
        total_storage = total_storage + single_sol[1]
        total_energy = total_energy + single_sol[2]
        sol[drone] = sol[drone] + single_sol[4]
        sensors_sel = sensors_sel + single_sol[4]
        sensors_sel.remove(0)
        sensors_sel.remove(0)
    return [total_profit, total_storage, total_energy, sol]

# - END  -

# - multiMRS BLOCK  -

# - START -
def multiMRS(wps, reward, weight, distance, hovering, E, S, nod, debug=False):
    wps_copy = copy.deepcopy(wps)
    wps_copy = wps_copy[0:len(weight)]
    set_wps = [(wps_copy[p][1], p) for p in range(len(weight))]
    sol = []
    for i in range(nod):
        sol.append([])

    total_energy = 0
    total_profit = 0
    total_storage = 0

    sensors_sel = []
    for drone in range(nod):
        # wps_copy.remove
        elements = del_drone_selection(wps_copy, sensors_sel)
        single_sol = MRS(elements, reward, weight, distance, hovering, E, S, debug)
        print("\n#drone: ", drone, " reward: ", single_sol[0])
        print("#drone: ", drone, " storage: ", single_sol[1])
        print("#drone: ", drone, " energy: ", single_sol[2])
        print("#drone: ", drone, " selection: ", single_sol[3])
        total_profit = total_profit + single_sol[0]
        total_storage = total_storage + single_sol[1]
        total_energy = total_energy + single_sol[2]
        sol[drone] = sol[drone] + single_sol[4]
        sensors_sel = sensors_sel + single_sol[4]
        sensors_sel.remove(0)
        sensors_sel.remove(0)
    return [total_profit, total_storage, total_energy, sol]

# - END  -


# - multiRSEO BLOCK -

# - START -

def multiRSEO(wps, reward, weight, distance, hovering, E, S, debug=False):
    set_wps = [(wps[p][1], p) for p in range(len(wps))]

    set_wps = np.array(set_wps)
    weights = np.array(weight)
    rewards = np.array(reward)
    N = len(rewards)

    if debug:
        print("RSEO with Energy: ", E, " Storage: ", S, " #Sensors: ", N)
        print()
    knapsack = APX_1_2_KP_algorithm(weights, rewards, S)
    if debug:
        print("KNAPSACK SOL: ")
        print("STORAGE: ", knapsack[2], " REWARD: ", knapsack[1])
        for item in knapsack[0]:
            print("- element: ", item, " r: ", rewards[item], " w: ", weights[item])
    cover_index, conver_set = set_cover(knapsack[0], set_wps)
    if debug:
        print("MIN SET COVER SOL: ")
        for item in cover_index:
            print("- element: ", wps[item][0], " sensors covered: ", wps[item][1])
    G = generate_graph(cover_index, distance)
    tsp = nx.algorithms.approximation.christofides(G, weight="weight")
    tsp_0 = tsp_elaboration(tsp, 0)
    energy = compute_weight_tsp(tsp_0, distance) + compute_hovering_tsp(set_wps, tsp_0, hovering)
    if debug:
        print("TSP: ", tsp_0)
        print("ENERGY: ", energy, " J, REWARD: ", knapsack[1], ", STORAGE: ", knapsack[2], " MB")
    info_wp_tsp = get_list_tsp_reward(tsp_0, set_wps, rewards)
    info_wp_tsp.sort(key=lambda x: x[1])
    while energy > E and len(tsp_0) > 3:
        node = info_wp_tsp.pop(0)
        tsp_0.remove(node[0])
        energy = compute_weight_tsp(tsp_0, distance) + compute_hovering_tsp(set_wps, tsp_0, hovering)
    total_profit, total_storage = get_tsp_reward_storage(tsp_0, set_wps, rewards, weights)
    if debug:
        print("TSP: ", tsp_0)
        print("ENERGY: ", energy, " J, REWARD: ", total_profit, ", STORAGE: ", total_storage, " MB")
    return [total_profit, total_storage, energy, tsp_0]


# - END -