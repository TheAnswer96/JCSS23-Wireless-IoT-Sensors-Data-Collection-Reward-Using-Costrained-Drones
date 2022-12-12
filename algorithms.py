# IMPORTS
from mknapsack import solve_multiple_knapsack
import mknapsack
import numpy as np
import networkx as nx
import random as rnd
from utils import *
import sympy as sp
from sklearn.cluster import KMeans


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
            effective_sol_sensors.append(get_sensor_from_selection(selected[1], set_wps))
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
        if debug:
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
        if debug:
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


def multiRSEO(wps, reward, weight, distance, hovering, E, S, nod, set_cover_reduction=False, debug=False):
    set_wps = [(wps[p][1], p) for p in range(len(wps))]
    set_wps = np.array(set_wps)
    weights = np.array(weight)
    rewards = np.array(reward)
    trivial_storage = weights.sum()
    N = len(rewards)
    capacities = []
    for i in range(nod):
        capacities.append(S)
    if debug:
        print("RSEO with Energy: ", E, " Storage: ", S, " #Sensors: ", N, "#Drones: ", nod)
        print()
    if trivial_storage > S:
        res = solve_multiple_knapsack(rewards[1:], weights[1:], capacities, method='mthm')
    else:
        rnd.seed(2)
        res = rnd.choices(range(1, nod + 1), k=N - 1)
    sol = parse_mkp_sol(res, weights, rewards, nod)

    if debug:
        print("KNAPSACK SOL: ")
        print("STORAGE: ", sol[1], " REWARD: ", sol[0])
        for item in sol[2]:
            print("- Drone: ", item[2], " r: ", item[0], " w: ", item[1])
    drone_sol = []
    if set_cover_reduction:
        for drone in range(nod):
            cover_index, cover_set = set_cover(set(sol[2][drone][2]), set_wps)
            drone_sol.append([cover_index, cover_set])
    else:
        for drone in range(nod):
            cover_set = []
            for i in range(len(sol[2][drone][2])):
                cover_set.append({sol[2][drone][2][i]})
            drone_sol.append([sol[2][drone][2], cover_set])
    if debug and set_cover_reduction:
        print("MIN SET COVER SOL: ")
        for drone in range(nod):
            print("DRONE: ", drone)
            for item in drone_sol[drone][0]:
                print("- element: ", wps[item][0], " sensors covered: ", wps[item][1])

    total_profit = 0
    total_storage = 0
    total_energy = 0
    TSPs = []

    for drone in range(nod):
        tsp_0, energy = TSP_generation(drone_sol[drone][0], distance, hovering, set_wps)
        if debug:
            print("PRE PRUNING:")
            print("TSP: ", tsp_0)
            print("ENERGY: ", energy, " J, REWARD: ", sol[2][drone][0], ", STORAGE: ", sol[2][drone][1], " MB")
        rcv_tsp, profit, energy, storage = TSP_recover(tsp_0, distance, rewards, weights, hovering, set_wps, E)
        TSPs.append([rcv_tsp, profit, energy, storage])
        total_energy = total_energy + energy
        total_profit = total_profit + profit
        total_storage = total_storage + storage
        if debug:
            print("AFTER PRUNING:")
            print("TSP: ", rcv_tsp)
            print("ENERGY: ", energy, " J, REWARD: ", profit, ", STORAGE: ", storage, " MB")
    return [total_profit, total_storage, total_energy, TSPs]


# - END -

def APX_partion(wps, reward, weight, distance, hovering, E, S, nod, strategy=0, debug=False):
    set_wps = [(wps[p][1], p) for p in range(len(wps))]
    idxs, sensors = wpstolist(wps)
    G = generate_graph(idxs[:len(reward)], distance)
    MST = nx.minimum_spanning_tree(G)
    if strategy == 0:
        traversal = list(nx.bfs_edges(MST, source=0))
    else:
        traversal = list(nx.dfs_edges(MST, source=0))
    unpacked = unpack_traversal(traversal)
    if debug:
        print("DFS: ", strategy, "BFS: ", (strategy + 1) % 2)
        print(traversal)
        print("VISIT UNPACKED:")
        print(unpacked)
    unpacked = unpacked[1:]

    partition = []
    partitions = []
    total_profit = 0
    total_storage = 0
    total_energy = 0
    profit = 0
    storage = 0
    energy = 0
    while unpacked != []:
        curr = unpacked.pop(0)
        feasible, param = is_feasible_partion([0] + partition + [curr, 0], set_wps, distance, hovering, weight, reward, E, S)
        if feasible:
            partition.append(curr)
            profit = param[0]
            energy = param[1]
            storage = param[2]
        if not feasible:
            partition = [0] + partition + [0]
            partitions.append([profit, energy, storage, partition])
            partition = [curr]
            feasible, param = is_feasible_partion([0] + partition + [0], set_wps, distance, hovering, weight,reward, E, S)
            profit = param[0]
            energy = param[1]
            storage = param[2]
            if unpacked == []:
                partitions.append([profit, energy, storage, [0] + partition + [0]])

    partitions.sort(key=lambda x: x[0], reverse=True)
    if len(partitions) < nod:
        stop = len(partitions)
    else:
        stop = nod
    for i in range(stop):
        total_profit = total_profit + partitions[i][0]
        total_energy = total_energy + partitions[i][1]
        total_storage = total_storage + partitions[i][2]
        if debug:
            print("Partiotion: ", i, " ", partitions[i])
    return [total_profit, total_energy, total_storage, partitions[:nod]]


def clustering_rseo(wps, reward, weight, distance, hovering, E, S, nod, debug=False):
    wps_copy = copy.deepcopy(wps)
    wps_copy = wps_copy[0:len(weight)]
    set_wps = [(wps_copy[p][1], p) for p in range(len(weight))]
    points = []
    for p in range(len(weight)):
        x,y = wps_copy[p][0]
        points.append((sp.N(x), sp.N(y)))

    sol = []
    for i in range(nod):
        sol.append([])
    # print(hovering)
    # print(distance)
    # print(reward)
    # print(weight)
    total_energy = 0
    total_profit = 0
    total_storage = 0

    sensors_sel = []
    sensors_sel_set = []
    kmeans = KMeans(n_clusters=nod, random_state=42)
    kmeans.fit(points)

    wps_clusterized = []
    weight_drones = []
    reward_drones = []
    hovering_drones = []
    distance_drones = []
    sensor_dict_exachange = []
    for drone in range(nod):
        sensors_sel_set.append([0])
        sensors_sel_set[drone] = set(sensors_sel_set[drone])
        sensors_sel.append([0])
        wps_clusterized.append([])
        weight_drones.append([])
        reward_drones.append([])
        hovering_drones.append([])
        # distance_drones.append([])
        sensor_dict_exachange.append({})

    for index in range(len(kmeans.labels_)):
        if not index in sensors_sel[kmeans.labels_[index]]:
            sensors_sel[kmeans.labels_[index]].append(index)
            sensors_sel_set[kmeans.labels_[index]].add(index)

    for drone in range(nod):
        for wp in sensors_sel[drone]:
            wps_clusterized[drone].append(wps[wp])
            reward_drones[drone].append(reward[wp])
            weight_drones[drone].append(weight[wp])
            hovering_drones[drone].append(hovering[wp])

    for wp in range(len(weight), len(wps)):
        for drone in range(nod):
            if wps[wp][1].issubset(sensors_sel_set[drone]):
                wps_clusterized[drone].append(wps[wp])
                sensors_sel[drone].append(wp)
    # print(sensors_sel)
    for i in range(len(sensors_sel)):
        for j in range(len(sensors_sel[i])):
            sensor_dict_exachange[i][sensors_sel[i][j]]= j
    # print(sensor_dict_exachange)
    # for drone in range(nod):
    #     sub_distance = []
    #     for l in range(len(distance)):
    #         row = []
    #         for k in range(len(distance[0])):
    #             if {l}.issubset(sensors_sel_set[drone]) and {k}.issubset(sensors_sel_set[drone]):
    #                 row.append(distance[l][k])
    #         if row != []:
    #             sub_distance.append(row)
    #     distance_drones.append(sub_distance)
    for drone in sensors_sel:
        sub_distance = []
        for wp1 in drone:
            row = []
            for wp2 in drone:
                row.append(distance[wp1][wp2])
            sub_distance.append(row)
        distance_drones.append(sub_distance)

    # print()
    for wp in range(len(wps_clusterized)):
        for index in wps_clusterized[wp][:]:
            app = []
            for item_set in index[1]:
                app.append(sensor_dict_exachange[wp][item_set])
            index[1] = set(app)
    # print(wps_clusterized[0])
    # print(len(distance_drones[0]), len(distance_drones[0][0]))
    # print(wps_clusterized[1])
    # print(len(distance_drones[1]), len(distance_drones[1][0]))
    # print(wps_clusterized[2])
    # print(len(distance_drones[2]), len(distance_drones[2][0]))

    for drone in range(nod):
        # print("DRONE: ", drone)
        # print(wps_clusterized[drone])
        partial = RSEO(wps_clusterized[drone], reward_drones[drone], weight_drones[drone], distance_drones[drone], hovering_drones[drone], E, S, False)
        total_profit = total_profit + partial[0]
        total_storage = total_storage + partial[1]
        total_energy = total_energy + partial[2]
        sol[drone] = partial
    return [total_profit, total_storage, total_energy, sol]
