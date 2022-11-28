# IMPORTS
import numpy as np
import networkx as nx
import copy


# - RSEO BLOCK -

# - START -

def set_cover(universe, subsets):
    downsizes = []
    for e in subsets:
        if set(e[0]).issubset(universe):
            downsizes.append(e)

    elements = set(e for s in downsizes for e in s[0])
    if elements != universe:
        return set(), set()
    covered = set()
    cover_index = []
    cover_set = []
    while covered != elements:
        subset = max(downsizes, key=lambda s: len(s[0] - covered))
        cover_index.append(subset[1])
        cover_set.append(subset[0])
        covered |= subset[0]

    return [cover_index, cover_set]


def takeFirst(elem):
    return elem[0]


def APX_1_2_KP_algorithm(w, v, W):
    ordered = []
    for i in range(1, len(w)):
        ordered.append((v[i] / w[i], v[i], w[i], i))
    ordered.sort(reverse=True, key=takeFirst)
    S1 = set()
    S1.add(0)
    weight = 0
    val = 0
    for _, vi, wi, i in ordered:
        if weight + wi <= W:
            S1.add(i)
            weight += wi
            val += vi
        else:
            S2 = {0, i}
            val2 = vi
            weight2 = wi
            if val > val2:
                return [S1, val, weight]
            else:
                return [S2, val2, weight2]
    return [S1, val, weight]


def generate_graph(nodes, distance):
    G = nx.Graph()
    for index in nodes:
        G.add_node(index)

    for i in nodes:
        for j in nodes:
            G.add_edge(i, j, weight=distance[i][j])
    return G


def compute_weight_tsp(tsp, w):
    weight = 0
    N = len(tsp)
    for i in range(N - 1):
        weight = weight + w[tsp[i]][tsp[i + 1]]
    return weight


def compute_hovering_tsp(wps, tsp, h):
    hovering = 0
    for node in tsp:
        for sensor in wps[node][0]:
            hovering = hovering + h[sensor]
    return hovering


def tsp_elaboration(tsp, root):
    tsp_rooted = []
    N = len(tsp)
    index = tsp.index(root)
    for i in range(index, N):
        tsp_rooted.append(tsp[i])
    if index != N:
        index = index + 1
    for i in range(1, index):
        tsp_rooted.append(tsp[i])
    return tsp_rooted


def get_list_tsp_reward(tsp, wps, reward):
    lst = []
    for node in tsp:
        r = 0
        if node != 0:
            for sensor in wps[node][0]:
                r = r + reward[sensor]
            lst.append([node, r])
    return lst


def get_tsp_reward_storage(tsp, wps, r, w):
    p = 0
    s = 0
    for node in tsp:
        if node != 0:
            for sensor in wps[node][0]:
                p = p + r[sensor]
                s = s + w[sensor]
    return p, s


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

def get_composite_reward_hovering_storage(r, h, s, w):
    lst_r = []
    lst_h = []
    lst_s = []
    for node in range(len(w)):
        re = 0
        ho = 0
        st = 0
        for sensor in w[node][0]:
            re = re + r[sensor]
            ho = ho + h[sensor]
            st = st + s[sensor]
        lst_r.append([re, w[node][1]])
        lst_h.append([ho, w[node][1]])
        lst_s.append([st, w[node][1]])
    return lst_r, lst_h, lst_s


def get_composite_reward_hovering_storage_single(r, h, s, ws):
    re = 0
    ho = 0
    st = 0
    for sensor in ws[0]:
        re = re + r[sensor]
        ho = ho + h[sensor]
        st = st + s[sensor]

    return re, ho, st


def get_ratios_mre(r, h, d, w, last):
    ratios = []
    for node in range(len(w)):
        ratio = 0
        if d[last][w[node][1]] != 0:
            ratio = r[node][0] / (d[last][node] + d[node][0] + h[node][0])
            ratios.append([ratio, w[node][1]])
    return ratios


def get_ratios_mrs(r, d, s, w, last):
    ratios = []
    for node in range(len(w)):
        ratio = 0
        if d[last][w[node][1]] != 0:
            ratio = r[node][0] / s[node][0]
            ratios.append([ratio, w[node][1]])
    return ratios


def update_sets(inserted, wps):
    to_remove = []
    for node in wps:
        if len(node[0].difference(wps[inserted][0])) != 0:
            node[0].difference_update(wps[inserted][0])
        else:
            to_remove.append(node)
    for e in to_remove:
        wps.remove(e)
    return wps

def get_sensor_from_selection(relative_pos, set_wp):
    index = 0
    for e in set_wp:
        if e[1] == relative_pos:
            index = list(e[0])[0]
    return index
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

# - MRE BLOCK  -

# - START -
def del_drone_selection(elements, todel):
    index_to_del = []
    for e in range(len(elements)):
        if list(elements[e][1])[0] in todel:
            index_to_del.append(e)
    index_to_del.sort(reverse=True)
    for index in index_to_del:
        elements.pop(index)
    print(elements)
    return elements
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

# - MRS BLOCK  -

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