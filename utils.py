# IMPORTS
import numpy as np
import networkx as nx
import copy

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