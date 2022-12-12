# IMPORTS
import pandas as pd
import pickle
import algorithms as alg
import ILP as ilp
import numpy as np

# # - GLOBAL VARIABLES -
# N_POINTS = [10, 15, 20, 25, 50, 100, 150, 200]
# H_DRONE = [20, 40]  # m
# ZIPF_PARAM = [0, 0.8, -0.8]
#
# E = [2500000, 5000000, 10000000]  # J
# S = [16000, 32000]  # MB

# FRA VARIABLES per multi-drone
N_DRONES = [4]
N_POINTS = [10, 15, 20, 25, 50, 100, 150, 200]
H_DRONE = [20]  # m
ZIPF_PARAM = [0]
E = [2500000]  # J
S = [4000]  # MB

def exaustive_multi_test(zero_hover=False):
    for n_drone in N_DRONES:
        for n_point in N_POINTS:
            for theta in ZIPF_PARAM:
                for h in H_DRONE:
                    name = "problems/exaustive_test/problem_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                        h) + ".dat"
                    file = open(name, 'rb')
                    instances = pickle.load(file)
                    for en in E:
                        for st in S:
                            results = pd.DataFrame(
                                columns=["rseo_profit", "rseo_storage", "rseo_energy", "mre_profit", "mre_storage",
                                         "mre_energy", "mrs_profit", "mrs_storage", "mrs_energy", "opt_profit",
                                         "partition_dfs_profit", "partition_dfs_storage", "partition_dfs_energy",
                                         "partition_bfs_profit", "partition_bfs_storage", "partition_bfs_energy",
                                         "rseo_clust_profit", "rseo_clust_storage", "rseo_clust_energy"])
                            for prob in instances:

                                print("n=%d, l=%d, E=%d, S=%d" % (n_point, n_drone, en, st))

                                print("rseo starts...")
                                hovering = [0 for i in range(len(prob[6]))]
                                if zero_hover:
                                    output = alg.multiRSEO(prob[0], prob[3], prob[4], prob[5], hovering, en, st,
                                                           n_drone, False)
                                else:
                                    output = alg.multiRSEO(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, n_drone,
                                                           False)
                                out_rseo = [output[0], output[1], output[2]]
                                print("rseo done.")

                                print("mre stars...")
                                if zero_hover:
                                    output = alg.multiMRE(prob[0], prob[3], prob[4], prob[5], hovering, en, st, n_drone,
                                                          False)
                                else:
                                    output = alg.multiMRE(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, n_drone,
                                                          False)
                                out_mre = [output[0], output[1], output[2]]
                                print("mre done.")

                                print("mrs stars...")
                                if zero_hover:
                                    output = alg.multiMRS(prob[0], prob[3], prob[4], prob[5], hovering, en, st, n_drone,
                                                          False)
                                else:
                                    output = alg.multiMRS(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, n_drone,
                                                          False)
                                out_mrs = [output[0], output[1], output[2]]
                                print("mrs done.")

                                if n_point < 20:
                                    if zero_hover:
                                        out_ilp = [ilp.opt_multi_ilp_cplex(prob[0], en, st, prob[3], prob[4], prob[5], hovering, n_drone, False)]
                                    else:
                                        out_ilp = [ilp.opt_multi_ilp_cplex(prob[0], en, st, prob[3], prob[4], prob[5], prob[6], n_drone, False)]
                                    print("ilp done.")
                                else:
                                    out_ilp = [0]
                                    print("ilp skipped.")
                                print("partition with DFS done.")
                                if zero_hover:
                                    output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], hovering, en, st, n_drone, 1,
                                                             False)
                                else:
                                    output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, n_drone, 1,
                                                             False)
                                out_partition_dfs = [output[0], output[2], output[1]]
                                print("partition with DFS done.")
                                print("partition with BFS stars...")
                                if zero_hover:
                                    output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], hovering, en, st, n_drone, 0,
                                                             False)
                                else:
                                    output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, n_drone, 0,
                                                             False)
                                out_partition_bfs = [output[0], output[2], output[1]]
                                print("partition with BFS done.")
                                print("rseo with clustering stars...")
                                if zero_hover:
                                    output = alg.clustering_rseo(prob[0], prob[3], prob[4], prob[5], hovering, en, st,
                                                             n_drone, False)
                                else:
                                    output = alg.clustering_rseo(prob[0], prob[3], prob[4], prob[5], prob[6], en, st,
                                                             n_drone, False)
                                out_cluster = [output[0], output[2], output[1]]
                                print("rseo with clustering done.")
                                to_append = out_rseo + out_mre + out_mrs + out_ilp + out_partition_dfs + out_partition_bfs + out_cluster
                                a_series = pd.Series(to_append, index=results.columns)
                                results = results.append(a_series, ignore_index=True)
                                # results.loc[len(results)] = out_rseo + out_mre + out_mrs + out_ilp
                            results_name = "results/multi-exaustive/result_multi"+str(n_drone)+"_n" + str(n_point) + "_t" + str(
                                theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                            if zero_hover:
                                results_name = "results/multi-exaustive/result_zerohover_multi" + str(n_drone) + "_n" + str(n_point) + "_t" + str(
                                    theta) + "_h" + str(
                                    h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                            results.to_csv(results_name)
                            print(results_name, " DONE.")

def exaustive_test(zero_hover=False):
    for n_point in N_POINTS:
        for theta in ZIPF_PARAM:
            for h in H_DRONE:
                name = "problems/exaustive_test/problem_n" + str(n_point) + "_t" + str(theta) + "_h" + str(h) + ".dat"
                file = open(name, 'rb')
                instances = pickle.load(file)
                for en in E:
                    for st in S:
                        results = pd.DataFrame(
                            columns=["rseo_profit", "rseo_storage", "rseo_energy", "mre_profit", "mre_storage",
                                     "mre_energy", "mrs_profit", "mrs_storage", "mrs_energy", "opt_profit",
                                     "partition_dfs_profit", "partition_dfs_storage", "partition_dfs_energy",
                                     "partition_bfs_profit", "partition_bfs_storage", "partition_bfs_energy"])
                        for prob in instances:
                            print("rseo starts...")
                            hovering = [0 for i in range(len(prob[6]))]
                            if zero_hover:
                                output = alg.RSEO(prob[0], prob[3], prob[4], prob[5], hovering, en, st, False)
                            else:
                                output = alg.RSEO(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, False)
                            out_rseo = [output[0], output[1], output[2]]
                            print("rseo done.")
                            print("mre stars...")
                            if zero_hover:
                                output = alg.MRE(prob[0], prob[3], prob[4], prob[5], hovering, en, st, False)
                            else:
                                output = alg.MRE(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, False)
                            out_mre = [output[0], output[1], output[2]]
                            print("mre done.")
                            print("mrs stars...")
                            if zero_hover:
                                output = alg.MRS(prob[0], prob[3], prob[4], prob[5], hovering, en, st, False)
                            else:
                                output = alg.MRS(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, False)
                            out_mrs = [output[0], output[1], output[2]]
                            print("mrs done.")
                            if n_point < 25:
                                if zero_hover:
                                    out_ilp = [
                                        ilp.opt_ilp_cplex(prob[0], en, st, prob[3], prob[4], prob[5], hovering, False)]
                                else:
                                    out_ilp = [
                                        ilp.opt_ilp_cplex(prob[0], en, st, prob[3], prob[4], prob[5], prob[6], False)]
                                print("ilp done.")
                            else:
                                out_ilp = [0]
                                print("ilp skipped.")
                            print("partition with DFS stars...")
                            if zero_hover:
                                output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], hovering, en, st, 1, 1,
                                                         False)
                            else:
                                output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, 1, 1,
                                                         False)
                            out_partition_dfs = [output[0], output[2], output[1]]
                            print("partition with DFS done.")
                            print("partition with BFS stars...")
                            if zero_hover:
                                output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], hovering, en, st, 1, 0,
                                                         False)
                            else:
                                output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, 1, 0,
                                                         False)
                            out_partition_bfs = [output[0], output[2], output[1]]
                            print("partition with BFS done.")
                            to_append = out_rseo + out_mre + out_mrs + out_ilp + out_partition_dfs + out_partition_bfs
                            a_series = pd.Series(to_append, index=results.columns)
                            results = results.append(a_series, ignore_index=True)
                            # results.loc[len(results)] = out_rseo + out_mre + out_mrs + out_ilp
                        results_name = "results/exaustive/result_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                            h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        if zero_hover:
                            results_name = "results/exaustive/result_zerohover_n" + str(n_point) + "_t" + str(
                                theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        results.to_csv(results_name)
                        print(results_name, " DONE.")
    return


def altitude_test(same_scenario=False, zero_hover=False):
    N_POINTS = [20]
    H_DRONE = [30, 35, 40, 45]
    ZIPF_PARAM = [0]

    E = [2500000, 5000000]  # J
    S = [2000, 4000, 6000, 8000]  # MB
    for n_point in N_POINTS:
        for theta in ZIPF_PARAM:
            for h in H_DRONE:
                if same_scenario:
                    name = "problems/altitude_same_set_test/problem_alt_n" + str(n_point) + "_t" + str(
                        theta) + "_h" + str(h) + ".dat"
                else:
                    name = "problems/altitude_test/problem_alt_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                        h) + ".dat"
                file = open(name, 'rb')
                instances = pickle.load(file)
                for en in E:
                    for st in S:
                        results = pd.DataFrame(
                            columns=["rseo_profit", "rseo_storage", "rseo_energy", "mre_profit", "mre_storage",
                                     "mre_energy", "mrs_profit", "mrs_storage", "mrs_energy", "opt_profit",
                                     "partition_dfs_profit", "partition_dfs_storage", "partition_dfs_energy",
                                     "partition_bfs_profit", "partition_bfs_storage", "partition_bfs_energy"])
                        for prob in instances:
                            print("rseo starts...")
                            hovering = [0 for i in range(len(prob[6]))]
                            if zero_hover:
                                output = alg.RSEO(prob[0], prob[3], prob[4], prob[5], hovering, en, st, False)
                            else:
                                output = alg.RSEO(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, False)
                            out_rseo = [output[0], output[1], output[2]]
                            print("rseo done.")
                            print("mre stars...")
                            if zero_hover:
                                output = alg.MRE(prob[0], prob[3], prob[4], prob[5], hovering, en, st, False)
                            else:
                                output = alg.MRE(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, False)
                            out_mre = [output[0], output[1], output[2]]
                            print("mre done.")
                            print("mrs stars...")
                            if zero_hover:
                                output = alg.MRS(prob[0], prob[3], prob[4], prob[5], hovering, en, st, False)
                            else:
                                output = alg.MRS(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, False)
                            out_mrs = [output[0], output[1], output[2]]
                            print("mrs done.")
                            if n_point < 50:
                                if zero_hover:
                                    out_ilp = [
                                        ilp.opt_ilp_cplex(prob[0], en, st, prob[3], prob[4], prob[5], hovering, False)]
                                else:
                                    out_ilp = [
                                        ilp.opt_ilp_cplex(prob[0], en, st, prob[3], prob[4], prob[5], prob[6], False)]
                                print("ilp done.")
                            else:
                                out_ilp = [0]
                                print("ilp skipped.")
                            print("partition with DFS stars...")
                            if zero_hover:
                                output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], hovering, en, st, 1, 1,
                                                         False)
                            else:
                                output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, 1, 1,
                                                         False)
                            out_partition_dfs = [output[0], output[2], output[1]]
                            print("partition with DFS done.")
                            print("partition with BFS stars...")
                            if zero_hover:
                                output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], hovering, en, st, 1, 0,
                                                         False)
                            else:
                                output = alg.APX_partion(prob[0], prob[3], prob[4], prob[5], prob[6], en, st, 1, 0,
                                                         False)
                            out_partition_bfs = [output[0], output[2], output[1]]
                            print("partition with BFS done.")
                            to_append = out_rseo + out_mre + out_mrs + out_ilp + out_partition_dfs + out_partition_bfs
                            a_series = pd.Series(to_append, index=results.columns)
                            results = results.append(a_series, ignore_index=True)
                            # results.loc[len(results)] = out_rseo + out_mre + out_mrs + out_ilp
                        if same_scenario:
                            results_name = "results/altitude_same_set/result_n" + str(n_point) + "_t" + str(
                                theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        else:
                            results_name = "results/altitude/result_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        if zero_hover:
                            results_name = "results/result_zerohover_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        results.to_csv(results_name)
                        print(results_name, " DONE.")
    return


def confi(data):
    from scipy.stats import t
    m = data.mean()
    s = data.std()

    dof = len(data) - 1

    confidence = 0.95

    t_crit = np.abs(t.ppf((1 - confidence) / 2, dof))

    return s * t_crit / np.sqrt(len(data))


def compact_csv_plot_ratio(zero_hover=False):
    for theta in ZIPF_PARAM:
        for h in H_DRONE:
            for en in E:
                for st in S:
                    compact_csv = pd.DataFrame(
                        columns=["x", "n", "rseo", "rseo_std", "rseo_conf", "mre", "mre_std", "mre_conf", "mrs",
                                 "mrs_std",
                                 "mrs_conf", "opt", "opt_std", "opt_conf"])
                    for n_point in [10, 15, 20, 25]:
                        csv_name = "results/result_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                            h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        if zero_hover:
                            csv_name = "results/result_zerohover_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        csv = pd.read_csv(csv_name)
                        csv["rseo_ratio"] = csv["rseo_profit"] / csv["opt_profit"]
                        csv["mre_ratio"] = csv["mre_profit"] / csv["opt_profit"]
                        csv["mrs_ratio"] = csv["mrs_profit"] / csv["opt_profit"]
                        csv["opt_ratio"] = csv["opt_profit"] / csv["opt_profit"]

                        rseo_mean = csv["rseo_ratio"].mean()
                        rseo_std = csv["rseo_ratio"].std()
                        rseo_conf = confi(1.0 * np.array(csv["rseo_ratio"]))

                        mre_mean = csv["mre_ratio"].mean()
                        mre_std = csv["mre_ratio"].std()
                        mre_conf = confi(1.0 * np.array(csv["mre_ratio"]))

                        mrs_mean = csv["mrs_ratio"].mean()
                        mrs_std = csv["mrs_ratio"].std()
                        mrs_conf = confi(1.0 * np.array(csv["mrs_ratio"]))

                        opt_mean = csv["opt_ratio"].mean()
                        opt_std = csv["opt_ratio"].std()
                        opt_conf = confi(1.0 * np.array(csv["opt_ratio"]))

                        to_append = [N_POINTS.index(n_point) + 1] + [n_point] + [rseo_mean] + [rseo_std] + [
                            rseo_conf] + [mre_mean] + [mre_std] + [
                                        mre_conf] + [
                                        mrs_mean] + [mrs_std] + [mrs_conf] + [opt_mean] + [opt_std] + [opt_conf]
                        a_series = pd.Series(to_append, index=compact_csv.columns)
                        compact_csv = compact_csv.append(a_series, ignore_index=True)

                    results_name = "results/csv/ratio/csv_ratio_t" + str(theta) + "_h" + str(
                        h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                    if zero_hover:
                        results_name = "results/csv/no_hovering/csv_ratio_t" + str(theta) + "_h" + str(
                            h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                    compact_csv.to_csv(results_name)
                    print(results_name, " DONE.")
    return


def compact_csv_plot_reward(zero_hover=False):
    for theta in ZIPF_PARAM:
        for h in H_DRONE:
            for en in E:
                for st in S:
                    compact_csv = pd.DataFrame(
                        columns=["x", "n", "rseo", "rseo_std", "rseo_conf", "mre", "mre_std", "mre_conf", "mrs",
                                 "mrs_std",
                                 "mrs_conf", "opt", "opt_std", "opt_conf"])
                    for n_point in N_POINTS:
                        csv_name = "results/result_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                            h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        if zero_hover:
                            csv_name = "results/result_zerohover_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        csv = pd.read_csv(csv_name)

                        rseo_mean = csv["rseo_profit"].mean()
                        rseo_std = csv["rseo_profit"].std()
                        rseo_conf = confi(1.0 * np.array(csv["rseo_profit"]))

                        mre_mean = csv["mre_profit"].mean()
                        mre_std = csv["mre_profit"].std()
                        mre_conf = confi(1.0 * np.array(csv["mre_profit"]))

                        mrs_mean = csv["mrs_profit"].mean()
                        mrs_std = csv["mrs_profit"].std()
                        mrs_conf = confi(1.0 * np.array(csv["mrs_profit"]))

                        opt_mean = csv["opt_profit"].mean()
                        opt_std = csv["opt_profit"].std()
                        opt_conf = confi(1.0 * np.array(csv["opt_profit"]))

                        to_append = [N_POINTS.index(n_point)] + [n_point] + [rseo_mean] + [rseo_std] + [rseo_conf] + [
                            mre_mean] + [mre_std] + [
                                        mre_conf] + [
                                        mrs_mean] + [mrs_std] + [mrs_conf] + [opt_mean] + [opt_std] + [opt_conf]
                        a_series = pd.Series(to_append, index=compact_csv.columns)
                        compact_csv = compact_csv.append(a_series, ignore_index=True)

                    results_name = "results/csv/reward/csv_t" + str(theta) + "_h" + str(
                        h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                    if zero_hover:
                        results_name = "results/csv/no_hovering/csv_t" + str(theta) + "_h" + str(
                            h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                    compact_csv.to_csv(results_name)
                    print(results_name, " DONE.")
    return


def replace_zeros():
    for theta in ZIPF_PARAM:
        for h in H_DRONE:
            for en in E:
                for st in S:
                    csv_name = "results/multi-exaustive/final_csv" + str(theta) + "_h" + str(
                        h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                    csv = pd.read_csv(csv_name)
                    csv['opt'].replace({0: "", 0.0: ""}, inplace=True)
                    csv['opt_std'].replace({0: "", 0.0: ""}, inplace=True)
                    csv['opt_conf'].replace({0: "", 0.0: ""}, inplace=True)
                    csv.to_csv(csv_name)
    return


def compact_csv_plot_ratio_altitude(zero_hover=False):
    ZIPF_PARAM = [0]
    E = [2500000, 5000000]  # J
    S = [2000, 4000, 8000, 16000]  # MB
    H_DRONE = [10, 15, 20, 25, 30, 35, 40, 45]

    for theta in ZIPF_PARAM:
        for en in E:
            for st in S:
                for n_point in [10, 15, 20]:
                    compact_csv = pd.DataFrame(
                        columns=["x", "n", "rseo", "rseo_std", "rseo_conf", "mre", "mre_std", "mre_conf", "mrs",
                                 "mrs_std",
                                 "mrs_conf", "opt", "opt_std", "opt_conf","partition_dfs", "partition_dfs_std", "partition_dfs_conf", "partition_bfs", "partition_bfs_std", "partition_bfs_conf"])
                    for h in H_DRONE:
                        csv_name = "results/altitude/result_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                            h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        if zero_hover:
                            csv_name = "results/altitude/result_zerohover_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        csv = pd.read_csv(csv_name)
                        csv["rseo_ratio"] = csv["rseo_profit"] / csv["opt_profit"]
                        csv["mre_ratio"] = csv["mre_profit"] / csv["opt_profit"]
                        csv["mrs_ratio"] = csv["mrs_profit"] / csv["opt_profit"]
                        csv["opt_ratio"] = csv["opt_profit"] / csv["opt_profit"]
                        csv["partition_dfs_ratio"] = csv["partition_dfs_profit"] / csv["opt_profit"]
                        csv["partition_bfs_ratio"] = csv["partition_bfs_profit"] / csv["opt_profit"]

                        rseo_mean = csv["rseo_ratio"].mean()
                        rseo_std = csv["rseo_ratio"].std()
                        rseo_conf = confi(1.0 * np.array(csv["rseo_ratio"]))

                        mre_mean = csv["mre_ratio"].mean()
                        mre_std = csv["mre_ratio"].std()
                        mre_conf = confi(1.0 * np.array(csv["mre_ratio"]))

                        mrs_mean = csv["mrs_ratio"].mean()
                        mrs_std = csv["mrs_ratio"].std()
                        mrs_conf = confi(1.0 * np.array(csv["mrs_ratio"]))

                        opt_mean = csv["opt_ratio"].mean()
                        opt_std = csv["opt_ratio"].std()
                        opt_conf = confi(1.0 * np.array(csv["opt_ratio"]))

                        dfs_mean = csv["partition_dfs_ratio"].mean()
                        dfs_std = csv["partition_dfs_ratio"].std()
                        dfs_conf = confi(1.0 * np.array(csv["partition_dfs_ratio"]))

                        bfs_mean = csv["partition_bfs_ratio"].mean()
                        bfs_std = csv["partition_bfs_ratio"].std()
                        bfs_conf = confi(1.0 * np.array(csv["partition_bfs_ratio"]))

                        to_append = [H_DRONE.index(h) + 1] + [h] + [rseo_mean] + [rseo_std] + [
                            rseo_conf] + [mre_mean] + [mre_std] + [
                                        mre_conf] + [
                                        mrs_mean] + [mrs_std] + [mrs_conf] + [opt_mean] + [opt_std] + [opt_conf] + [dfs_mean] + [dfs_std] + [dfs_conf] + [bfs_mean] + [bfs_std] + [bfs_conf]
                        a_series = pd.Series(to_append, index=compact_csv.columns)
                        compact_csv = compact_csv.append(a_series, ignore_index=True)

                    results_name = "results/altitude/final_csv/alt_t" + str(theta) + "_n" + str(
                        n_point) + "_en" + str(en) + "_st" + str(st) + ".csv"
                    if zero_hover:
                        results_name = "results/altitude/final_csv/alt_NH_ratio_t" + str(theta) + "_n" + str(
                            n_point) + "_en" + str(en) + "_st" + str(st) + ".csv"
                    compact_csv.to_csv(results_name)
                    print(results_name, " DONE.")
    return


def compact_csv_plot_reward(zero_hover=False):
    ZIPF_PARAM = [0]
    E = [2500000, 5000000, 10000000]  # J
    S = [2000, 4000, 8000, 16000]  # MB
    H_DRONE = [20]

    for theta in ZIPF_PARAM:
        for h in H_DRONE:
            for en in E:
                for st in S:
                    compact_csv = pd.DataFrame(
                        columns=["x", "n", "rseo", "rseo_std", "rseo_conf", "mre", "mre_std", "mre_conf", "mrs",
                                 "mrs_std",
                                 "mrs_conf", "opt", "opt_std", "opt_conf"])
                    for n_point in N_POINTS:
                        csv_name = "results/exaustive/result_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                            h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        if zero_hover:
                            csv_name = "results/exaustive/result_zerohover_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        csv = pd.read_csv(csv_name)

                        rseo_mean = csv["rseo_profit"].mean()
                        rseo_std = csv["rseo_profit"].std()
                        rseo_conf = confi(1.0 * np.array(csv["rseo_profit"]))

                        mre_mean = csv["mre_profit"].mean()
                        mre_std = csv["mre_profit"].std()
                        mre_conf = confi(1.0 * np.array(csv["mre_profit"]))

                        mrs_mean = csv["mrs_profit"].mean()
                        mrs_std = csv["mrs_profit"].std()
                        mrs_conf = confi(1.0 * np.array(csv["mrs_profit"]))

                        opt_mean = csv["opt_profit"].mean()
                        opt_std = csv["opt_profit"].std()
                        opt_conf = confi(1.0 * np.array(csv["opt_profit"]))

                        to_append = [N_POINTS.index(n_point)] + [n_point] + [rseo_mean] + [rseo_std] + [rseo_conf] + [
                            mre_mean] + [mre_std] + [
                                        mre_conf] + [
                                        mrs_mean] + [mrs_std] + [mrs_conf] + [opt_mean] + [opt_std] + [opt_conf]
                        a_series = pd.Series(to_append, index=compact_csv.columns)
                        compact_csv = compact_csv.append(a_series, ignore_index=True)

                    results_name = "results/exaustive/final_csv/ex_t" + str(theta) + "_h" + str(
                        h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                    if zero_hover:
                        results_name = "results/exaustive/final_csv/ex_nohover_t" + str(theta) + "_h" + str(
                            h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                    compact_csv.to_csv(results_name)
                    print(results_name, " DONE.")
    return

def compact_csv_plot_reward_multi(zero_hover=False):
    ZIPF_PARAM = [0]
    E = [2500000, 5000000]  # J
    S = [2000, 4000]  # MB
    H_DRONE = [20]
    N_DRONES = [2, 3, 4]

    for theta in ZIPF_PARAM:
        for h in H_DRONE:
            for en in E:
                for st in S:
                    for drone in N_DRONES:
                        compact_csv = pd.DataFrame(
                            columns=["x", "n", "rseo", "rseo_std", "rseo_conf", "mre", "mre_std", "mre_conf", "mrs",
                                     "mrs_std",
                                     "mrs_conf", "opt", "opt_std", "opt_conf", "partition_dfs", "partition_dfs_std",
                                     "partition_dfs_conf", "partition_bfs", "partition_bfs_std", "partition_bfs_conf", "cluster", "cluster_std", "cluster_conf"])
                        for n_point in N_POINTS:
                            csv_name = "results/multi-exaustive/result_multi"+str(drone)+"_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                            if zero_hover:
                                csv_name = "results/multi-exaustive/result_multi"+str(drone)+"_zerohover_n" + str(n_point) + "_t" + str(theta) + "_h" + str(
                                    h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                            csv = pd.read_csv(csv_name)

                            rseo_mean = csv["rseo_profit"].mean()
                            rseo_std = csv["rseo_profit"].std()
                            rseo_conf = confi(1.0 * np.array(csv["rseo_profit"]))

                            mre_mean = csv["mre_profit"].mean()
                            mre_std = csv["mre_profit"].std()
                            mre_conf = confi(1.0 * np.array(csv["mre_profit"]))

                            mrs_mean = csv["mrs_profit"].mean()
                            mrs_std = csv["mrs_profit"].std()
                            mrs_conf = confi(1.0 * np.array(csv["mrs_profit"]))

                            opt_mean = csv["opt_profit"].mean()
                            opt_std = csv["opt_profit"].std()
                            opt_conf = confi(1.0 * np.array(csv["opt_profit"]))

                            dfs_mean = csv["partition_dfs_profit"].mean()
                            dfs_std = csv["partition_dfs_profit"].std()
                            dfs_conf = confi(1.0 * np.array(csv["partition_dfs_profit"]))

                            bfs_mean = csv["partition_bfs_profit"].mean()
                            bfs_std = csv["partition_bfs_profit"].std()
                            bfs_conf = confi(1.0 * np.array(csv["partition_bfs_profit"]))

                            clust_mean = csv["rseo_clust_profit"].mean()
                            clust_std = csv["rseo_clust_profit"].std()
                            clust_conf = confi(1.0 * np.array(csv["rseo_clust_profit"]))

                            to_append = [N_POINTS.index(n_point)] + [n_point] + [rseo_mean] + [rseo_std] + [rseo_conf] + [mre_mean] + [mre_std] + [mre_conf] + [mrs_mean] + [mrs_std] + [mrs_conf] + [opt_mean] + [opt_std] + [opt_conf] + [dfs_mean] + [dfs_std] + [dfs_conf] + [bfs_mean] + [bfs_std] + [bfs_conf] + [clust_mean] + [clust_std] + [clust_conf]
                            a_series = pd.Series(to_append, index=compact_csv.columns)
                            compact_csv = compact_csv.append(a_series, ignore_index=True)

                        results_name = "results/multi-exaustive/final_csv/ex_multi"+str(drone)+"_t" + str(theta) + "_h" + str(
                            h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        if zero_hover:
                            results_name = "results/multi-exaustive/final_csv/ex_multi"+str(drone)+"_nohover_t" + str(theta) + "_h" + str(
                                h) + "_en" + str(en) + "_st" + str(st) + ".csv"
                        compact_csv.to_csv(results_name)
                        print(results_name, " DONE.")
    return