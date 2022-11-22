# IMPORTS
import inputs as ipp
import time
import ILP as ilp
import pickle
import algorithms as alg
import sympy as sp
import tests as ts



# - MAIN -
if __name__ == '__main__':
    tic = time.perf_counter()
    #a = ipp.generate_3D_points(100, 0, 20)
    # file = open("app.dat", 'rb')
    # a = pickle.load(file)
    #ipp.generate_problem_instance(True)
    # exit()
    # for i in range(len(a[0])):
    #     print("WAYPOINT: ", sp.N(a[0][i][0]))
    #     for j in a[0][i][1]:
    #         print("SENSOR: ",j, " STORAGE: ",a[4][j], " HOVERING: ",a[6][j], " REWARD: ", a[3][j])
    # print()
    # # for i in range(len(a[0])):
    # #     for j in range(len(a[0])):
    # #         print("EDGE ", i, " , ",j," cost: ",a[5][i][j])
    # #output = ilp.opt_ilp_cplex(a[0], E[0], S[0], a[3], a[4], a[5], a[6], True)
    # output = alg.RSEO(a[0], a[3], a[4], a[5], a[6], E[0], S[0], True)
    # print(output)
    # output = alg.MRE(a[0], a[3], a[4], a[5], a[6], E[0], S[0], False)
    # print(output)
    # output = alg.MRS(a[0], a[3], a[4], a[5], a[6], E[0], S[0], False)
    # print(output)
    #ts.exaustive_test(False)
    ts.compact_csv_plot_reward(False)
    #ts.compact_csv_plot_ratio(False)
    ts.replace_zeros()
    toc = time.perf_counter()
    print(f"Test in {toc - tic:0.4f} seconds")