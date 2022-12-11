# IMPORTS
import time
import sympy as sp
import algorithms as alg
import numpy as np
import inputs as ipp
import pickle
import tests as ts

# - MAIN -
if __name__ == '__main__':
    tic = time.perf_counter()
    name = "problems/exaustive_test/problem_n" + str(10) + "_t" + str(0) + "_h" + str(
        20) + ".dat"
    file = open(name, 'rb')
    instances = pickle.load(file)
    a = instances[0]
    # file = open("app.dat", 'rb')
    # a = pickle.load(file)

    # ipp.generate_problem_instance(True)

    # exit()
    # a = [[[sp.Point2D(0, 0), {0}], [sp.Point2D(-20, -1), {1}], [sp.Point2D(-30, -0.), {2}],
    #       [sp.Point2D(-0.2, -0.05), {1, 2}], [sp.Point2D(-80., -0.7), {1, 2}]],
    #      [[sp.Point2D(-20, -1), {1}], [sp.Point2D(-30, -0.), {2}]],
    #      [[sp.Point2D(-0.2, -0.05), {1, 2}], [sp.Point2D(-80., -0.7), {1, 2}]], np.array([0, 5, 9]),
    #      np.array([0, 968, 445]), np.array([[0., 100000.53522477, 100000.64059647, 0.73234208,
    #                                          100000.37307297],
    #                                         [100000.53522477, 0., 100000.3126132, 0.51446583,
    #                                          100000.51446583],
    #                                         [100000.64059647, 100000.3126132, 0., 0.63489234,
    #                                          100000.63489234],
    #                                         [0.73234208, 100000.51446583, 0.63489234, 0.,
    #                                          100000.50813822],
    #                                         [100000.37307297, 100000.51446583, 0.63489234, 100000.50813822,
    #                                          0.]]), np.array([0., 75288.88888889, 34611.11111111])]
    # for i in range(len(a[0])):
    #      print("WAYPOINT: ",i ," ", sp.N(a[0][i][0]))
    #      for j in a[0][i][1]:
    #          print("SENSOR: ",j, " STORAGE: ",a[4][j], " HOVERING: ",a[6][j], " REWARD: ", a[3][j])
    # print()
    # ipp.generate_problem_instance_altitude(False, True)

    # print("SAME SCENARIO:")
    # ts.altitude_test(True, False)

    # print("DIFFERENT SCENARIO:")
    # ts.altitude_test(False, False)

    # # for i in range(len(a[0])):
    # #     for j in range(len(a[0])):
    # #         print("EDGE ", i, " , ",j," cost: ",a[5][i][j])
    #output = ilp.opt_ilp_cplex(a[0], 2500000, 44, a[3], a[4], a[5], a[6], True)
    #output = ilp.opt_multi_ilp_cplex(a[0], 126000, 1000, a[3], a[4], a[5], a[6], 2, True)
    #output = ilp.opt_ilp_cplex(a[0], 2500000, 16000, a[3], a[4], a[5], a[6],  True)
    #output = alg.RSEO(a[0], a[3], a[4], a[5], a[6], E[0], S[0], True)
    # print(output)
    # output = alg.multiRSEO(a[0], a[3], a[4], a[5], a[6], 250000, 4518, 2, True, True)
    # print(output)
    # print(ipp.generate_problem_instance_altitude(True, True))4 437 403

    # output = alg.APX_partion(a[0], a[3], a[4], a[5], a[6], 5000000, 2000, 2, 0, True)
    # print(output)
    # exit()
    #output = alg.multiMRE(a[0], a[3], a[4], a[5], a[6], 2500000, 1000,3, False)
    #print(output)
    #output = alg.MRS(a[0], a[3], a[4], a[5], a[6], 2500000, 1000, True)
    #print("1. ",output)
    #output = alg.multiMRS(a[0], a[3], a[4], a[5], a[6], 2500000, 1000, 2, True)
    #print("2. ",output)


    # print("EXHAUSTIVE SCENARIO:")
    # ts.exaustive_test(False)

    print("EXHAUSTIVE MULTI SCENARIO:")
    ts.exaustive_multi_test(False)

    # ts.compact_csv_plot_reward(False)
    # #ts.compact_csv_plot_ratio(False)
    # ts.replace_zeros()
    # toc = time.perf_counter()
    # print(f"Test in {toc - tic:0.4f} seconds")