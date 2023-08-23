# IMPORTS
import time
import sympy as sp
import algorithms as alg
import numpy as np
import inputs as ipp
import pickle
import tests as ts
import ILP as ilp

# - MAIN -
if __name__ == '__main__':
    ####################################
    ####################################
    ####################################
    # Use the following bunch of lines for visualizing the structure of a problem instance.
    
    # name = "problems/exaustive_test/problem_n" + str(10) + "_t" + str(0) + "_h" + str(
    #     20) + ".dat"
    # file = open(name, 'rb')
    # instances = pickle.load(file)
    # print("Instance Example", instances)
    
    ####################################
    ####################################
    ####################################
    
    ####################################
    ####################################
    ####################################
    # Use the following function for generating new problem instances. Check `inputs.py` for function parameters.
    
    # ipp.generate_problem_instance(True)
    
    ####################################
    ####################################
    ####################################

    ####################################
    ####################################
    ####################################
    # The following lines contain a running example of RSEO single drone.

    #ex_instance = [[[sp.Point2D(0, 0), {0}], [sp.Point2D(-20, -1), {1}], [sp.Point2D(-30, -0.), {2}],
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
    #output = alg.RSEO(a[0], a[3], a[4], a[5], a[6], 2500000, 16000, True)
    ####################################
    ####################################
    ####################################

    ####################################
    ####################################
    ####################################
    # The following lines contain some calling test function example.
    
    # print("ALTITUDE SINGLE DRONE SCENARIO:")
    # ts.altitude_test(False, False)
    #print("EXHAUSTIVE SINGLE DRONE SCENARIO:")
    # ts.exaustive_test(False)
    # print("EXHAUSTIVE MULTI DRONE SCENARIO:")
    # ts.exaustive_multi_test(False)
    # ts.compact_csv_plot_ratio_altitude(False)
    # ts.compact_csv_plot_reward(False)
    # #ts.compact_csv_plot_ratio(False)
    # ts.replace_zeros()
    ####################################
    ####################################
    ####################################

