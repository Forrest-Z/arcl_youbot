import TSP_DP_new as TSP_DP
from time import time
import matplotlib.pyplot as plt
import math
import numpy as np
import experiment_test

def log_the_list(raw_list):
    log_list = raw_list
    num_of_list = len(raw_list)
    for i in range(num_of_list):
        log_list[i] = math.log(raw_list[i])
    return log_list

computation_time_DP = {}
costs_DP = {}
computation_time_greedy = {}
costs_greedy = {}
computation_time_DP_improvement = {}
costs_DP_improvement = {}

computation_time_gtsp = {}
# computation_time_gtsp_old = {}

obj_nums = [3]
iteration_times = 1
for n in obj_nums:
    computation_time_DP[n] = []
    costs_DP[n] = []
    computation_time_greedy[n] = []
    costs_greedy[n] = []
    computation_time_DP_improvement[n] = []
    costs_DP_improvement[n] = []

    computation_time_gtsp[n] = []
    # computation_time_gtsp_old[n] = []

    for iter in range(iteration_times):
        print("n", n)
        samples = experiment_test.Workspace_Objects(n)
        obj_list = samples.objs
        # usage
        DP = TSP_DP.DP_solver(obj_list)
        DP.experiment()

        
        
        
        
        '''
        start_DP = time()
        print("n", n)
        samples = experiment_test.Workspace_Objects(n)
        DP = TSP_DP.DP_solver(samples.objs)


        start = time() # Intialization time
        # DP.init_workplace() # initiation, exits and objects
        stop = time()
        print("Initiation Time:", str(stop-start), "seconds")
        
        
        start = time()
        DP.dynamic_programming() # dynamic programming framework
        stop = time()
        print("DP_time", str(stop-start), "seconds")

        print("time_member_generation", DP.time_member_generation)
        print("time_subgroup_generation", DP.time_subgroup_generation)
        print("time_distance_computation", DP.time_distance_computation)
        print("time_vsg", DP.time_vsg)
        print("time_gtsp", DP.time_gtsp)
        # print("time_gtsp_old", DP.time_gtsp_old)

        start = time()
        DP.construct_paths()
        stop = time()
        print("path_construction", str(stop-start), "seconds")

        computation_time_DP[n].append(stop-start_DP-DP.time_vsg_improvement)
        costs_DP[n].append(DP.cost)
        print("DP Computing Time:", str(stop-start_DP), "seconds")
        print("cost", DP.cost)

        computation_time_DP_improvement[n].append(stop-start_DP)
        costs_DP_improvement[n].append(DP.improved_cost)
        print("DP Improvement Computing Time:", str(stop-start_DP), "seconds")
        print("cost", DP.improved_cost)

        print "obj_order", DP.DP_obj_order
        print "pickup points", DP.DP_pickup_points
        print "robot_locations", DP.DP_robot_locations

        start = time()
        length = DP.greedy_heuristics()
        stop = time()
        print("greedy", str(stop-start), "seconds")

        computation_time_greedy[n].append(stop-start)
        costs_greedy[n].append(length)

        print "obj_order", DP.greedy_obj_order
        print "pickup points", DP.greedy_pickup_points
        print "robot_locations", DP.greedy_robot_locations

        computation_time_gtsp[n].append(DP.time_gtsp)
        # computation_time_gtsp_old[n].append(DP.time_gtsp_old)
        
        # print "DP.paths", DP.paths
        # print "DP.improved_paths", DP.paths_improved

        # red lines are objects blue lines are paths.
        
        for obj in DP.objs:
            plt.plot([obj.x_min, obj.x_max], [obj.a*obj.x_min+obj.b, obj.a*obj.x_max+obj.b], color='r')
        for path in DP.paths:
            for line in path:
                Xs = []
                Ys = []
                for point in line:
                    Xs.append(point.x)
                    Ys.append(point.y)
                plt.plot(Xs, Ys, color='b')
        for line in DP.greedy_paths:
            Xs = []
            Ys = []
            for point in line:
                Xs.append(point.x)
                Ys.append(point.y)
            plt.plot(Xs, Ys, color='orange')
        plt.show()
        '''
        '''
        for obj in DP.objs:
            plt.plot([obj.x_min, obj.x_max], [obj.a*obj.x_min+obj.b, obj.a*obj.x_max+obj.b], color='r')
        for path in DP.pointwise_paths:
            for i in range(len(path)-1):
                plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], color='b')
        plt.show()
        '''
'''
print("DP", computation_time_DP)
print("greedy", computation_time_greedy)
print("DP_Improved", computation_time_DP_improvement)

computation_time_DP_average = []
costs_DP_average = []

for n in obj_nums:
    computation_time_DP_average.append(np.average(computation_time_DP[n]))
    costs_DP_average.append(np.average(costs_DP[n]))

computation_time_greedy_average = []
costs_greedy_average = []

for n in obj_nums:
    computation_time_greedy_average.append(np.average(computation_time_greedy[n]))
    costs_greedy_average.append(np.average(costs_greedy[n]))

computation_time_DP_improvement_average = []
costs_DP_improvement_average = []

for n in obj_nums:
    computation_time_DP_improvement_average.append(np.average(computation_time_DP_improvement[n]))
    costs_DP_improvement_average.append(np.average(costs_DP_improvement[n]))


computation_time_gtsp_average = []

for n in obj_nums:
    computation_time_gtsp_average.append(np.average(computation_time_gtsp[n]))
'''
'''
computation_time_gtsp_old_average = []

for n in obj_nums:
    computation_time_gtsp_old_average.append(np.average(computation_time_gtsp_old[n]))
'''
'''
plt.clf() 
plt.cla() 
plt.title('Computation Time(k=3)')
plt.plot(obj_nums, log_the_list(computation_time_DP_average), label='DP')
plt.plot(obj_nums, log_the_list(computation_time_greedy_average), label='greedy')
plt.plot(obj_nums, log_the_list(computation_time_DP_improvement_average), label='DP_improvement')
plt.legend()
plt.xlabel('obj numbers')
plt.ylabel('Time')
# plt.savefig('/home/kai/Documents/Kai Gao/Clutter Removal/codes/P3/time_k31126_short.png')
plt.show()

plt.clf() 
plt.cla() 
plt.title('Cost(k=3)')
plt.plot(obj_nums, costs_DP_average, label='DP')
plt.plot(obj_nums, costs_greedy_average, label='greedy')
plt.plot(obj_nums, costs_DP_improvement_average, label='DP_Improvement')
plt.legend()
plt.xlabel('obj numbers')
plt.ylabel('Cost')
# plt.savefig('/home/kai/Documents/Kai Gao/Clutter Removal/codes/P3/cost_k31126_short.png')
plt.show()
'''

'''
plt.clf() 
plt.cla() 
plt.title('Computation Time(k=3)')
plt.plot(obj_nums, log_the_list(computation_time_gtsp_average), label='Improved')
plt.plot(obj_nums, log_the_list(computation_time_gtsp_old_average), label='Version-11.19')
plt.legend()
plt.xlabel('obj numbers')
plt.ylabel('Time')
plt.savefig('C:\\Users\\kg627\\Desktop\\Rutgers\\Robotics\\Clutter Removal\\codes\\P3\\gstp_improvement.png')
#plt.show()
'''