from time import time
import pyvisgraph as vg
import numpy as np
import random, math
from math import *
import copy
import itertools
import math
from gurobipy import*
import matplotlib.pyplot as plt


XDIM = 2.50
YDIM = 2.50
k = 3.0
MAX_Length = k*(XDIM+YDIM)
robot_radius = 0.43
EXIT = vg.Point(0.0, 0.0)

class  Dynamic_Group(object):
    def __init__(self, index, objs, min_cost_dict, Exit, overlapping_dict):
        self.min_cost_dict=min_cost_dict
        self.exit = Exit
        self.num_discrete = 4
        self.elements = []
        self.paths = []
        self.cost = 0.0
        self.improved_cost = 0.0
        self.first_subgroup_path = []
        self.first_subgroup_path_improved = []
        self.index = index
        self.n = int(floor(log(index, 2)))
        self.subgroup_dictionary = {}
        self.time_member_generation = 0
        self.time_subgroup_generation = 0
        self.time_distance_computation = 0
        self.time_vsg = 0
        self.time_vsg_improvement = 0
        self.time_gtsp = 0
        # self.time_gtsp_old = 0
        self.overlapping_dict = overlapping_dict
        self.first_pick_subgroup = Subgroup()
        self.first_pick_order = []

        # outputs
        self.obj_order = []

        start = time()
        self.generate_members_from_index(objs)
        end = time()
        self.time_member_generation = end - start

        start = time()
        self.generate_subgroups()
        end = time()
        self.time_subgroup_generation = end - start

        start = time()
        self.compute_distance()
        end = time()
        self.time_distance_computation = end - start

    def is_valid_subgroup(self,subgroup_index):
        rest = self.index - subgroup_index
        if rest < 0:
            return False
        if rest == 0:
            return True
        yield_set = set()
        for i in generate_group_member(subgroup_index):
            yield_set = (yield_set | set(self.overlapping_dict[i]))
        for i in generate_group_member(rest):
            if i in yield_set:
                return False
        return True


    def generate_members_from_index(self, objs):
        rest = self.index
        for i in range(1,self.n+1):
            if rest >= 2**(self.n+1-i):
                rest = rest - 2**(self.n+1-i)
                self.elements.append(objs[self.n+1-i-1])

    def generate_subgroups( self):
        subgroups = [] 
        for obj in self.elements:
            if self.is_valid_subgroup(2**(obj.index)):
                subgroup = Subgroup() 
                subgroup.elements.append(obj)
                subgroup.weight = obj.weight
                subgroup.index = 2**(obj.index)
                self.subgroup_dictionary[subgroup.index] = copy.deepcopy(subgroup)
                subgroups.append(subgroup) 
        k_groups = copy.deepcopy( subgroups) 
        while len(k_groups)>0:
            k_groups = self.generate_larger_subgroups( k_groups) 
            subgroups = subgroups + k_groups
        return subgroups

    def generate_larger_subgroups( self,  k_groups):
        new_groups = []
        new_group_indexs = []
        for obj in self.elements:
            for subgroup in k_groups:
                if (subgroup.index//(2**(obj.index)))%2 == 0: #obj not in subgroup
                    new_weight = subgroup.weight + obj.weight 
                    new_index = subgroup.index + 2**obj.index 
                    if new_weight<=1 & (new_index not in new_group_indexs)& self.is_valid_subgroup(new_index): #capacity constraint and check repeat
                        new_group = Subgroup() 
                        new_group.weight = new_weight 
                        new_group.elements = subgroup.elements + [obj]
                        new_group.index = new_index
                        new_groups.append(new_group) 
                        new_group_indexs.append(new_index) 
                        self.subgroup_dictionary[new_group.index] = copy.deepcopy(new_group)
        return new_groups

    def generate_obstacles(self, subgroup):
        rest_elements = []
        for obj in self.elements:
            if not (subgroup.index >> obj.index):
                rest_elements.append(obj)
        return rest_elements

    def compute_distance(self):
        min_distance = MAX_Length*self.index
        for subgroup_index in self.subgroup_dictionary.keys():
            subgroup = self.subgroup_dictionary[subgroup_index]
            all_discrete_points = [[self.exit]]
            for obj in subgroup.elements:
                discrete_points = []
                for i in range(self.num_discrete + 1):
                    x = obj.x_real_min + float(i)/float(self.num_discrete) * (obj.x_real_max - obj.x_real_min)
                    discrete_points.append(vg.Point(x, obj.a * x + obj.b))
                all_discrete_points.append(discrete_points)
            all_discrete_points.append([self.exit])

            # print("points_vg", all_discrete_points)

            rest_elements = self.generate_obstacles(subgroup)
            
            start = time()
            [_, tour_path, tour_order] = self.vsgraph(all_discrete_points, rest_elements)
            end = time()
            self.time_vsg = self.time_vsg + (end - start)
            
            distance = self.path_length_computing(tour_path) + self.min_cost_dict[self.index - subgroup_index][0]
            if distance <= min_distance:
                self.cost = distance
                self.first_pick_subgroup = subgroup
                self.first_pick_order = tour_order
                '''
                path = []
                for i in range(len(tour_path)):
                    if tour_path[i] ==-1:
                        path.append(0)
                    else:
                        path.append(subgroup.elements[tour_path[i]].index)
                '''
                self.first_subgroup_path = copy.deepcopy(tour_path)
                min_distance = distance
                # print "yes"

        # output
        self.order_transformation()

        start = time()
        # [_, self.first_subgroup_path_improved] = self.vsgraph_improvement(self.first_subgroup_path, 1)
        # [improved_distance, _] = self.vsgraph_improvement(self.first_subgroup_path, 1)
        end = time()
        improved_distance = self.path_length_computing(self.first_subgroup_path_improved)
        self.improved_cost = improved_distance + self.min_cost_dict[self.index - self.first_pick_subgroup.index][3]
        self.time_vsg_improvement = self.time_vsg_improvement + (end - start)

    def order_transformation(self):
        for index_in_group in self.first_pick_order:
            self.obj_order.append(self.first_pick_subgroup.elements[index_in_group-1].index)

    def vsgraph_improvement(self, tour_path, polish_index):
    
        all_discrete_points = [[self.exit]]
        for order_index in range(len(self.first_pick_order)):
            obj_index = self.first_pick_order[order_index]
            obj = self.first_pick_subgroup.elements[obj_index-1]
            discrete_points = []
            x_min = tour_path[order_index+1][0].x - (obj.x_max-obj.x_min)/(2.0 * self.num_discrete)
            x_max = tour_path[order_index+1][0].x + (obj.x_max-obj.x_min)/(2.0 * self.num_discrete)
            x_min = max(x_min, obj.x_min)
            x_max = min(x_max, obj.x_max)
            for i in range(self.num_discrete + 1):
                x = x_min + float(i)/float(self.num_discrete) * (x_max - x_min)
                discrete_points.append(vg.Point(x, obj.a * x + obj.b))
            all_discrete_points.append(discrete_points)
        all_discrete_points.append([self.exit])

        print("points_vg_improve", all_discrete_points)

        obstacles = self.generate_obstacles(self.first_pick_subgroup)

        polys = []
        for obstacle in obstacles:
            polys.append([vg.Point(obstacle.x_min, obstacle.a * obstacle.x_min + obstacle.b), vg.Point(obstacle.x_max, obstacle.a * obstacle.x_max + obstacle.b)])
        g = vg.VisGraph()
        g.build(polys)

        Distance = {}
        num_objs = len(all_discrete_points) - 2
        for i in range( 1, num_objs*(self.num_discrete+1)+1):
            for j in range(1, i):
                path = g.shortest_path(all_discrete_points[(i-1)//(self.num_discrete+1) + 1][(i-1)%(self.num_discrete+1)], all_discrete_points[(j - 1)//(self.num_discrete+1) + 1][(j - 1)%(self.num_discrete+1)])
                path_length = 0.0
                for i_point in range(len(path)-1):
                    path_length = path_length + sqrt((path[i_point].x - path[i_point+1].x)**2+(path[i_point].y - path[i_point+1].y)**2)
                Distance[i,j] = path_length
                Distance[j,i] = path_length
            Distance[i, i] = 0.0

        for i in range(1,num_objs*(self.num_discrete+1) + 1):
            path = g.shortest_path(all_discrete_points[(i-1)//(self.num_discrete+1) + 1][(i-1)%(self.num_discrete+1)], all_discrete_points[0][0])
            path_length = 0.0
            for i_point in range(len(path)-1):
                path_length = path_length + sqrt((path[i_point].x - path[i_point+1].x)**2+(path[i_point].y - path[i_point+1].y)**2)
            Distance[i,0] = Distance[0,i] = Distance[ i,num_objs*(self.num_discrete+1) + 1] = Distance[num_objs*(self.num_discrete+1) + 1, i] = path_length
        Distance[0, 0] = Distance[0,num_objs*(self.num_discrete+1) + 1] = Distance[num_objs*(self.num_discrete+1) + 1, 0] = Distance[num_objs*(self.num_discrete+1) + 1, num_objs*(self.num_discrete+1) + 1] = 0.0
        
        [length, flow_path] = self.minflow(range(len(self.first_pick_order)), Distance)

        # print("distance_improvement", Distance)

        vs_path = [g.shortest_path(self.exit, all_discrete_points[flow_path[1]//(self.num_discrete+1)+1][flow_path[1]%(self.num_discrete+1)])]
        for i in range(1, len(flow_path)-2):
            vs_path.append(g.shortest_path(all_discrete_points[flow_path[i]//(self.num_discrete+1)+1][flow_path[i]%(self.num_discrete+1)], all_discrete_points[flow_path[i+1]//(self.num_discrete+1)+1][flow_path[i+1]%(self.num_discrete+1)]))
        vs_path.append(g.shortest_path(all_discrete_points[flow_path[-2]//(self.num_discrete+1)+1][flow_path[-2]%(self.num_discrete+1)], self.exit))
        return[length, vs_path]


    def minflow(self, order, distance):
        a = []
        for i in range(len(order)*(self.num_discrete+1)+2):
            a.append([])
            for j in range(len(order)*(self.num_discrete+1)+2):
                a[i].append(0.0)
        for i in range(self.num_discrete+1):
            a[0][i + order[0]*(self.num_discrete+1) + 1] = distance[0, i + order[0]*(self.num_discrete+1) + 1]
        for i in range(len(order)-1):
            for j in range(self.num_discrete+1):
                for l in range(self.num_discrete+1):
                    a[j + order[i]*(self.num_discrete+1) + 1][l + order[i+1]*(self.num_discrete+1) + 1] = distance[ j + order[i]*(self.num_discrete+1) + 1, l + order[i+1]*(self.num_discrete+1) + 1]
        for i in range(self.num_discrete+1):
            a[i + order[-1]*(self.num_discrete+1) + 1][-1] = distance[i + order[-1]*(self.num_discrete+1) + 1, 0]
        

        m = Model()
        m.setParam('OutputFlag', 0)
        m.modelSense=GRB.MINIMIZE

        fij = m.addVars(len(order)*(self.num_discrete+1)+2, len(order)*(self.num_discrete+1)+2, vtype = GRB.INTEGER,ub=1, lb=-1, name = 'f_ij')
        
        m.setObjective(sum(a[i][j]*fij[i,j] for i in range(len(order)*(self.num_discrete+1)+2) for j in range(len(order)*(self.num_discrete+1)+2)))

        for i in range(len(order)*(self.num_discrete+1)+2):
            for j in range(len(order)*(self.num_discrete+1)+2):
                if a[i][j]>0.0:
                    m.addConstr(fij[i,j]<=1)
                else:
                    m.addConstr(fij[i,j]<=0)
                m.addConstr(fij[i,j]+fij[j,i]==0)
            if (i!=0) & (i!=(len(order)*(self.num_discrete+1)+1)):
                m.addConstr(sum(fij[i,j] for j in range(len(order)*(self.num_discrete+1)+2))==0)
        m.addConstr(sum(fij[0, i] for i in range(len(order)*(self.num_discrete+1)+2))==1)
        m.addConstr(sum(fij[i, len(order)*(self.num_discrete+1)+1] for i in range(len(order)*(self.num_discrete+1)+2))==1)

        m.optimize()
        obj = m.getObjective()
        '''
        print("order", order)
        print("flow", fij)
        print("a", a)
        print("distance", distance)
        '''
        path = self.path_construction_flow(fij, len(order))
        return [obj.getValue(), path]

    def path_construction_flow(self, fij, num_objs):
        path = [-1]
        for i in range(num_objs):
            point = path[-1]+1
            for j in range(1, num_objs*(self.num_discrete+1)+1):
                if fij[ point,j].x > 0.5:
                    path.append(j-1)
                    break
        path.append(-1)
        # print("path", path)
        return path


    def vsgraph(self, all_discrete_points, obstacles): # compute the min cost to pick up all the objs in the subgroup with the existance of the obstacles.
        polys = []
        for obstacle in obstacles:
            half_thickness = robot_radius
            delta_x = half_thickness*math.cos(math.atan(-1.0/obstacle.a))
            delta_y = half_thickness*math.sin(math.atan(-1.0/obstacle.a))
            # polys.append([vg.Point(obstacle.x_min, obstacle.a * obstacle.x_min + obstacle.b), vg.Point(obstacle.x_max, obstacle.a * obstacle.x_max + obstacle.b), vg.Point(obstacle.x_max + 1, obstacle.a * obstacle.x_max + obstacle.b + 1), vg.Point(obstacle.x_min + 1, obstacle.a * obstacle.x_min + obstacle.b + 1)])
            polys.append([vg.Point(obstacle.x_min+delta_x, obstacle.a * obstacle.x_min +delta_y + obstacle.b), vg.Point(obstacle.x_max+delta_x, obstacle.a * obstacle.x_max + delta_y + obstacle.b), vg.Point(obstacle.x_max-delta_x, obstacle.a * obstacle.x_max - delta_y + obstacle.b), vg.Point(obstacle.x_min - delta_x, obstacle.a * obstacle.x_min - delta_y + obstacle.b)])
        g = vg.VisGraph()
        g.build(polys)

        Distance = {}
        num_objs = len(all_discrete_points) - 2
        for i in range( 1, num_objs*(self.num_discrete+1)+1 + 1):
            for j in range(1, i):
                path = g.shortest_path(all_discrete_points[(i-1)//(self.num_discrete+1) + 1][(i-1)%(self.num_discrete+1)], all_discrete_points[(j - 1)//(self.num_discrete+1) + 1][(j - 1)%(self.num_discrete+1)])
                path_length = 0.0
                for i_point in range(len(path)-1):
                    path_length = path_length + sqrt((path[i_point].x - path[i_point+1].x)**2+(path[i_point].y - path[i_point+1].y)**2)
                Distance[i,j] = path_length
                Distance[j,i] = path_length
            Distance[i, i] = 0.0

        for i in range(1,num_objs*(self.num_discrete+1) + 1):
            path = g.shortest_path(all_discrete_points[(i-1)//(self.num_discrete+1) + 1][(i-1)%(self.num_discrete+1)], all_discrete_points[0][0])
            path_length = 0.0
            for i_point in range(len(path)-1):
                path_length = path_length + sqrt((path[i_point].x - path[i_point+1].x)**2+(path[i_point].y - path[i_point+1].y)**2)
            Distance[i,0] = Distance[0,i] = Distance[ i,num_objs*(self.num_discrete+1) + 1] = Distance[num_objs*(self.num_discrete+1) + 1, i] = path_length
        Distance[0, 0] = Distance[0,num_objs*(self.num_discrete+1) + 1] = Distance[num_objs*(self.num_discrete+1) + 1, 0] = Distance[num_objs*(self.num_discrete+1) + 1, num_objs*(self.num_discrete+1) + 1] = 0.0
        
        # print("distance", Distance)

        start = time()
        [length, gtsp_path] = self.generalized_tsp(Distance, num_objs)
        end = time()
        self.time_gtsp = self.time_gtsp + (end-start)

        # print("gtsp_path", gtsp_path)

        '''
        start = time()
        [length, gtsp_path] = self.generalized_tsp_old(Distance, num_objs)
        end = time()
        self.time_gtsp_old = self.time_gtsp_old + (end-start)
        '''

        gtsp_pick_up_order = [gtsp_path[1]//(self.num_discrete+1)+1]
        vs_path = [g.shortest_path(self.exit, all_discrete_points[gtsp_path[1]//(self.num_discrete+1)+1][gtsp_path[1]%(self.num_discrete+1)])]
        for i in range(1, len(gtsp_path)-2):
            gtsp_pick_up_order.append(gtsp_path[i+1]//(self.num_discrete+1)+1)
            vs_path.append(g.shortest_path(all_discrete_points[gtsp_path[i]//(self.num_discrete+1)+1][gtsp_path[i]%(self.num_discrete+1)], all_discrete_points[gtsp_path[i+1]//(self.num_discrete+1)+1][gtsp_path[i+1]%(self.num_discrete+1)]))
        vs_path.append(g.shortest_path(all_discrete_points[gtsp_path[-2]//(self.num_discrete+1)+1][gtsp_path[-2]%(self.num_discrete+1)], self.exit))
        # [vs_path, gtsp_pick_up_order] = self.path_order_modification(vs_path, gtsp_pick_up_order)
        return[length, vs_path, gtsp_pick_up_order]

    '''
    def path_order_modification(self, raw_paths, order):
        paths = []
        for raw_path in raw_paths:
            path = []
            for i in range(len(raw_path)-1):
                line = [raw_path[i], raw_path[i+1]]
                path.append(line)
            paths.append(path)
        for i in range(len(order)):
            path = paths[i]
            for line_i in range(len(path)):
                line = path[line_i]
                r1 = (line[0].x, line[0].y)
                r2 = (line[1].x, line[1].y)
                for j in range(i+1, len(order)):
                    l1 = (self.elements[j-1].x_min, self.elements[j-1].a * self.elements[j-1].x_min + self.elements[j-1].b)
                    l2 = (self.elements[j-1].x_max, self.elements[j-1].a * self.elements[j-1].x_max + self.elements[j-1].b)
                    if self.line_intersection_check(r1, r2, l1, l2):
                        l1 = (self.elements[j-1].x_real_min, self.elements[j-1].a * self.elements[j-1].x_real_min + self.elements[j-1].b)
                        l2 = (self.elements[j-1].x_real_max, self.elements[j-1].a * self.elements[j-1].x_real_max + self.elements[j-1].b)
                        alpha_l = (l1[1]*(r2[0]-r1[0]) - r1[1]*(r2[0] - l1[0]) + r2[1]*(r1[0] - l1[0]))/((r2[1]-r1[1])*(l2[0]-l1[0]) - (l2[1]-l1[1])*(r2[0]-r1[0]))
                        point = vg.Point(l1[0]+alpha_l*(l2[0]-l1[0]), l1[1]+alpha_l*(l2[1]-l1[1]))
                        path1 = copy.deepcopy(path[0:line_i]) + copy.deepcopy([[line[0],point]])
                        path2 = copy.deepcopy([[point,line[1]]]) + copy.deepcopy(path[line_i+1:])
                        new_path_j = copy.deepcopy(paths[j]) + copy.deepcopy(paths[j+1])
                        paths = copy.deepcopy(paths[0:i]) + copy.deepcopy([path1, path2]) + copy.deepcopy(paths[i+1:j]) + [copy.deepcopy(new_path_j)] + copy.deepcopy(paths[j+2:])
                        order = order[0:i] + [j] + order[i:j] + order[j+1:]
                        break
        new_raw_paths = []
        print paths
        for path in paths:
            print path[0]
            raw_path = [path[0][0]]
            for line in path:
                raw_path.append(line[1])
            new_raw_paths.append(raw_path)

        return [new_raw_paths, order]
    '''

    def line_intersection_check(self, r1, r2, l1, l2):
        epsilon = 0.001        
        if ((l2[1]-l1[1])*(r2[0]-r1[0]))==((r2[1]-r1[1])*(l2[0]-l1[0])):
            return False
        # l1 + alpha_l...
        alpha_l = (l1[1]*(r2[0]-r1[0]) - r1[1]*(r2[0] - l1[0]) + r2[1]*(r1[0] - l1[0]))/((r2[1]-r1[1])*(l2[0]-l1[0]) - (l2[1]-l1[1])*(r2[0]-r1[0]))
        if (alpha_l>=0-epsilon) & (alpha_l<=1+epsilon):
            alpha_r = (r1[1]*(l2[0]-l1[0]) - l1[1]*(l2[0] - r1[0]) + l2[1]*(l1[0] - r1[0]))/((l2[1]-l1[1])*(r2[0]-r1[0]) - (r2[1]-r1[1])*(l2[0]-l1[0]))
            if (alpha_r>=0-epsilon) & (alpha_r<=1+epsilon):
                return True
        return False

    def generalized_tsp(self, distance, num_objs):
        
        # m=Model("VRP")
        m = Model()
        m.setParam('OutputFlag', 0)
        m.modelSense=GRB.MINIMIZE

        si = m.addVars(num_objs*(self.num_discrete+1), vtype = GRB.BINARY, name = 's_i')
        ti = m.addVars(num_objs*(self.num_discrete+1), vtype = GRB.BINARY, name = 't_i')
        ui = m.addVars(num_objs*(self.num_discrete+1), vtype = GRB.INTEGER, name = 'u_i')
        xij=m.addVars(num_objs*(self.num_discrete+1), num_objs*(self.num_discrete+1),vtype=GRB.BINARY,name='X_ij')
        y_i_in = m.addVars(num_objs*(self.num_discrete+1) , vtype=GRB.BINARY, name='y_i_in')
        y_i_out = m.addVars(num_objs*(self.num_discrete+1), vtype=GRB.BINARY, name='y_i_out')

        m.setObjective(sum(distance[i+1,j+1]*xij[i,j] for i in range(num_objs*(self.num_discrete+1)) for j in range(num_objs*(self.num_discrete+1)))+sum(distance[0,k+1]*si[k] for k in range(num_objs*(self.num_discrete+1)))+sum(distance[l+1,0]*ti[l] for l in range( num_objs*(self.num_discrete+1))) )

        m.addConstr(sum(si[i] for i in range(num_objs*(self.num_discrete+1))) == 1)
        m.addConstr(sum(ti[i] for i in range(num_objs*(self.num_discrete+1))) == 1)


        for i in range(num_objs*(self.num_discrete+1)):
            m.addConstr(sum(xij[j,i] for j in range(num_objs*(self.num_discrete+1))) + si[i]==y_i_in[i])
            m.addConstr(sum(xij[i,j] for j in range(num_objs*(self.num_discrete+1))) + ti[i]==y_i_out[i])
            m.addConstr(y_i_in[i]==y_i_out[i])
            m.addConstr(xij[i,i]==0)


        for i in range(num_objs):
            m.addConstr(sum(y_i_out[j] for j in range(i*(self.num_discrete+1), (i+1)*(self.num_discrete+1)))>=1)
        
        for i in range(num_objs*(self.num_discrete+1)):
            m.addConstr(ui[i]>=2)
            m.addConstr(ui[i]<=num_objs*(self.num_discrete+1)+1)
            for j in range(num_objs*(self.num_discrete+1)):
                m.addConstr( ui[i]-ui[j]+1<=(num_objs*(self.num_discrete+1))*(1-xij[i,j]))

        for i in range(num_objs):
            for j in range(num_objs):
                if i == j:
                    continue
                if self.elements[j].index in self.overlapping_dict[self.elements[i].index]:
                    for point_i in range((self.num_discrete+1)*i, (self.num_discrete+1)*(i+1)):
                        for point_j in range((self.num_discrete+1)*j, (self.num_discrete+1)*(j+1)):
                            m.addConstr(ui[point_i]>=ui[point_j])
                            # print "constraint", num_objs, self.elements[i].index, point_i, self.elements[j].index, point_j

        '''
        for num_range in self.set_range(num_objs):
            for size in range(1, len(num_range)+1):
                for T_points in tuple(itertools.combinations(num_range, size)):
                    m.addConstr((sum(xij[i,j] for i in T_points for j in T_points) + sum(si[i] for i in T_points) + sum(ti[i] for i in T_points))<= len(T_points)+1)
        for size in range(1, num_objs*(self.num_discrete+1)+1):
            for T_points in tuple(itertools.combinations(range(num_objs*(self.num_discrete+1)), size)):
                m.addConstr(sum(xij[i,j] for i in T_points for j in T_points)<=len(T_points)-1)
        '''
        m.optimize()
        obj = m.getObjective()
        '''
        dist = 0
        for i in range(num_objs*(self.num_discrete+1)):
            for j in range(num_objs*(self.num_discrete+1)):
                for k in range(num_objs*(self.num_discrete+1)):
                    for l in range( num_objs*(self.num_discrete+1)):
                        dist = dist + (distance[i+1,j+1]*xij[i,j].x)+(distance[0,k+1]*si[k].x)+(distance[l+1,0]*ti[l].x)
                        print('xij', xij[i,j].x, 'distance', distance[i+1,j+1], 'si', si[k].x, 'distance', distance[0,k+1], 'ti', ti[l].x, 'dist', distance[l+1,0])
        print('dist', dist)
        '''

        path = self.path_construction(xij, si, ti)
        return [obj.getValue(), path]

    def generalized_tsp_old(self, distance, num_objs):
        
        # m=Model("VRP")
        m = Model()
        m.setParam('OutputFlag', 0)
        m.modelSense=GRB.MINIMIZE

        si = m.addVars(num_objs*(self.num_discrete+1), vtype = GRB.BINARY, name = 's_i')
        ti = m.addVars(num_objs*(self.num_discrete+1), vtype = GRB.BINARY, name = 't_i')
        xij=m.addVars(num_objs*(self.num_discrete+1), num_objs*(self.num_discrete+1),vtype=GRB.BINARY,name='X_ij')
        y_i_in = m.addVars(num_objs*(self.num_discrete+1) , vtype=GRB.BINARY, name='y_i_in')
        y_i_out = m.addVars(num_objs*(self.num_discrete+1), vtype=GRB.BINARY, name='y_i_out')

        m.setObjective(sum(distance[i+1,j+1]*xij[i,j] for i in range(num_objs*(self.num_discrete+1)) for j in range(num_objs*(self.num_discrete+1)))+sum(distance[0,k+1]*si[k] for k in range(num_objs*(self.num_discrete+1)))+sum(distance[l+1,0]*ti[l] for l in range( num_objs*(self.num_discrete+1))) )

        m.addConstr(sum(si[i] for i in range(num_objs*(self.num_discrete+1))) == 1)
        m.addConstr(sum(ti[i] for i in range(num_objs*(self.num_discrete+1))) == 1)


        for i in range(num_objs*(self.num_discrete+1)):
            m.addConstr(sum(xij[j,i] for j in range(num_objs*(self.num_discrete+1))) + si[i]==y_i_in[i])
            m.addConstr(sum(xij[i,j] for j in range(num_objs*(self.num_discrete+1))) + ti[i]==y_i_out[i])
            m.addConstr(y_i_in[i]==y_i_out[i])
            m.addConstr(xij[i,i]==0)


        for i in range(num_objs):
            m.addConstr(sum(y_i_out[j] for j in range(i*(self.num_discrete+1), (i+1)*(self.num_discrete+1)))>=1)
            
        for num_range in self.set_range(num_objs):
            for size in range(1, len(num_range)+1):
                for T_points in tuple(itertools.combinations(num_range, size)):
                    m.addConstr((sum(xij[i,j] for i in T_points for j in T_points) + sum(si[i] for i in T_points) + sum(ti[i] for i in T_points))<= len(T_points)+1)
        for size in range(1, num_objs*(self.num_discrete+1)+1):
            for T_points in tuple(itertools.combinations(range(num_objs*(self.num_discrete+1)), size)):
                m.addConstr(sum(xij[i,j] for i in T_points for j in T_points)<=len(T_points)-1)

        m.optimize()
        obj = m.getObjective()
        '''
        dist = 0
        for i in range(num_objs*(self.num_discrete+1)):
            for j in range(num_objs*(self.num_discrete+1)):
                for k in range(num_objs*(self.num_discrete+1)):
                    for l in range( num_objs*(self.num_discrete+1)):
                        dist = dist + (distance[i+1,j+1]*xij[i,j].x)+(distance[0,k+1]*si[k].x)+(distance[l+1,0]*ti[l].x)
                        print('xij', xij[i,j].x, 'distance', distance[i+1,j+1], 'si', si[k].x, 'distance', distance[0,k+1], 'ti', ti[l].x, 'dist', distance[l+1,0])
        print('dist', dist)
        '''

        path = self.path_construction(xij, si, ti)
        return [obj.getValue(), path]

    def path_construction(self, xij, si, ti):
        num_objs = int(len(si)/(self.num_discrete+1))
        path = [-1]
        for i in range(num_objs*(self.num_discrete+1) ):
            if si[i].x>0.5:
                path.append(i)
                break
        for i in range(num_objs - 1):
            point = path[-1]
            for j in range(num_objs*(self.num_discrete+1)):
                if xij[ point,j].x > 0.5:
                    path.append(j)
                    break
        path.append(-1)
        return path

    def path_length_computing(self,path):
        length = 0.0
        for line in path:
            for i in range(len(line)-1):
                length = length + math.sqrt((line[i].x - line[i+1].x)**2+(line[i].y - line[i+1].y)**2)
        return length

    def set_range(self, num_objs):
        for obj in range(num_objs):
            if obj==0:
                yield range((self.num_discrete+1), num_objs*(self.num_discrete+1))
            if obj == num_objs-1:
                yield range((num_objs-1)*(self.num_discrete+1))
            yield range(obj*(self.num_discrete+1)) + range((obj+1)*(self.num_discrete+1), num_objs*(self.num_discrete+1))

                
class Workplace_Object(object):
    def __init__(self):
        self.weight = 0.0
        self.a = 0.0
        self.b = 0.0
        self.x_min = 0.0
        self.x_max = 0.0
        self.x_real_min = 0.0
        self.x_real_max = 0.0
        self.index = 0
        # self.P1 = vg.Point(0,0)
        # self.P2 = vg.Point(0,0)

    def random_stick(self):
        length = min( XDIM, YDIM)/10.0
        x = (XDIM - 2*length)*random.random()+length
        y = (YDIM - 2*length)*random.random()+length
        self.a = tan(2*math.pi*random.random())
        self.b = y-self.a*x
        self.x_min = x
        self.x_max = x + length/sqrt(1+self.a**2)
        # self.P1 = vg.Point(self.x_min, self.a*self.x_min+self.b)
        # self.P2 = vg.Point(self.x_max, self.a*self.x_max+self.b)

def compute_distance(p1, p2):
    dim = len(p1)
    dist = 0
    for i in range(0,dim):
        dist = dist + sqrt(p1[i]**2 + p2[i]**2)
    return dist

class Subgroup(object):
    def __init__(self):
        self.weight = 0
        self.elements = []
        self.min_cost = 0
        self.path = []
        self.index = 0
        self.paraX = []
        self.pointwise_path = []

    def add_element(self, obj):
        self.elements.append(obj)
        self.weight = self.weight + obj.weight
        self.index = self.index + 2**(obj.index)

    def compute_all_possibilities(self, exit):
        orders = list(itertools.permutations([i for i in range(len(self.elements))], len(self.elements)))
        optimal_order = orders[0]
        self.min_cost = 3*len(self.elements)*(XDIM+YDIM)
        self.paraX = [0.0 for i in range(len(self.elements))]
        for order in orders:
            optimization_result = self.compute_min_cost(order, exit)
            cost = optimization_result[0]
            if cost < self.min_cost:
                self.min_cost = cost
                optimal_order = order
                self.paraX = optimization_result[1]
        self.path.append(0)
        self.pointwise_path.append([exit.x, exit.y])
        for i in range(len(self.elements)):
            self.path.append(self.elements[optimal_order[i]].index)
            self.pointwise_path.append([self.paraX[i], self.elements[optimal_order[i]].a * self.paraX[i] + self.elements[optimal_order[i]].b])
        self.path.append(0)
        self.pointwise_path.append([exit.x, exit.y])
        
    def compute_min_cost(self, order, exit):
        X = [0 for i in range(len(order))]
        for i in range(len(X)):
            X[i] = self.elements[order[i]].x_min
        iteration_times = 0
        while 1 & (iteration_times<10**2):
            loss_old = self.obj_function(order, X, exit)
            df = self.d_obj_function(order, X, exit)
            alpha = 10.0
            loss_new = self.obj_function(order, [X[i]-alpha*df[i] for i in range(len(X))], exit)
            if loss_new == loss_old:
                X = self.X_modification(order, X)
                break
            while (loss_old <= loss_new) & (alpha > 2**(-5)):
                alpha = alpha/2
                loss_new = self.obj_function(order, [X[i]-alpha*df[i] for i in range(len(X))], exit)
            X = [X[i]-alpha*df[i] for i in range(len(X))]
            X = self.X_modification(order, X)
            if((loss_new-loss_old)<10**(-16)):
                return [loss_new, X]
            iteration_times = iteration_times + 1
        return [loss_new, X]
    
    def X_modification(self, order, X):
        for i in range(len(X)):
            X[i] = max(X[i], self.elements[order[i]].x_min)
            X[i] = min(X[i], self.elements[order[i]].x_max)
        return X

    def d_obj_function(self, order, X, exit):
        df = [0 for i in range(len(X))]
        df[0] = ((X[0] - exit.x)+self.elements[order[0]].a*((self.elements[order[0]].a*X[0]+self.elements[order[0]].b)-exit.y))/(math.sqrt((X[0] - exit.x)**2+((self.elements[order[0]].a*X[0]+self.elements[order[0]].b)-exit.y)**2))
        for i in range(len(X)-1):
            df[i] = df[i] - ((X[i+1]-X[i]) + self.elements[order[i]].a*((self.elements[order[i+1]].a*X[i+1]+self.elements[order[i+1]].b)-(self.elements[order[i]].a*X[i]+self.elements[order[i]].b)))/(math.sqrt((X[i+1] - X[i])**2+((self.elements[order[i+1]].a*X[i+1]+self.elements[order[i+1]].b)-(self.elements[order[i]].a*X[i]+self.elements[order[i]].b))**2))
            df[i+1] = ((X[i+1]-X[i]) + self.elements[order[i+1]].a*((self.elements[order[i+1]].a*X[i+1]+self.elements[order[i+1]].b)-(self.elements[order[i]].a*X[i]+self.elements[order[i]].b)))/(math.sqrt((X[i+1] - X[i])**2+((self.elements[order[i+1]].a*X[i+1]+self.elements[order[i+1]].b)-(self.elements[order[i]].a*X[i]+self.elements[order[i]].b))**2))
        df[-1] = df[-1] + ((X[-1]-exit.x)+self.elements[order[-1]].a*((self.elements[order[-1]].a*X[-1]+self.elements[order[-1]].b)-exit.y))/(math.sqrt((X[-1] - exit.x)**2+((self.elements[order[-1]].a*X[-1]+self.elements[order[-1]].b)-exit.y)**2))
        return df

    def obj_function(self, order, X, exit):
        X = self.X_modification(order, X)
        obj = 0
        obj = obj + math.sqrt((X[0]-exit.x)**2 + ((self.elements[order[0]].a*X[0]+self.elements[order[0]].b) - exit.y)**2)
        for i in range(len(order)-1):
            obj = obj + math.sqrt((X[i+1]-X[i])**2 + ((self.elements[order[i]].a*X[i]+self.elements[order[i]].b) - (self.elements[order[i+1]].a*X[i+1]+self.elements[order[i+1]].b))**2)
        obj = obj + math.sqrt((exit.x-X[-1])**2 + ( exit.y - (self.elements[order[-1]].a*X[-1]+self.elements[order[-1]].b))**2)
        return obj

class DP_solver(object):
    def __init__(self, objs):
        self.obj_list = self.dict2list(objs)
        self.table = {} 
        self.exit = EXIT
        self.objs = []
        self.subgroup_dictionary = {}
        self.dy_group_dictionary = {}
        self.n = len(objs.items())
        self.paths = []
        self.paths_improved = []
        self.greedy_paths = []
        self.pointwise_paths = []
        self.cost = 0
        self.improved_cost = 0
        self.time_member_generation = 0
        self.time_subgroup_generation = 0
        self.time_distance_computation = 0
        self.time_vsg = 0
        self.time_vsg_improvement = 0
        self.time_gtsp = 0
        # self.time_gtsp_old = 0
        self.overlapping_dict = {}

        # output
        self.DP_pickup_points = []
        self.DP_robot_locations = []
        self.DP_obj_order = []
        self.greedy_pickup_points = []
        self.greedy_robot_locations = []
        self.greedy_obj_order = []
        index_accum = 0
        for sorted_obj in self.obj_list:
            # print "obj", obj
            obj = copy.deepcopy(sorted_obj)
            del obj[0]
            new_obj =Workplace_Object()
            index_accum += 1 
            new_obj.index = index_accum
            [new_obj.a, new_obj.b, new_obj.x_min, new_obj.x_max, new_obj.x_real_min, new_obj.x_real_max, length]= self.point2property(obj)
            if length > 0.19:
                new_obj.weight = 0.6
            else:
                new_obj.weight = 0.3
            # new_obj.weight = random_weight()
            print "index", new_obj.index
            print("weight", new_obj.weight)
            self.objs.append(new_obj)
        self.overlapping_checking()
    
    def dict2list(self,obj_dict):
        obj_list = []
        for key in obj_dict.keys():
            for i in range(len(key)):
                if key[i]=='_':
                    obj_index = int(key[(i+1):len(key)])
            print "index", obj_index
            obj_list.append([obj_index])
            obj_list[-1] = obj_list[-1] + obj_dict[key]
        obj_list = sorted(obj_list, key= lambda e : e[0])
        return obj_list

    def experiment(self):
        # self.dynamic_programming()
        # self.construct_paths()
        self.greedy_heuristics()
        
        # modify obj_order
        self.DP_obj_order = self.obj_index_motification(self.DP_obj_order)
        self.greedy_obj_order = self.obj_index_motification(self.greedy_obj_order)
        
        for obj in self.objs:
            plt.plot([obj.x_real_min, obj.x_real_max], [obj.a*obj.x_real_min+obj.b, obj.a*obj.x_real_max+obj.b], color='r')
        for path in self.paths:
            for line in path:
                Xs = []
                Ys = []
                for point in line:
                    Xs.append(point.x)
                    Ys.append(point.y)
                plt.plot(Xs, Ys, color='b')
        for line in self.greedy_paths:
            Xs = []
            Ys = []
            for point in line:
                Xs.append(point.x)
                Ys.append(point.y)
            plt.plot(Xs, Ys, color='orange')
        plt.show()
    
    def obj_index_motification(self, obj_order):
        new_order = []
        for tour in obj_order:
            new_order.append([])
            for obj in tour:
                new_order[-1].append(self.obj_list[obj-1][0])
        return new_order

    def overlapping_checking(self):
        for i in range(self.n):
            self.overlapping_dict[self.objs[i].index] = []
            '''
            for j in range(0,i):
                if self.line_intersection_check(self.objs[i], self.objs[j]):
                    # self.num_overlapping_obj += 1
                    self.overlapping_dict[self.objs[i].index].append(self.objs[j].index)
            '''
        print self.overlapping_dict

    def obj_line_intersection_check(self, obj1, obj2):
        r1 = (obj1.x_min, obj1.a*obj1.x_min+obj1.b)
        r2 = (obj1.x_max, obj1.a*obj1.x_max+obj1.b)

        l1 = (obj2.x_min, obj2.a*obj2.x_min+obj2.b)
        l2 = (obj2.x_max, obj2.a*obj2.x_max+obj2.b)
        
        return self.line_intersection_check(r1, r2, l1, l2)

    '''
    def line_intersection_check(self, r1, r2, l1, l2):        
        if ((l2[1]-l1[1])*(r2[0]-r1[0]))==((r2[1]-r1[1])*(l2[0]-l1[0])):
            return False
        alpha_l = (l1[1]*(l2[0]-r1[0]) - r1[1]*(l2[0] - l1[0]) + r2[1]*(r1[0] - l1[0]))/((r2[1]-r1[1])*(l2[0]-l1[0]) - (l2[1]-l1[1])*(r2[0]-r1[0]))
        print alpha_l
        if (alpha_l>=0) & (alpha_l<=1):
            alpha_r = (r1[1]*(r2[0]-l1[0]) - l1[1]*(r2[0] - r1[0]) + l2[1]*(l1[0] - r1[0]))/((l2[1]-l1[1])*(r2[0]-r1[0]) - (r2[1]-r1[1])*(l2[0]-l1[0]))
            print alpha_r
            if (alpha_r>=0) & (alpha_r<=1):
                return True
        return False
    '''
    
    def line_intersection_check(self, r1, r2, l1, l2):
        epsilon = 0.001        
        if ((l2[1]-l1[1])*(r2[0]-r1[0]))==((r2[1]-r1[1])*(l2[0]-l1[0])):
            return False
        # l1 + alpha_l...
        alpha_l = (l1[1]*(r2[0]-r1[0]) - r1[1]*(r2[0] - l1[0]) + r2[1]*(r1[0] - l1[0]))/((r2[1]-r1[1])*(l2[0]-l1[0]) - (l2[1]-l1[1])*(r2[0]-r1[0]))
        print alpha_l
        if (alpha_l>=0-epsilon) & (alpha_l<=1+epsilon):
            alpha_r = (r1[1]*(l2[0]-l1[0]) - l1[1]*(l2[0] - r1[0]) + l2[1]*(l1[0] - r1[0]))/((l2[1]-l1[1])*(r2[0]-r1[0]) - (r2[1]-r1[1])*(l2[0]-l1[0]))
            print alpha_r
            if (alpha_r>=0-epsilon) & (alpha_r<=1+epsilon):
                return True
        return False
    

    # def init_workplace(self):
        # print "properties", new_obj.a, new_obj.b, new_obj.x_min, new_obj.x_max
        
        
        

    def point2property(self, obj):
        length_1 = math.sqrt((obj[0][0] - obj[1][0])**2 + (obj[0][1] - obj[1][1])**2)
        length_2 = math.sqrt((obj[0][0] - obj[3][0])**2 + (obj[0][1] - obj[3][1])**2)
        # print "length", length_1, length_2
        p1 = vg.Point(obj[0][0], obj[0][1])
        if length_1 > length_2:
            p2 = vg.Point(obj[1][0], obj[1][1])
            p3 = vg.Point(obj[2][0], obj[2][1])
            p4 = vg.Point(obj[3][0], obj[3][1])
        else:
            p4 = vg.Point(obj[1][0], obj[1][1])
            p3 = vg.Point(obj[2][0], obj[2][1])
            p2 = vg.Point(obj[3][0], obj[3][1])
        x_real_min = min((p1.x+p4.x)/2.0, (p2.x+p3.x)/2.0)
        x_real_max = max((p1.x+p4.x)/2.0, (p2.x+p3.x)/2.0)
        a = (p2.y-p1.y)/((p2.x-p1.x))
        b = (p1.y+p4.y)/2.0 - (p1.x+p4.x)/2.0 * a
        x_min = x_real_min - (x_real_max - x_real_min)*robot_radius/max(length_1, length_2)
        x_max = x_real_max + (x_real_max - x_real_min)*robot_radius/max(length_1, length_2)
        return [a,b,x_min,x_max, x_real_min, x_real_max, max(length_1, length_2)]

    def greedy_heuristics(self):
        epsilon = 0.001
        paths = []
        

        objs_to_pick_up = {}
        for obj in self.objs:
            objs_to_pick_up[obj.index] = obj
        
        tour_pickup_points = []
        tour_robot_locations = []
        tour_object_order = []
        while len(objs_to_pick_up.values()) > 0:
            current_point = self.exit
            current_load = 0.0
            while current_load<1.0:
                is_full = 1
                sorted_list = self.greedy_sort_objs_by_distance(objs_to_pick_up, current_point, current_load)
                for obj in sorted_list:
                    if (self.objs[obj[2] - 1].weight <= (1-current_load))& (self.greedy_valid_pick(obj[2], objs_to_pick_up.keys())):
                        polys = []
                        for key in objs_to_pick_up.keys():
                            if key != obj[2]:
                                obstacle = objs_to_pick_up[key]
                            else:
                                continue
                            half_thickness = robot_radius
                            delta_x = half_thickness*math.cos(math.atan(-1.0/obstacle.a))
                            delta_y = half_thickness*math.sin(math.atan(-1.0/obstacle.a))
                            # polys.append([vg.Point(obstacle.x_min, obstacle.a * obstacle.x_min + obstacle.b), vg.Point(obstacle.x_max, obstacle.a * obstacle.x_max + obstacle.b), vg.Point(obstacle.x_max + 1, obstacle.a * obstacle.x_max + obstacle.b + 1), vg.Point(obstacle.x_min + 1, obstacle.a * obstacle.x_min + obstacle.b + 1)])
                            polys.append([vg.Point(obstacle.x_min+delta_x, obstacle.a * obstacle.x_min +delta_y + obstacle.b), vg.Point(obstacle.x_max+delta_x, obstacle.a * obstacle.x_max + delta_y + obstacle.b), vg.Point(obstacle.x_max-delta_x, obstacle.a * obstacle.x_max - delta_y + obstacle.b), vg.Point(obstacle.x_min - delta_x, obstacle.a * obstacle.x_min - delta_y + obstacle.b)])
                        g = vg.VisGraph()
                        g.build(polys)
                        paths.append(g.shortest_path(vg.Point(current_point.x+epsilon, current_point.y+epsilon), vg.Point( obj[1].x + epsilon, obj[1].y+epsilon)))
                        current_point = obj[1]
                        tour_pickup_points.append((obj[1].x, obj[1].y))
                        tour_object_order.append(obj[2])
                        tour_robot_locations.append(self.acquire_greedy_robot_location(paths[-1]))
                        current_load = current_load + self.objs[obj[2]-1].weight
                        objs_to_pick_up.pop(obj[2])
                        is_full = 0
                        break
                if is_full == 1:
                    self.greedy_obj_order.append(tour_object_order)
                    self.greedy_pickup_points.append(tour_pickup_points)
                    self.greedy_robot_locations.append(tour_robot_locations)
                    paths.append(g.shortest_path(vg.Point(current_point.x+epsilon, current_point.y+epsilon), vg.Point(self.exit.x+epsilon, self.exit.y+ epsilon)))
                    tour_pickup_points = []
                    tour_robot_locations = []
                    tour_object_order = []
                    break
        self.greedy_paths = paths
        length = self.greedy_path_length(paths)
        return length

    def acquire_greedy_robot_location(self, path):
        p1 = path[-1]
        p2 = path[-2]
        length = math.sqrt((p1.x-p2.x)**2+(p1.y-p2.y)**2)
        return (p1.x+(p2.x-p1.x)*(robot_radius/length), p1.y+(p2.y-p1.y)*(robot_radius/length))


    def greedy_path_length(self, paths):
        length = 0
        for path in paths:
            for i in range(len(path)-1):
                length = length + sqrt((path[i].x-path[i+1].x)**2+(path[i].y-path[i+1].y)**2)
        return length
        
    def greedy_sort_objs_by_distance(self, objs_to_pick_up, current_point, current_load):
        object_list = []
        for obj in objs_to_pick_up.values():
            [distance, point] = self.greedy_point2obj(current_point, obj)
            object_list.append([distance, point, obj.index])
        return sorted(object_list, key=lambda e: e[0])

    def greedy_point2obj(self, point, obj):
        min_end = vg.Point(obj.x_real_min, obj.a*obj.x_real_min+obj.b)
        max_end = vg.Point(obj.x_real_max, obj.a*obj.x_real_max+obj.b)
        cos_theta_min = ((point.x-obj.x_real_min)*(obj.x_real_max-obj.x_real_min)+(point.y-(obj.a*obj.x_real_min+obj.b))*((obj.a*obj.x_real_max+obj.b)-(obj.a*obj.x_real_min+obj.b)))/(sqrt((point.x-obj.x_real_min)**2+(point.y-(obj.a*obj.x_real_min+obj.b))**2)*sqrt((obj.x_real_max-obj.x_real_min)**2+((obj.a*obj.x_real_max+obj.b)-(obj.a*obj.x_real_min+obj.b))**2))
        cos_theta_max = ((point.x-obj.x_real_max)*(obj.x_real_min-obj.x_real_max)+(point.y-(obj.a*obj.x_real_max+obj.b))*((obj.a*obj.x_real_min+obj.b)-(obj.a*obj.x_real_max+obj.b)))/(sqrt((point.x-obj.x_real_max)**2+(point.y-(obj.a*obj.x_real_max+obj.b))**2)*sqrt((obj.x_real_min-obj.x_real_max)**2+((obj.a*obj.x_real_min+obj.b)-(obj.a*obj.x_real_max+obj.b))**2))
        if cos_theta_max*cos_theta_min<=0:
            if cos_theta_min<=0:
                return [sqrt((point.x-obj.x_real_min)**2+(point.y-(obj.a*obj.x_real_min+obj.b))**2), vg.Point(obj.x_real_min, obj.a*obj.x_real_min+obj.b)]
            else:
                return [sqrt((point.x-obj.x_real_max)**2+(point.y-(obj.a*obj.x_real_max+obj.b))**2), vg.Point(obj.x_real_max, obj.a*obj.x_real_max+obj.b)]
        else:
            distance_opt2min = sqrt((point.x-obj.x_real_min)**2+(point.y-(obj.a*obj.x_real_min+obj.b))**2)*cos_theta_min
            distance = distance_opt2min*tan(math.acos(cos_theta_min))
            return[distance, vg.Point(obj.x_real_min+1/sqrt(1+obj.a**2)*distance_opt2min, obj.a*obj.x_real_min+obj.b+obj.a/sqrt(1+obj.a**2)*distance_opt2min)]

    def greedy_valid_pick(self, obj_index, objs_list):
        intersection = set(self.overlapping_dict[obj_index]) & set(objs_list)
        if len(intersection) > 0.5:
            return False
        return True

    def dynamic_programming(self):
        self.dy_group_dictionary[0] = [0, Subgroup(), [], 0]
        for dy_num in range(1, self.n + 1):
            for set in self.set_iterator(dy_num):
                dy_group = Dynamic_Group(set, self.objs, self.dy_group_dictionary, self.exit,self.overlapping_dict)
                self.time_member_generation = self.time_member_generation + dy_group.time_member_generation
                self.time_subgroup_generation = self.time_subgroup_generation + dy_group.time_subgroup_generation
                self.time_distance_computation = self.time_distance_computation + dy_group.time_distance_computation
                self.time_vsg = self.time_vsg + dy_group.time_vsg
                self.time_vsg_improvement = self.time_vsg_improvement + dy_group.time_vsg_improvement
                self.time_gtsp = self.time_gtsp + dy_group.time_gtsp
                # self.time_gtsp_old = self.time_gtsp_old + dy_group.time_gtsp_old
                self.dy_group_dictionary[set] = [dy_group.cost, copy.deepcopy(dy_group.first_pick_subgroup), copy.deepcopy(dy_group.first_subgroup_path), dy_group.improved_cost, copy.deepcopy(dy_group.first_subgroup_path_improved), dy_group.obj_order]
        self.cost = self.dy_group_dictionary[2**(self.n+1)-2][0]
        self.improved_cost = self.dy_group_dictionary[2**(self.n+1)-2][3]


    def construct_paths(self):
        rest_index = 2**(self.n+1)-2
        while rest_index !=0:
            [new_path, new_order] = self.path_order_modification(self.dy_group_dictionary[rest_index][2], self.dy_group_dictionary[rest_index][5])
            self.paths.append(new_path)
            self.paths_improved.append(self.dy_group_dictionary[rest_index][4])
            self.DP_obj_order.append(new_order)
            rest_index = rest_index - self.dy_group_dictionary[rest_index][1].index
        self.pickup_points_robot_location_processing()

    def path_order_modification(self, raw_paths, order):
        obj_dict = {}
        for obj in self.objs:
            obj_dict[obj.index] = copy.deepcopy(obj)
        paths = []
        for raw_path in raw_paths:
            path = []
            for i in range(len(raw_path)-1):
                line = [raw_path[i], raw_path[i+1]]
                path.append(line)
            paths.append(path)
        for i in range(len(order)):
            path = paths[i]
            for line_i in range(len(path)):
                line = path[line_i]
                r1 = (line[0].x, line[0].y)
                r2 = (line[1].x, line[1].y)
                for j in range(i+1, len(order)):
                    l1 = (obj_dict[order[j]].x_min, obj_dict[order[j]].a * obj_dict[order[j]].x_min + obj_dict[order[j]].b)
                    l2 = (obj_dict[order[j]].x_max, obj_dict[order[j]].a * obj_dict[order[j]].x_max + obj_dict[order[j]].b)
                    if self.line_intersection_check(r1, r2, l1, l2):
                        l1 = (obj_dict[order[j]].x_real_min, obj_dict[order[j]].a * obj_dict[order[j]].x_real_min + obj_dict[order[j]].b)
                        l2 = (obj_dict[order[j]].x_real_max, obj_dict[order[j]].a * obj_dict[order[j]].x_real_max + obj_dict[order[j]].b)
                        alpha_l = (l1[1]*(r2[0]-r1[0]) - r1[1]*(r2[0] - l1[0]) + r2[1]*(r1[0] - l1[0]))/((r2[1]-r1[1])*(l2[0]-l1[0]) - (l2[1]-l1[1])*(r2[0]-r1[0]))
                        point = vg.Point(l1[0]+alpha_l*(l2[0]-l1[0]), l1[1]+alpha_l*(l2[1]-l1[1]))
                        path1 = copy.deepcopy(path[0:line_i]) + copy.deepcopy([[line[0],point]])
                        path2 = copy.deepcopy([[point,line[1]]]) + copy.deepcopy(path[line_i+1:])
                        new_path_j = copy.deepcopy(paths[j]) + copy.deepcopy(paths[j+1])
                        paths = copy.deepcopy(paths[0:i]) + copy.deepcopy([path1, path2]) + copy.deepcopy(paths[i+1:j]) + [copy.deepcopy(new_path_j)] + copy.deepcopy(paths[j+2:])
                        order = order[0:i] + [order[j]] + order[i:j] + order[j+1:]
                        break
        new_raw_paths = []
        # print paths
        for path in paths:
            print path[0]
            raw_path = [path[0][0]]
            for line in path:
                raw_path.append(line[1])
            new_raw_paths.append(raw_path)

        return [new_raw_paths, order]

    def pickup_points_robot_location_processing(self):
        for path in self.paths:
            tour_pickup_points = []
            tour_robot_locations = []
            for segment_index in range(len(path)-1):
                segment = path[segment_index]
                p1 = segment[-1]
                p2 = segment[-2]
                if p1 == p2:
                    tour_pickup_points.append((p1.x, p1.y))
                    tour_robot_locations.append(tour_robot_locations[-1])
                    continue
                length = math.sqrt((p1.x-p2.x)**2+(p1.y-p2.y)**2)
                tour_pickup_points.append((p1.x, p1.y))
                tour_robot_locations.append((p1.x+(p2.x-p1.x)*(robot_radius/length), p1.y+(p2.y-p1.y)*(robot_radius/length)))
            self.DP_pickup_points.append(tour_pickup_points)
            self.DP_robot_locations.append(tour_robot_locations)

    # input size, output the possible set indices. The complete set is self.objs
    def set_iterator(self, size):
        for members in tuple(itertools.combinations(range(1, self.n+1),size)):
            s = 0
            for obj in members:
                s += 1 << obj
            yield s

    def subgroup_iterator(self, set_index):
        for subgroup_index in self.ordered_subgroup_keys:
            if (subgroup_index>set_index):
                break
            if self.is_subgroup( set_index, subgroup_index) & 1:
                yield subgroup_index

    def is_subgroup(self, set, subgroup):
        for i in range(1,self.n+1):
            if(subgroup<(1 << i)):
                return True
            if( ((subgroup >> i)&(~(set >> i)))%2):
                return False
        return True

    def generate_subgroups( self, dy_group):
        subgroups = [] 
        for obj in dy_group.elements:
            subgroup = Subgroup() 
            subgroup.elements.append(obj)
            subgroup.weight = obj.weight
            subgroup.index = 2**(obj.index)
            self.subgroup_dictionary[subgroup.index] = copy.deepcopy(subgroup)
            subgroups.append(subgroup) 
        k_groups = copy.deepcopy( subgroups) 
        while len(k_groups)>0:
            k_groups = self.generate_larger_subgroups( dy_group, k_groups) 
            subgroups = subgroups + k_groups
        return subgroups

    def generate_larger_subgroups( self, dy_group,  k_groups):
        new_groups = []
        new_group_indexs = []
        for obj in dy_group.elements:
            for subgroup in k_groups:
                if (subgroup.index//(2**(obj.index)))%2 == 0: #obj not in subgroup
                    new_weight = subgroup.weight + obj.weight 
                    new_index = subgroup.index + 2**obj.index 
                    if new_weight<=1 & (new_index not in new_group_indexs): #capacity constraint and check repeat
                        new_group = Subgroup() 
                        new_group.weight = new_weight 
                        new_group.elements = subgroup.elements + [obj]
                        new_group.index = new_index
                        new_groups.append(new_group) 
                        new_group_indexs.append(new_index) 
                        self.subgroup_dictionary[new_group.index] = copy.deepcopy(new_group)
        return new_groups 

def random_weight():
    return 1/k + (2.0/k-1.0/k)*random.random()
    
def acquire_group_indexs(elements):
    index = 0
    for obj in elements:
        index = index + 2**obj.index
    return index

def make_dist_matrix(x, y):
    """Creates distance matrix for the given coordinate vectors"""
    N = len(x)
    xx = np.vstack( (x,)*N )
    yy = np.vstack( (y,)*N )
    return np.sqrt( (xx - xx.T)**2 + (yy - yy.T)**2 )

def generate_group_member(index):
    rest = index
    n = int(floor(log(index, 2)))
    for i in range(n):
        if ((rest>>(n-i)) >= 1):
            yield n-i
            rest -= 2**(n-i)
            if rest == 0:
                break
