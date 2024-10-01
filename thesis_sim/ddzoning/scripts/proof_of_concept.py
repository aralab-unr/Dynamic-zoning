import random
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import heapq
import math
import copy
from itertools import permutations, combinations
from zone.part import part
from queue import Queue


class concept(part):
        def __init__(self):

            map = np.array([
                #A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,a,b,c,d 
                [0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #A
                [1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #B
                [0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #C
                [0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #D
                [0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #E
                [0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #F
                #A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,a,b,c,d 
                [1,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #G
                [0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #H
                [0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #I
                [0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #J
                [0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0], #K
                [0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0], #L
                [0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0], #M
                [0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0], #N
                #A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,a,b,c,d 
                [0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0], #O
                [0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0], #P
                [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0], #Q
                [0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0], #R
                [0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0], #S
                [0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0], #T
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0], #U
                #A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,a,b,c,d 
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0], #V
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0], #W
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0], #X
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0], #Y
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0], #Z
                #A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,a,b,c,d 
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0], #a
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0], #b
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1], #c
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0], #d
            ])

            ptloc = {"A":[0,200],
                    "B":[50,200],
                    "C":[100,200],
                    "D":[150,200],
                    "E":[200,200],
                    "F":[250,200],
                    "G":[0,150],
                    "H":[50,150],
                    "I":[100,150],
                    "J":[150,150],
                    "K":[200,150],
                    "L":[250,150],
                    "M":[0,100],
                    "N":[50,100],
                    "O":[100,100],
                    "P":[150,100],
                    "Q":[200,100],
                    "R":[250,100],
                    "S":[0,50],
                    "T":[50,50],
                    "U":[100,50],
                    "V":[150,50],
                    "W":[200,50],
                    "X":[250,50],
                    "Y":[0,0],
                    "Z":[50,0],
                    "a":[100,0],
                    "b":[150,0],
                    "c":[200,0],
                    "d":[250,0]
                    }

            variables_set = pd.read_csv(os.path.join(os.getcwd(), 'data/LE', 'Variables_of_Zone_Warehouse_Phase1.csv'), sep=',', header=0, names=['value'], encoding = 'utf-8')
            self.critical_points = pd.read_csv(os.path.join(os.getcwd(), 'data/LE','Critical_Points.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
            self.workstation_loc = pd.read_csv(os.path.join(os.getcwd(), 'data/LE','Workstation_Loaction.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
            self.adjacency_Mtx_pd = pd.read_csv(os.path.join(os.getcwd(), 'data/LE','Adjacency_matrix.csv'), sep=',', header=0, names=list(self.critical_points.index.values), encoding = 'utf-8')
            self.workstation_points = pd.read_csv(os.path.join(os.getcwd(), 'data/LE','Workstation_points.csv'), sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
            self.workstation_loads = pd.read_csv(os.path.join(os.getcwd(), 'data/LE','Workstation_Loads.csv'), sep=',', header=0, names=list(self.workstation_loc.index.values), encoding = 'utf-8')
            values = variables_set['value']

            self.Wd = values.get('Wd')
            self.Wf = values.get('Wf')
            self.V = values.get('V')
            self.tl = values.get('tl')
            self.tu = values.get('tu')
            self.T = values.get('T')
            self.lt = values.get('lt')

            #workstation queues
            self.ws_qs = []
            
            #number of workstations
            self.numws = len(self.workstation_points['workstation'])

            self.adj_dist = 250

            self.dickcopy = {}

        def set_up(self):
            #create map
            self.zonews = [['WS9', 'WS14', 'WS6', 'WS10'], ['WS5', 'WS11', 'WS16', 'WS12', 'WS13', 'WS8', 'WS17', 'WS18', 'WS15'], ['WS2', 'WS4', 'WS1', 'WS3', 'WS7']]
            self.zonecs = [[['q', 'r', 'Q', 'X', 'W', 'v'], ['q', 'P', 'N', 'n'], ['r', 'q']], [['s', 'Y', 'Z', 'x'], ['x', 'Z', 't'], ['t', 'U', 'V', 'u'], ['m', 'L', 'p'], ['m', 'L', 'p', 'R', 'S', 'T', 's'], ['u', 'a', 'y'], ['y', 'a', 'b', 'z'], ['z', 'b', 'c', 'w']], [['j', 'B', 'C', 'D', 'l'], ['l', 'D', 'C', 'B', 'i'], ['i', 'A', 'E', 'k'], ['k', 'F', 'G', 'J', 'O', 'o']]]
            #               (0,1)        (0,2)    (1,2)
            self.ts = [['WS14', 'WS6'], ['WS6'], ['WS7']]
            #self.all_zone = [['WS7', 'WS15', 'WS18', 'WS14', 'WS10','WS9','WS17', 'WS16', 'WS8','WS6', 'WS4', 'WS2', 'WS3'], ['WS9', 'WS11', 'WS16', 'WS12', 'WS13', 'WS17', 'WS8','WS6', 'WS3'], ['WS2', 'WS1', 'WS4', 'WS3', 'WS5', 'WS6']]
            self.all_zone = [['WS9', 'WS14', 'WS6', 'WS10'], ['WS5', 'WS11', 'WS16', 'WS12', 'WS13', 'WS8', 'WS17', 'WS18', 'WS15', 'WS14', 'WS6','WS7'], ['WS2', 'WS4', 'WS1', 'WS3', 'WS7','WS6']]
            self.num_zone = len(self.zonews)
            
            #self.zonecs_all = [[['q', 'r', 'Q', 'X', 'W', 'v'], ['q', 'P', 'N', 'n'], ['r', 'q']], [['s', 'Y', 'Z', 'x'], ['x', 'Z', 't'], ['t', 'U', 'V', 'u'], ['m', 'L', 'p'], ['m', 'L', 'p', 'R', 'S', 'T', 's'], ['u', 'a', 'y'], ['y', 'a', 'b', 'z'], ['z', 'b', 'c', 'w'], ['v', 'b', 'z'], ['n', 'N', 'M', 'L', 'm'], ['m', 'H', 'I', 'E', 'k', 'F', 'G', 'J', 'O', 'o']], [['j', 'B', 'C', 'D', 'l'], ['l', 'D', 'C', 'B', 'i'], ['i', 'A', 'E', 'k'], ['k', 'F', 'G', 'J', 'O', 'o'], ['n', 'o']]]

            #robot queues
            self.robotqs =[{} for _ in range(self.num_zone)]

            #robot positions    WS15      WS9      WS3
            self.robotpos = ['15','9','3']
            self.robotxy = np.array([(230,70),(130,165),(130,315)])

            #create initial map with distance
            self.create_map()

            #create all adj mtx for each zone
            self.all_adj_matrix = [copy.deepcopy(self.adjacency_Mtx) for _ in range(self.num_zone)]
            self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zonecs, self.adjacency_Mtx)
            print("size:",len(self.all_adj_matrix))

            #create ws distance mtx
            self.workstation_dist_mtx = np.zeros((self.numws,self.numws))
            for x in range(self.numws):
                  nodex = self.workstation_points.at[x,'critical_points']
                  for y in range(self.numws):
                        if(x != y):
                            nodey = self.workstation_points.at[y,'critical_points']
                            dist_path, dist = self.shortest_dist(nodex,nodey,self.adjacency_Mtx)
                            self.workstation_dist_mtx[x,y] = dist
                        else:
                            self.workstation_dist_mtx[x,y] = 0

            self.ws_loads = [np.zeros((self.numws,self.numws)) for _ in range(self.num_zone)] #used to record load for each zone

            self.gen_intialws_q() #create parts and assign them to ws queues
            
            #load robots
            for wsq in self.ws_qs:
                for _ in range(wsq.qsize()):
                    part = wsq.get()
                    self.queue_part(part,self.zonews,self.ts)
                    #wsq.put(part)

            #find WS neighbors within a dist
            self.ws_neighbors = self.WS_neighbors(self.adj_dist)

        def start_sim(self):
            #to record the load of each robot
            self.r_load = []
            robot_err = [[] for x in range(self.num_zone)]
            robot_it = [[] for x in range(self.num_zone)]
            
            print("\nsize of 1q:",len(self.robotqs[0]))
            print("size of 2q:",len(self.robotqs[1]))
            print("size of 3q:",len(self.robotqs[2]))

            #reload parts for every robot
            for robot in range(self.num_zone):
                self.load_parts(robot, self.zonews, self.ts)

            #get the load of each robot
            for robot in range(self.num_zone):
                self.record_load(robot)

                #get the load of each robot
                self.r_load.append(self.zone_load(self.all_zone[robot],robot))
            Sum = 0
            for i in range(self.num_zone):
                Sum += len(self.robotqs[i])
            print("\nsum of parts", Sum)
            print("size of 1q:",len(self.robotqs[0]))
            print("size of 2q:",len(self.robotqs[1]))
            print("size of 3q:",len(self.robotqs[2]))
            
            self.best_r_load = copy.deepcopy(self.r_load)

            #find the average load between each robot using weight average consesus
            cal_load = self.avg_consenus(np.array(self.r_load))

            best_zonews = copy.deepcopy(self.zonews)
            best_zonecs = copy.deepcopy(self.zonecs)
            best_allzone = copy.deepcopy(self.all_zone)
            best_adjmtx = copy.deepcopy(self.all_adj_matrix)
            best_ts = copy.deepcopy(self.ts)

            all_error = []

            #expiremental algorithm
            max_it = 20
            for ep in range(1,51):
                #figure out which robot is neighbors with either robot
                r_neigh = self.findneighbors(self.num_zone, self.robotxy, 200)

                print("Epoc:", ep)
                
                tot_err = 0
                #record total error between all zones
                for r in range(self.num_zone):
                    tot_err += abs(self.best_r_load[r] - cal_load[r])
                all_error.append(tot_err/self.num_zone)

                for roboti in range(self.num_zone):

                    best_error = self.calc_error(roboti, r_neigh, self.best_r_load, cal_load)

                    for it in range(1,max_it):
                        
                        p = math.log1p(100/it)*.3
                        
                        #select a random neighbor from robot
                        robotj = random.choice(r_neigh[roboti])
                        
                        #get tip ws from both zones
                        i_tip = self.finding_tip_ws(self.zonews[roboti],self.zonecs[roboti])
                        j_tip = self.finding_tip_ws(self.zonews[robotj],self.zonecs[robotj])

                        #calculate the current load with the current zone design
                        self.load_parts(roboti, self.zonews, self.ts)
                        self.load_parts(robotj, self.zonews, self.ts)
                        #Sum = 0
                        #for i in range(self.num_zone):
                            #Sum += len(self.robotqs[i])
                        #print("\nsum of parts", Sum)
                        #print("size of 1q:",len(self.robotqs[0]))
                        #print("size of 2q:",len(self.robotqs[1]))
                        #print("size of 3q:",len(self.robotqs[2]))

                        self.record_load(roboti)
                        self.record_load(robotj)
                        
                        self.r_load[roboti] = self.zone_load(self.all_zone[roboti], roboti)
                        self.r_load[robotj] = self.zone_load(self.all_zone[robotj], robotj)

                        #get neighbor load and compare 
                        i_load = self.r_load[roboti]
                        j_load = self.r_load[robotj]

                        #**********figure out if i is giving or taking and exchange**********
                        if i_load >= j_load:
                            #roboti i is giving

                            #select a random WS to give to robot j that is adjacent
                            At = self.find_adjtip(i_tip,j_tip) #find tip ws in i and j that are adjacent

                            #exchange random tip ws to j robot
                            zonews_c, zonecs_c, alladj_c = self.exchange_ws(At, robotj)

                        elif i_load < j_load:
                            #robot i taking

                            #select a random WS to give to robot i that is adjacent
                            At = self.find_adjtip(j_tip,i_tip) #find tip ws in i and j that are adjacent
                            #exchange random tip ws to i robot
                            zonews_c, zonecs_c, alladj_c = self.exchange_ws(At, roboti)

                        #calculate new loads for roboti and robotj
                        #find new transfer stations
                        ts_c, all_zonecs_c = self.find_transfer_stations(zonews_c, zonecs_c, alladj_c, roboti, robotj)
                        #(0,1),(0,2),(1,2)
                        newts = copy.deepcopy(self.ts)
                        if roboti == 0 and robotj == 1: 
                            newts[0] = copy.deepcopy(ts_c)
                        elif roboti == 0 and robotj == 2: 
                            newts[1] = copy.deepcopy(ts_c)
                        elif roboti == 1 and robotj == 2: 
                            newts[2] = copy.deepcopy(ts_c)

                        #requeue each part and figure out the load of the new zonesij
                        self.load_parts(roboti, zonews_c, newts)
                        self.load_parts(robotj, zonews_c, newts)
                        test1 = self.check_repeats(roboti,robotj)
                        if (test1):
                            print("no repeats")
                        else:
                            print("repeats")

                        Sum = 0
                        for i in range(self.num_zone):
                            Sum += len(self.robotqs[i])
                        print("\nsum of parts", Sum)
                        print("size of 1q:",len(self.robotqs[0]))
                        print("size of 2q:",len(self.robotqs[1]))
                        print("size of 3q:",len(self.robotqs[2]))
                        self.record_load(roboti)
                        self.record_load(robotj)

                        #figure out if new zone design is closer to the average
                        #add transfer ws to all_zone
                        all_zonews_c = copy.deepcopy(self.all_zone)
                        all_zonews_c[roboti] = copy.deepcopy(zonews_c[roboti])
                        all_zonews_c[robotj] = copy.deepcopy(zonews_c[robotj])
                        for ws in ts_c:
                            if ws not in zonews_c[roboti]:
                                all_zonews_c[roboti].append(ws)
                            if ws not in zonews_c[robotj]:
                                all_zonews_c[robotj].append(ws)

                        #new loads
                        i_load = self.zone_load(all_zonews_c[roboti],roboti)
                        j_load = self.zone_load(all_zonews_c[robotj],robotj)
                        load_c = copy.deepcopy(self.r_load) #copy the structure
                        load_c[roboti] = i_load #replace load with i and j load
                        load_c[robotj] = j_load
                        
                        #calculate error
                        old_error = self.calc_error(roboti, r_neigh, self.r_load, cal_load)
                        new_error = self.calc_error(roboti, r_neigh, load_c, cal_load)

                        #accept if error is better than old error
                        r = random.random()
                        if new_error <= old_error or r < p:
                            self.zonews = copy.deepcopy(zonews_c)
                            self.zonecs = copy.deepcopy(zonecs_c)
                            self.ts = copy.deepcopy(newts)
                            self.all_zone = copy.deepcopy(all_zonews_c)
                            self.r_load[roboti] = copy.deepcopy(i_load)
                            self.r_load[robotj] = copy.deepcopy(j_load)
                            self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zonecs, self.adjacency_Mtx)
                            #self.valid_zone(self.zonews)
                        else:
                            self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zonecs, self.adjacency_Mtx)
                            #return robot qs to thier orginal 
                            self.load_parts(roboti, self.zonews, self.ts)
                            self.load_parts(robotj, self.zonews, self.ts)
                            self.record_load(roboti)
                            self.record_load(robotj)
                            #self.valid_zone(self.zonews)

                        if new_error < best_error: #save zone if best
                            best_error = copy.deepcopy(new_error)
                            best_zonews = copy.deepcopy(self.zonews)
                            best_zonecs = copy.deepcopy(self.zonecs)
                            best_allzone = copy.deepcopy(self.all_zone)
                            best_adjmtx = copy.deepcopy(self.all_adj_matrix)
                            best_ts = copy.deepcopy(self.ts)
                            self.best_r_load[roboti] = copy.deepcopy(self.r_load[roboti])
                            self.best_r_load[robotj] = copy.deepcopy(self.r_load[robotj])
                            robot_err[roboti].append(copy.deepcopy(best_error))
                            robot_it[roboti].append((ep-1)*max_it + it)

                    self.zonews = copy.deepcopy(best_zonews)
                    self.zonecs = copy.deepcopy(best_zonecs)
                    self.allzone = copy.deepcopy(best_allzone)
                    self.all_adj_matrix = copy.deepcopy(best_adjmtx)
                    self.ts = copy.deepcopy(best_ts)     
                    self.r_load = copy.deepcopy(self.best_r_load)
                    #reload parts that are connected to roboti
                    self.load_parts(roboti, self.zonews, self.ts)
                    for neigh in r_neigh[roboti]:
                        self.load_parts(neigh, self.zonews, self.ts)
                    #record the load with roboti and each neighbor
                    for neigh in r_neigh[roboti]:
                        self.record_load(neigh)         
                    self.record_load(roboti) 

            print("new zones!")
            print(self.zonews)
            print("(0,1), (0,2), (1,2)")
            print(self.ts)
            print(self.zonecs)
            #reload parts for every robot
            for robot in range(self.num_zone):
                self.load_parts(robot, self.zonews, self.ts)
            print("average",sum(self.r_load)/3)
            print("current loads",self.r_load)
            print("sum:", len(self.robotqs[0])+len(self.robotqs[1])+len(self.robotqs[2]))
            print("size of 1q:",len(self.robotqs[0]))
            print("size of 2q:",len(self.robotqs[1]))
            print("size of 3q:",len(self.robotqs[2]))
            '''
            fig, axis = plt.subplots(2,3)
            axis[0,0].plot(robot_it[0],robot_err[0])
            axis[0,0].set_title("robot1")
            axis[0,1].plot(robot_it[1],robot_err[1])
            axis[0,1].set_title("robot2")
            axis[0,2].plot(robot_it[2],robot_err[2])
            axis[0,2].set_title("robot3")
            axis[1,:].plot(all_error)
            axis[1,:].set_title("all error")
            '''
            plot1 = plt.subplot2grid((3,3),(0,0))
            plot2 = plt.subplot2grid((3,3),(0,1))
            plot3 = plt.subplot2grid((3,3),(0,2))
            plot4 = plt.subplot2grid((3,3),(1,0),colspan=4)\
            
            plot1.plot(robot_it[0],robot_err[0])
            plot1.set_title("robot1")
            plot2.plot(robot_it[1],robot_err[1])
            plot2.set_title("robot2")
            plot3.plot(robot_it[2],robot_err[2])
            plot3.set_title("robot3")
            plot4.plot(all_error)
            plot4.set_title("all error")

            plt.tight_layout()
            plt.show()

                    
        #functions for zone support(phase1 and 2)
        def create_map(self):
            self.adjacency_Mtx = self.adjacency_Mtx_pd.to_numpy()

            #convert row names to a list
            row_names = self.critical_points.index.to_list()

            numrows , numcoloumns = np.shape(self.adjacency_Mtx)

            for i in range(0,numrows):
                    for j in range(0,numcoloumns):
                        if self.adjacency_Mtx[i,j] == 0:
                                continue
                        else:
                                point1 = row_names[i]
                                point2 = row_names[j]
                                distance = self.get_distance(point1,point2)
                                self.adjacency_Mtx[i,j] = distance

        def shortest_dist(self, i, j, adj_matrix):
            #all_points is the set of all points in grid
            all_points = self.critical_points.index.to_list()
            unvisited_nodes=all_points

            #define priorty queue
            points_left = []

            #dictionarys for nodes and prev
            dist = dict()
            prev = dict()

            #dijkstra
            for node in all_points:
                dist[node] = 5000
                prev[node] = None
                if node != i:
                        heapq.heappush(points_left, (5000,node))
            dist[i] = 0
            heapq.heappush(points_left, (dist[i],i))
            
            while len(points_left) != 0:
                visted_dist, visited_point = heapq.heappop(points_left) #U
                visited_point_neighbors = self.get_neighbors(visited_point,adj_matrix) #neighbors of U
                unvisited_nodes.remove(visited_point)#mark current point as visited
                #prev.append(visited_point)#record the selected nodes

                if(visited_point == j):
                        break

                for node in visited_point_neighbors:
                        if node in unvisited_nodes:
                            #node = one unvisited neighbor of U
                            TDist = dist[visited_point] + self.get_distance(visited_point,node)
                            if TDist < dist[node]:
                                    prev_dist = dist[node]
                                    dist[node] = TDist
                                    prev[node] = visited_point
                                    node_index = points_left.index((prev_dist,node))
                                    points_left[node_index] = (dist[node],node)
                                    heapq.heapify(points_left)
        
            #if there is no path return none, 0
            if (prev[j] == None):
                return None,0

            #get the shorest path from prev
            current_node = j
            path = [j]
            while (current_node!=i):
                current_node = prev[current_node]
                path.append(current_node)

            path.reverse()
            return path,dist[j]
        
        def get_neighbors(self,i,adj_matrix):
            neighbor = []
            column_index = 0
            #get row corrisponding to i in adjacency matrix
            #i is a character and rows are numbered
            row_index = self.critical_points.index.get_loc(i)
            for j in adj_matrix[row_index]:
                if j != 0:
                        neighbor.append(self.critical_points.index[column_index])
                column_index+=1
            return neighbor
        
        def WS_neighbors(self,adj_dist):
            #adj_dist is the adjacency distance that a WS can have
            #neighbor_mtx = np.zeros([self.num_ws,self.num_ws])
            neighbors = []
            for ws in range(self.numws):
                ws_neigh = []
                for neigh in range(self.numws):
                    dist = self.workstation_dist_mtx[ws,neigh]
                    if(dist < adj_dist and dist != 0):
                        ws_str = 'WS' + str(neigh+1)
                        ws_neigh.append(ws_str)
                neighbors.append(ws_neigh)
            
            return neighbors
        
        def get_distance(self,i,j):
            #i and j are characters
            #get row as a dataframe and then convert to numpy
            point1 = self.critical_points.loc[i]
            point1 = point1.to_numpy()
            point2 = self.critical_points.loc[j]
            point2 = point2.to_numpy()
            #distance = math.dist(point1,point2)
            return math.dist(point1,point2)

        def zone_load(self, ws_in_zone, robot):
            #equations 11-14
            #finding Lpz
            #to find Lpz you need to find fpzij with is the load specifically in the zone
            #print("finding Lpz")
            self.fpzijD = {}

            all_fpzij = self.finding_zone_all_fpzij(ws_in_zone, robot)

            sum_DA_DB = 0

            #finding get gpzij
            for i in ws_in_zone:
                for j in ws_in_zone:
                    if (i != j):
                        gpzij = self.finding_gpzij(i, j, ws_in_zone, all_fpzij)
                        
                        index_i = self.ws_to_index(i)
                        index_j = self.ws_to_index(j)
                        dpzij = self.workstation_dist_mtx[index_i,index_j]
                        DApzij = gpzij * dpzij

                        fpzij = self.fpzijD[i + ',' + j]
                        DBpzij = fpzij*dpzij

                        sum_DA_DB += DApzij + DBpzij
            
            Lpz = (sum_DA_DB/self.V) + (all_fpzij*(self.tu + self.tl))
            self.fpzijD.clear()

            #if Lpz < 1:
                #print("rut ro")

            return Lpz
        
        def finding_gpzij(self,i,j,ws_in_zone, all_fpzij):
            #finding fpzki and fpzjk
            fpzki = 0
            fpzjk = 0
            for k in ws_in_zone:
                if(k!=i):
                    fpzki += self.fpzijD[k + ',' + i]
                if(k!=j):
                    fpzjk += self.fpzijD[j + ',' + k]
            return (fpzki*fpzjk)/all_fpzij
        
        def finding_fpzij(self,i,j,robot):
            #If i is a tip workstation, then fpzij includes
            #not only fij (which is the flow originating from i and going to j),
            # - but also the flow that originates from workstations in other zones,
            #   passes i, and arrives at j
            # - and the flow that originates from workstations in other zones, passes i, passes j, and arrives at workstations
            #   in other zones.
            # i and j are in the form 'ws1'

            i_index = self.ws_to_index(i)
            j_index = self.ws_to_index(j)

            fpzij = self.ws_loads[robot][i_index][j_index]

            return fpzij

        def finding_zone_all_fpzij(self,ws_in_zone,robot):
            all_fpzij = 1
            for i in ws_in_zone:
                for j in ws_in_zone:
            #for i in len(self.ws_loads[robot]): # i is index
                #for j in len(self.ws_loads[robot][i]): # j is index
                    if(i != j):
                        fpzij = self.finding_fpzij(i, j, robot)
                        all_fpzij += fpzij
                        self.fpzijD[i + ',' + j] = fpzij
            return all_fpzij

        def finding_tip_ws(self, zone_workstations, zone_crit_segments):
            #crit_segments is a 2D list
            #zone_workstaions is a list
            #all points is a dataframe
            workstations = self.workstation_points.loc[:,'workstation'].to_list()
            crit_workstations = self.workstation_points.loc[:,'critical_points'].to_list()

            tip_ws = []
            adj_crit_points = []

            for node in zone_workstations:
                adj_crit_points.clear()
                
                #node is "WS1" .. 
                ws_crit_point = crit_workstations[workstations.index(node)]
                tipcheck = 0
                #find adj crit points, a tip workstation is one that only has one branch connecting to it
                
                row = self.adjacency_Mtx[self.crit_point_index(ws_crit_point)]
                for x in range(len(row)):
                    if row[x]!=0:
                        adj_crit_points.append(self.index_crit_point(x))

                for point in adj_crit_points:
                    for segment in zone_crit_segments:
                        if point in segment and ws_crit_point in segment:
                            tipcheck += 1
                            break
                """
                for segment in zone_crit_segments:
                    if ws_crit_point in segment:
                        tipcheck +=1
                """

                if(tipcheck == 1):
                    tip_ws.append(node)

            #special case for when there is only one WS in zone
            if len(zone_workstations) == 1:
                tip_ws.append(zone_workstations[0])

            return tip_ws

        def remove_tip_ws(self, zonews, zonecs, all_adjMatrix, tipws):
            #best ws:
            #[['WS8', 'WS9', 'WS4', 'WS3'], ['WS5', 'WS2', 'WS1', 'WS7'], ['WS11', 'WS6', 'WS10']]
            #best cs:
            #[[['d', 'V', 'W', 'c'], ['c', 'W', 'N', 'M', 'K'], ['P', 'O', 'J', 'K']], [['S', 'R', 'Q', 'H', 'I'], ['E', 'D', 'H', 'I'], ['U', 'T', 'S']], [['X', 'Y', 'Z', 'a'], ['a', 'Z', 'Y', 'b']]]
            #find zone that zone cs is in
            for z in range(self.num_zone):
                if tipws in zonews[z]:
                    zonei = z
                    break
            #find the crit pint that tip ws is at
            crit_tip = self.ws_crit_point(tipws)

            #find workstations that are connected to tip ws
            connectedws = []
            for segment in zonecs[zonei]:
                if crit_tip in segment:
                    criti = segment.index(crit_tip)
                    if criti == 0:
                        connectedws.append(self.crit_point_ws(segment[len(segment)-1]))
                    else:
                        connectedws.append(self.crit_point_ws(segment[0]))
            
            #special case for when tipws is only connected to one ws
            #remove tipws and cs and update adj matrix
            if len(connectedws) == 1:
                #print("single tip ws")
                for segment in zonecs[zonei]:
                    if crit_tip in segment:
                        zonecs[zonei].remove(segment)
                        zonews[zonei].remove(tipws)

                        #remove path from adj matrix in alpha
                        all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, zonecs, self.adjacency_Mtx)

            #case for when there are multiple ws connected to tipws
            if len(connectedws) > 1:
                #print("multiple tip ws")
                zonews[zonei].remove(tipws)
                new_zonecs = copy.deepcopy(zonecs)
                #remove all segemnts that are connected to tipws
                for segment in zonecs[zonei]:
                    if (crit_tip in segment):
                        new_zonecs[zonei].remove(segment)

                #update adj_matrix
                all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, new_zonecs, self.adjacency_Mtx)

                #rebuld zone
                skipws = []
                for wsi in range(len(connectedws)):
                    if wsi == 0:
                        #save the location of the inserted crit segemnts in zonecs
                        inserti = len(new_zonecs[zonei])
                    
                    ws1 = connectedws[wsi]
                    if ws1 not in skipws:
                        #find closest WS in connectedws
                        bestdist = 1000
                        index1 = self.ws_to_index(ws1)
                        for ws in connectedws:
                            if ws != ws1 and ws != tipws:
                                index2 = self.ws_to_index(ws)
                                if self.workstation_dist_mtx[index1][index2] < bestdist:
                                    bestdist = self.workstation_dist_mtx[index1][index2]
                                    ws2 = ws

                        #find the path connecting two prev connected WS
                        critws1 = self.ws_crit_point(ws1)
                        critws2 = self.ws_crit_point(ws2)
                        path,dist = self.shortest_dist(critws1, critws2, all_adjMatrix[zonei])
                        #if there is no path connecting the two prev WS then select a random one from connected WS
                        if(path is None):
                            while (True):
                                ws2 = random.choice(connectedws)
                                if ws2 != ws1:
                                    critws1 = self.ws_crit_point(ws1)
                                    critws2 = self.ws_crit_point(ws2)
                                    path,dist = self.shortest_dist(critws1, critws2, all_adjMatrix[zonei])
                                if path is not None:
                                    break
                        new_zonecs[zonei].append(path)
                        all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, new_zonecs, self.adjacency_Mtx)
                        skipws.append(ws2)
                
                #special case for when segments are disconnected creating two different zones in the same zone
                if(len(new_zonecs[zonei]) != len(zonews[zonei])-1):
                    pool = []
                    group = []
                    #find groups
                    for segment in new_zonecs[zonei]:
                        first = segment[0]
                        last = segment[len(segment)-1]
                        pool.append([first,last])

                    skippair = []
                    groupi = -1
                    for pair in pool:
                        if pair in skippair:
                            continue
                        group.append([])
                        groupi += 1
                        first = pair[0]
                        last = pair[1]
                        group[groupi].append(pair[0])
                        group[groupi].append(pair[1])
                        follow = first
                        skippair.append(pair)
                        added = True
                        while(added):
                            added = False
                            for pair in pool:
                                if pair not in skippair:
                                    if follow in pair:
                                        group[groupi].append(pair[0])
                                        group[groupi].append(pair[1])
                                        if pair.index(follow) == 0:
                                            follow = pair[1]
                                        else:
                                            follow = pair[0]
                                        skippair.append(pair)
                                        added = True
                        follow = last
                        added = True
                        while(added):
                            added = False
                            for pair in pool:
                                if pair not in skippair:
                                    if follow in pair:
                                        group[groupi].append(pair[0])
                                        group[groupi].append(pair[1])
                                        if pair.index(follow) == 0:
                                            follow = pair[1]
                                        else:
                                            follow = pair[0]
                                        skippair.append(pair)
                                        added = True
                    
                    #temporary
                    for gi1 in range(len(group)):
                        if len(group)-1 < gi1: 
                            break
                        g1 = group[gi1]
                        for gi2 in range(len(group)):
                            if len(group)-1 < gi2: 
                                break
                            g2 = group[gi2]
                            if (gi1 != gi2):
                                for critws in g1:
                                    if critws in g2:
                                        if len(g1)>len(g2):
                                            for x in g2:
                                                group[gi1].append(x)
                                            del group[gi2]
                                        else:
                                            for x in g1:
                                                group[gi2].append(x)
                                            del group[gi1]
                                        break
                    #connect to closest ws not in group
                    skipg = []
                    for gi in range(len(group)):
                        if gi in skipg:
                            continue
                        bestdist = 1000
                        bestcrit1 = ''
                        bestcrit2 = ''
                        for critws1 in group[gi]:
                            #find closest WS connectedws
                            ws1 = self.crit_point_ws(critws1)
                            index1 = self.ws_to_index(ws1)
                            for gi2 in range(len(group)):
                                if gi2 == gi:
                                    continue
                                for critws2 in group[gi2]:
                                    ws2 = self.crit_point_ws(critws2)
                                    index2 = self.ws_to_index(ws2)
                                    if (self.workstation_dist_mtx[index1][index2] < bestdist) and (critws1 != critws2):
                                        bestdist = self.workstation_dist_mtx[index1][index2]
                                        bestcrit1 = critws1
                                        bestcrit2 = critws2
                                        skipgi = gi2
                        
                        skipg.append(skipgi)
                        all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, new_zonecs, self.adjacency_Mtx)
                        path,dist = self.shortest_dist(bestcrit1,bestcrit2,all_adjMatrix[zonei])
                        if(path is None):
                            print("none hit")
                            print(zonews)
                            print(new_zonecs)
                            print(bestcrit1)
                            print(bestcrit2)
                            print(tipws)
                        new_zonecs[zonei].insert(inserti,path)
                        all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, new_zonecs, self.adjacency_Mtx)

                #restructure the zone
                new_zonews = []
                for segment in new_zonecs[zonei]:
                    ws1 = self.crit_point_ws(segment[0])
                    ws2 = self.crit_point_ws(segment[len(segment)-1])
                    if ws1 not in new_zonews:
                        new_zonews.append(ws1)
                    if ws2 not in new_zonews:
                        new_zonews.append(ws2)

                zonews[zonei] = copy.deepcopy(new_zonews)
                zonecs = copy.deepcopy(new_zonecs)

            return copy.deepcopy(zonews), copy.deepcopy(zonecs)

        def update_adj_matrixs(self, all_adj_matrixs, crit_segments, map):

            for x in range(len(all_adj_matrixs)):
                #make adj matrix the same as map to start
                all_adj_matrixs[x] = copy.deepcopy(map)
                
                #now remove every point in every other crit segments
                for i in range(len(crit_segments)):
                    if i != x:
                        for path in crit_segments[i]:
                                all_adj_matrixs[x] = self.adj_remove_seg(path, all_adj_matrixs[x])

            return all_adj_matrixs
        
        def adj_remove_seg(self, path, adj_matrixs):

            all_points = self.critical_points.index.to_list()
            for pt in path:
                    index_of_pt = all_points.index(pt)
                    #have to remove the point from being used 
                    adj_matrixs[index_of_pt, :] = 0
                    adj_matrixs[ :, index_of_pt] = 0

            return adj_matrixs

        def ws_to_index(self,i):
            ws_list = self.workstation_points.loc[:,'workstation'].to_list()
            i_index = ws_list.index(i)
            return i_index
        
        def index_to_ws(self,i):
            ws_list = self.workstation_points.loc[:,'workstation'].to_list()
            return ws_list[i]
        
        def ws_crit_point(self,i):
            ws_list = self.workstation_points.loc[:,'workstation'].to_list()
            ws_crit_point = self.workstation_points.loc[:,'critical_points'].to_list()
            i_index = ws_list.index(i)
            return ws_crit_point[i_index]
        
        def crit_point_ws(self,i):
            ws_list = self.workstation_points.loc[:,'workstation'].to_list()
            ws_crit_point = self.workstation_points.loc[:,'critical_points'].to_list()
            i_index = ws_crit_point.index(i)
            return ws_list[i_index]
        
        def crit_point_index(self, i):
            row_names = self.critical_points.index.to_list()
            return row_names.index(i)
        
        def index_crit_point(self, i):
            row_names = self.critical_points.index.to_list()
            return row_names[i]
        
        def all_path(self):
            #calculate the path of all workstations that share a load
            #self.all_paths_mtx = np.zeros((len(self.ws_loads[0,:]),len(self.ws_loads[:,0])))
            #all_adj_matrix = [[] for i in range(self.nz)] 2D list
            #this makes a dictionary of paths that share a load between them
            self.all_paths_mtx = {}
            
            for i in range(len(self.workstation_points)):
                i_ws = self.index_to_ws(i)
                for j in range(len(self.workstation_points)):
                    j_ws = self.index_to_ws(j)
                    if (self.ws_loads[i,j] == 0):
                        continue
                    else:
                        crit_ws_i = self.ws_crit_point(self.index_to_ws(i))
                        crit_ws_j = self.ws_crit_point(self.index_to_ws(j))
                        path,dist = self.shortest_dist(crit_ws_i, crit_ws_j, self.adjacency_Mtx)
                        #path = np.array(path)
                        self.all_paths_mtx[i_ws +','+ j_ws] = path
            
            #print(self.all_paths_mtx)

        #functions for finding transfer stations

        def find_transfer_stations(self, zonews, zonecs, all_adj_matrix, roboti, robotj):
            zonecs_all = copy.deepcopy(zonecs)

            all_tip_ws = [[] for i in range(self.num_zone)]
            for zone in range(0,self.num_zone):
                all_tip_ws[zone].append(self.finding_tip_ws(zonews[zone], zonecs_all[zone]))

            size_of_comb = []
            for x in range(0, self.num_zone): 
                size_of_comb.append(x)

            #neighboring_zones = combinations(size_of_comb, 2)
            #neighboring_zones = list(neighboring_zones)
            #neighboring_zone example
            #[(0,1),(0,2),(1,2)]

            #ts = [[] for i in range(len(neighboring_zones))]
            ts = []
            pool_tipws = []
            for zone in range(self.num_zone):
                pool_tipws.append(self.finding_tip_ws(zonews[zone], zonecs_all[zone]))

            #step 1
            #for pair_index in range(0,len(self.neighboring_zones)):
            #roboti = self.neighboring_zones[pair_index][0] #alpha and beta are indexs
            #robotj = self.neighboring_zones[pair_index][1]

            #print("alpha: ", alpha)
            #print("beta: ", beta)

            #step 2 and 3 find TW(alpha and beta)
            #tip workstations adjacent to zone
            roboti_ws = zonews[roboti]

            robotj_ws = zonews[robotj]

            roboti_tip = pool_tipws[roboti]
            robotj_tip = pool_tipws[robotj]

            TW = self.find_TW(roboti_ws, robotj_ws, roboti_tip, robotj_tip)

            SPC_TW_dist = []
            SPC_TW_path = []

            #step 5 collect all valid paths from TW(alpha) and TW(beta)
            #from TW find that closest workstation in beta or alpha
            #TW[0] alpha,  TW[1] beta is in form WS1, WS2 etc...
            if TW[0] is not None and TW[1] is not None:
                #get adj matrix of each zone
                union_mtx = self.union_matrixs(zonecs_all, self.adjacency_Mtx, roboti, robotj, ts)

                #this matrix contains all points in alpha and beta zone as well as points that are available to take
                #testing

                for wsa in TW[0]:
                    wsa_crit = self.ws_crit_point(wsa)
                    for wsb in TW[1]:
                        wsb_crit = self.ws_crit_point(wsb)
                        path, dist = self.shortest_dist(wsa_crit, wsb_crit, union_mtx)
                        if path is not None:
                            SPC_TW_dist.append(dist)
                            SPC_TW_path.append(path)
                
                i=0
                #condition if there was no feasible path
                while len(SPC_TW_path) != 0:

                    #step 6
                    #select the shortest path out of SPC
                    SPC = SPC_TW_path[SPC_TW_dist.index(min(SPC_TW_dist))]

                    #step 7
                    A = self.crit_point_ws(SPC[0]) #first tip ws connected by SP
                    B = self.crit_point_ws(SPC[len(SPC)-1]) #last tip ws connected by SP
                    if((A in ts) or (B in ts)):
                        SPC_TW_path.remove(SPC)
                        SPC_TW_dist.remove(min(SPC_TW_dist))
                        continue
                    
                    CSP = copy.deepcopy(SPC)
                    
                    #numAlpha and numBeta should have at least 1
                    #step 8
                    Lroboti = self.r_load[roboti]
                    Lrobotj = self.r_load[robotj]
                    if Lroboti >= Lrobotj:
                        ts.append(A)

                        #assign crit segment to beta
                        zonecs_all[robotj].append(CSP)
                        all_adj_matrix = self.update_adj_matrixs_ts(all_adj_matrix, zonecs_all, self.adjacency_Mtx, ts)

                    if Lroboti < Lrobotj:
                        ts.append(B)

                        #assign crit segemnt to alpha
                        zonecs_all[roboti].append(CSP)
                        all_adj_matrix = self.update_adj_matrixs_ts(all_adj_matrix, zonecs_all, self.adjacency_Mtx, ts)

                    SPC_TW_path.remove(SPC)
                    SPC_TW_dist.remove(min(SPC_TW_dist))
                    i += 1 #return to step 6

            #print("final map with transfer stations")
            #print(zonecs_all)
            #print(zonews)
            
            #print("Transfer stations (0,1),(0,2),(1,2)")
            #print(ts)

            #self.print_map()
            
            return ts, zonecs_all
        
        def union_matrixs(self, crit_segments, map, alpha, beta, transfer_stations):
            #alpha and beta are indexs
            #only want remove points that are not in alpha or bata
            union_mtx = map.copy()

            combinedcs = []
            for seg in crit_segments[alpha]:
                combinedcs.append(seg)
            for seg in crit_segments[beta]:
                combinedcs.append(seg)

            for zone in range(0,len(crit_segments)):
                #now remove every point in every crit seg not alpha or beta
                if zone != alpha and zone != beta:
                    for path in crit_segments[zone]:
                            union_mtx = self.adj_remove_seg_ts(combinedcs, path,union_mtx, transfer_stations)

            return union_mtx
        
        def adj_remove_seg_ts(self, zonecs, path, adj_matrixs, transfer_stations):
            restart= False
            #zonecs contains the cs in home zone
            for pt in path:
                if pt == path[0] or pt == path[len(path)-1]:
                    compare_ws = self.crit_point_ws(pt)
                    for x in transfer_stations:
                        if (compare_ws in x):
                            restart = True
                    for seg in zonecs:
                        if pt in seg:
                            restart = True
                    if restart:
                        restart = False
                        continue
                index_of_pt = self.crit_point_index(pt)
                #have to remove the point from being used 
                adj_matrixs[index_of_pt, :] = 0
                adj_matrixs[ :, index_of_pt] = 0

            return adj_matrixs
        
        def update_adj_matrixs_ts(self, all_adj_matrixs, crit_segments, map, transfer_stations):
            
            for zone in range(0,len(all_adj_matrixs)):
                #make adj matrix the same as map to start
                all_adj_matrixs[zone] = copy.deepcopy(map)
                
                #now remove every point in every other crit segments
                #points as transferstaions can be treated as if they are not apart of a zone i.e ignored
                for other_zone in range(0,len(crit_segments)):
                    if other_zone != zone:
                        for path in crit_segments[other_zone]:
                                all_adj_matrixs[zone] = self.adj_remove_seg_ts(crit_segments[zone], path,all_adj_matrixs[zone],transfer_stations)

            return all_adj_matrixs

        def find_TW(self, alpha_ws, beta_ws, alpha_tip, beta_tip):
            TW = [[] for i in range(2)]

            workstations = self.workstation_points.loc[:,'workstation'].to_list()

            for ws in alpha_tip:
                ws_index = self.ws_to_index(ws)
                tip_closest = self.workstation_dist_mtx[ws_index]
                tip_closest = list(tip_closest)

                #now filter for those worksations that not in alpha zone
                index = 0
                for ws_dist in tip_closest:
                    ws_compare = workstations[index]
                    index += 1
                    if (ws_dist == 0) or (ws_compare in alpha_ws): #if ws is the tip the tip ws in question
                        continue
                    elif (ws_compare in beta_ws) and (ws_dist <= self.adj_dist):
                        TW[0].append(ws)
                        break

            for ws in beta_tip:
                ws_index = self.ws_to_index(ws)
                tip_closest = self.workstation_dist_mtx[ws_index]
                tip_closest = list(tip_closest)

                #now filter for those worksations that not in beta zone
                index = 0
                for ws_dist in tip_closest:
                    ws_compare = workstations[index]
                    index += 1
                    if (ws_dist == 0) or (ws_compare in beta_ws): #if ws is the tip the tip ws in question
                        continue
                    elif (ws_compare in alpha_ws) and (ws_dist <= self.adj_dist):
                        TW[1].append(ws)
                        break

            return TW

        #functions for sim set-up

        def gen_intialws_q(self):
            routes = pd.read_csv(os.path.join(os.getcwd(), 'data/LE', 'processing_routes_test.csv'), sep=',', header=0, names=['part_type','route','qty'], encoding = 'utf-8')
            
            for i in range(self.numws): #create queues for every workstation
                self.ws_qs.append(Queue(maxsize=0))

            part_types = routes['part_type']
            qty = routes['qty']
            ID_count = 0
            for i in range(len(part_types)): #create part objects and assign them to thier first WS
                route = routes.at[i,'route'].split(',')
                ws_index = int(route[0])-1
                for j in range(int(qty[i])):
                    self.ws_qs[ws_index].put(part(part_types[i], ID_count))
                    ID_count += 1
            
            self.num_parts = ID_count

        def queue_part(self, part, zonews, ts):

            #functions adds part to robot Q assigned to that zone
            currentws = 'WS' + part.get_currentws() #part pick up ws
            nextws = 'WS' + part.get_nextws() #part drop off ws
            trans_ws = ts #get zone transfer stations
            current_zone = -1
            next_zone = -1
            
            if nextws == "WS-1": # check for when part has reached the end
                #print("part finished")
                self.num_fin += 1
                part.add_cycleTime()
                return

            #get the current zone that part is in
            #if part is not a transfer part then the next_zone will be in the same zone as the current_ws
            current_zone = self.find_wszone(currentws, zonews)
            next_zone = self.find_wszone(nextws, zonews)

            #if part has been dropped off at transfer station then the 
            #next_zone will be in the same zone as the current_ws
            #print("part going through zone:", part.get_tsthru())
            if(part.get_transfer()):
                if(part.get_tsthru()): #case for when a part is passing through a zone
                    #print("recieving thru part:", part.get_ID())
                    currentws,nextws = part.at_ts()
                    currentws = "WS" + currentws
                    nextws = "WS" + nextws
                    part.set_transfer(True)
                    current_zone = self.find_wszone(currentws, zonews)
                    next_zone = self.find_wszone(nextws, zonews)
                    if current_zone == next_zone:
                        part.set_transfer(False)

                else: #case for when a part is a transfer part that has a end traget in next zone
                    #print("recieving transfer part:",part.get_ID())
                    currentws,nextws = part.at_ts()
                    currentws = "WS" + currentws
                    nextws = "WS" + nextws
                    current_zone = self.find_wszone(currentws, zonews)
                    next_zone = self.find_wszone(nextws, zonews)
                    part.set_transfer(False)

            #case for when part is to be dropped off at transfer station and is a transfer part
            elif (current_zone != next_zone): 
                #print("recvieving part:",part.get_ID(),"that is to be dropped off at transfer station")
                part.set_transfer(True)

            #print("part:",part.get_ID(),"current zone:", current_zone)
            #print("part:",part.get_ID(),"next zone:", next_zone)

            if (not part.get_transfer()): #case for when the recieving part is not being transfered and being dropped in same zone 
                #print("recieving part:",part.get_ID()," that is going to be droped off in same zone as nextws robot")
                #add part into robot queue or update part in queue
                
                '''
                for part_in in self.robotqs[next_zone-1].keys():
                    if part_in == part.get_ID():
                        self.dickcopy = copy.deepcopy(self.robotqs[next_zone-1])
                        self.dickcopy.pop(part_in)
                        break
                if len(self.dickcopy) == 0:
                    self.robotqs[next_zone-1].update({part.get_ID():part})
                else:
                    self.dickcopy.update({part.get_ID():part})
                    self.robotqs[next_zone-1] = copy.deepcopy(self.dickcopy)
                    self.dickcopy.clear()
                '''
                #self.dickcopy.update({part.get_ID():part})
                self.robotqs[next_zone-1].update({part.get_ID():part})

            else:
                #print("processing transfer part:",part.get_ID())
                #find closest transfer station from current point
                #[(0,1),(0,2),(1,2)] -transfer stations
                tszone = [[1,2],[1,3],[2,3]] #need a better way of getting this
                #need to edit this so thaty it is not depended on the current 3 zone deisgn
                possible_ts = []
                if((current_zone == 1 and next_zone == 2) or (current_zone == 2 and next_zone == 1)): possible_ts = trans_ws[0]
                elif((current_zone == 1 and next_zone == 3) or (current_zone == 3 and next_zone == 1)): possible_ts = trans_ws[1] 
                elif((current_zone == 2 and next_zone == 3) or (current_zone == 3 and next_zone == 2)): possible_ts = trans_ws[2] 
                part.set_tsthru(False)

                #case for when a zone doesnt have a transfer station 
                #move part though to another zone
                possible_zones = []
                if len(possible_ts) == 0:
                    for zonepair in tszone:
                        if (next_zone in zonepair) and (current_zone not in zonepair):
                            possible_zones = copy.deepcopy(zonepair)
                            if possible_zones[0] == next_zone:
                                possible_zone = possible_zones[1]
                            else:
                                possible_zone = possible_zones[0]
                            zonec = 0
                            for zonep in tszone: 
                                if (possible_zone in zonep) and (current_zone in zonep):
                                    possible_ts = trans_ws[zonec]
                                    #print("part going through one zone and into another")
                                    next_zone = possible_zone
                                    part.set_tsthru(True)
                                    break
                                zonec += 1
                            break
                
                #find closest ts_station
                best_ts = self.find_closestTS(currentws, possible_ts)
                #print("best ts:", best_ts)

                #set part as transfer part
                part.set_transfer(True)
                
                #special cases
                #case for when the transfer station is the next ws, just add part to ws queue
                if(nextws == best_ts):
                    #print("part:",part.get_ID(),"next ws is at transfer station")
                    part.set_transfer(False)
                
                #case for when part that has been processed is at a transfer station
                elif(currentws == best_ts):
                    #print("part:",part.get_ID(),"current ws is at the transfer station, just have robot in next zone pick up part")
                    current_zone = next_zone
                    if not part.get_tsthru():
                        best_ts = nextws
                        part.set_transfer(False)
                    else:
                        #print("next possible_ts:", trans_ws[tszone.index(possible_zones)])
                        best_ts = self.find_closestTS(currentws, trans_ws[tszone.index(possible_zones)]) #find the next transfer station
                
                part.going_to_ts(best_ts.replace("WS",""))
                #add part to robot q or update part that already exists in queue
                '''
                for part_in in self.robotqs[next_zone-1].keys():
                    if part_in == part.get_ID():
                        self.dickcopy = copy.deepcopy(self.robotqs[next_zone-1])
                        self.dickcopy.pop(part_in)
                        break
                if len(self.dickcopy) == 0:
                    self.robotqs[next_zone-1].update({part.get_ID():part})
                else:
                    self.dickcopy.update({part.get_ID():part})
                    self.robotqs[next_zone-1] = copy.deepcopy(self.dickcopy)
                    self.dickcopy.clear()
                '''
                #print("size of queue:", len(self.robotqs[next_zone-1]))
                self.robotqs[current_zone-1].update({part.get_ID():part})
                #print("size of queue:", len(self.robotqs[next_zone-1]))

        def find_closestTS(self,currentws,possibleTS):
            #find closest ts station
            ws_dist_mtx = self.workstation_dist_mtx.copy()
            best_ts = possibleTS[0] #default
            ts_dist = 5000
            if len(possibleTS) > 1:
                for ws in possibleTS:
                    new_ts_dist = ws_dist_mtx[self.ws_to_index(currentws)][self.ws_to_index(ws)]
                    if new_ts_dist < ts_dist:
                        best_ts = ws
                        ts_dist = new_ts_dist
            return best_ts

        def find_wszone(self, ws, zonews):
            current_zones = zonews
            for x in range(0,len(current_zones)):
                if ws in current_zones[x]:
                    return x + 1

        #functions for finding average consesus 

        def avg_consenus(self,measurement):
            max_it = 500
            error=np.zeros([max_it,self.num_zone])
            mse=np.zeros([max_it])
            F = measurement.mean()

            neighborList = self.findneighbors(self.num_zone, self.robotxy, 200)
            for it in range(max_it):

                #calculate mean square error
                m=measurement.copy()
                error[it,:]=(F-m[:])
                squaredError=error[it,:]**2
                mse[it]=(np.sum(squaredError))/self.num_zone

                for robot in range(self.num_zone):
                    #calculate weights
                    #maximum degree weigths
                    #weight = maxdegreeweights(numNodes,neighborList,k)
                    #metropolis weights
                    weight = self.metropolisweights(self.num_zone,neighborList,robot)
                    
                    #calculate sum of neighbors weights
                    Ni=neighborList[robot]
                    
                    Niweight=0
                    
                    #calucate neighbor part of measurement
                    if Ni[0] is not None:
                        for j in Ni:
                            Niweight+=weight[j]*measurement[j]
                            #Niweight+= 0.02 + measurement[j]
                    
                    #calculate next measurement
                    measurement[robot] = measurement[robot]*weight[robot] +  Niweight

            #print("calculated average: ", measurement[0])
            #print("actual average:", F)

            '''
            #plot the error
            plt.figure(3)
            plt.plot(error)
            plt.title("Error of all nodes")
            plt.show()
            '''
            return measurement
        
        def findneighbors(self, n, Xi, r):
            allNi=[0]*n
            for j in range(n):
            
                #return this no neighbor
                neighbors=np.array([None])

                #keep track of index
                index=0
                
                #node to find neighbor
                xi=Xi[j]

                #find neighbors
                for i in range(0,n):
                    xj=Xi[i,:]
                    if (np.array_equal(xj,xi) is False):
                        if(np.linalg.norm(xj-xi) < r):
                            neighbors=np.insert(neighbors,index,i)
                            index+=1
                
                if neighbors[0] is not None:
                    neighbors = np.delete(neighbors,index)

                allNi[j]=neighbors
            
            return allNi

        def metropolisweights(self,n,NiList,nodeIndex):
            nodeNeighbor=NiList[nodeIndex]
            nodedegree=len(nodeNeighbor)
            weights=[0]*n
            weightsum=0
            
            if nodeNeighbor[0] is not None:
                for i in nodeNeighbor:
                    Nidegree=len(NiList[i])
                    weights[i]=1/(1+max(nodedegree,Nidegree))
                    weightsum+=weights[i]
            
                weights[nodeIndex]=1-weightsum
                return weights
            else:
                weights[nodeIndex]=1
                return weights

        def maxdegreeweights(self,n,NiList,nodeIndex):
            nodeNeighbor=NiList[nodeIndex]
            degree=len(nodeNeighbor)
            weights=[0]*n
            
            if nodeNeighbor[0] != None:
                for i in range(n):
                    if i == nodeIndex:
                        weights[i]=1-(degree/n)
                    elif i in nodeNeighbor:
                        weights[i]=1/n
                return weights
            else:
                weights[nodeIndex] = 1
                return weights

        #functions for sim

        def re_queue(self):
            for wsq in self.ws_qs: #queue every part in wsq
                for _ in range(wsq.qsize()):
                    part = wsq.get()
                    #part.at_ws()
                    self.queue_part(part)
                    wsq.put(part)

        def find_adjtip(self,i_tip,j_tip):
            #select a random WS to give to robot j that is adjacent
            At = {}
            for wsi in i_tip:
                At[wsi] = []
                for wsj in j_tip:
                    if wsj in self.ws_neighbors[self.ws_to_index(wsi)]:
                        At[wsi].append(wsj)
                if At[wsi] == []:
                    del At[wsi]
            return At.copy()

        def exchange_ws(self, At, robotj):
            while(True):
                if len(At) < 1:
                    break
                tipws = random.choice(list(At.keys())) #choose a random ws to give
                rand_neigh = random.choice(At[tipws])
                
                self.valid_zone(self.zonews)

                #remove tip from zone
                copy_solws, copy_solcs = self.remove_tip_ws(copy.deepcopy(self.zonews), copy.deepcopy(self.zonecs), copy.deepcopy(self.all_adj_matrix), tipws)

                #update adj matrix
                cadj = self.update_adj_matrixs(copy.deepcopy(self.all_adj_matrix), copy_solcs, self.adjacency_Mtx)

                #find path and check
                crit1 = self.ws_crit_point(tipws)
                crit2 = self.ws_crit_point(rand_neigh)
                path, dist = self.shortest_dist(crit1,crit2,cadj[robotj])

                if path is None or dist > self.adj_dist:
                    #restart
                    At[tipws].remove(rand_neigh)
                    if At[tipws] == []:
                        del At[tipws]
                    continue
                else:
                    copy_solws[robotj].append(tipws)
                    copy_solcs[robotj].append(path)
                    cadj = self.update_adj_matrixs(copy.deepcopy(self.all_adj_matrix), copy_solcs, self.adjacency_Mtx)

                    self.valid_zone(self.zonews)
                    self.valid_zone(copy_solws)

                    return copy_solws, copy_solcs, cadj
            
            self.valid_zone(self.zonews)
            return self.zonews, self.zonecs, self.all_adj_matrix

        def load_parts(self, roboti, zone_c, ts):
            #function loads parts that are currently in roboti zone
            #send parts that are currently robot dic to thier WS
            #in this way parts are not copied

            #delete parts from robotq that are in the zone
            #only parts that are being transferred should remain
            #print("\nrobot",roboti,"Q")
            #print(zone_c[roboti])
            
            for _ in range(len(self.robotqs[roboti])):
                #send part to workstations
                ID, part = self.robotqs[roboti].popitem() #remove part from dictionary 
                current_ws = int(part.get_currentws())
                self.ws_qs[current_ws-1].put(part) #give to workstation
            
            #for every ws in system reload parts
            #for real sim, each wrokstation will be in it own thread or ROS node
            #load robots
            for wsq in self.ws_qs:
                for _ in range(wsq.qsize()):
                    part = wsq.get()
                    self.queue_part(part,zone_c,ts)


            '''
            robotd = copy.deepcopy(self.robotqs[roboti])
            for key, value in self.robotqs[roboti].items():
                current_ws = "WS" + str(value.get_currentws())
                next_ws = "WS" + str(value.get_nextws()) #should be the og next_ws
                ID = key
                if(ID == 5):
                    print("repeat part")
                #print("part:", ID)
                #print("current ws",current_ws)
                #print("next ws:",next_ws)
                #filter parts whoose next and current WS is not in robot i zone
                if current_ws not in zone_c[roboti] and next_ws not in zone_c[roboti] and not value.get_tsthru():
                    #print("getting rid of part:", ID)
                    del robotd[key]
            self.robotqs[roboti].clear()
            self.robotqs[roboti] = copy.deepcopy(robotd)
            
            #queue every part in wsq for each zone
            #parts at a transfer station are already saved
            #print("\nrobot",roboti,"Q")
            #print(zone_c[roboti])

            for ws in zone_c[roboti]:
                wsq = self.ws_qs[self.ws_to_index(ws)]
                for _ in range(wsq.qsize()):
                    part = wsq.get()
                    current_ws = "WS" + str(part.get_currentws())
                    next_ws = "WS" + str(part.get_nextws())
                    #ID = part.get_ID()
                    #print("part:", ID)
                    #print("current ws",current_ws)
                    #print("next ws:",next_ws)
                    
                    if ((next_ws not in zone_c[roboti]) and (not part.get_transfer())):
                        #figure out if currentws is a transfer station
                        #do not want to requeue parts that are already saved
                        at_ts = False
                        for ts_set in ts:
                            if current_ws in ts_set:
                                at_ts = True
                                break
                        if at_ts:
                            wsq.put(part)
                            continue
                    
                    part.set_transfer(False)
                    part.set_tsthru(False)
                    self.queue_part(part, zone_c, ts)
                    wsq.put(part)
            '''
            
        def record_load(self, roboti):
            #record the current load of a robot
            self.ws_loads[roboti] = np.zeros((self.numws, self.numws))

            for p in self.robotqs[roboti].values():
                ws1 = p.get_currentws()
                ws2 = p.get_nextws()
                self.ws_loads[roboti][int(ws1)-1][int(ws2)-1] += 1
            
            if np.sum(self.ws_loads[roboti]) == 0:
                print("nothing in roboti")
                print("lenth of roboti i",len(self.robotqs[roboti]))

        def valid_zone(self,zonesws):
            count = 0
            for x in zonesws:
                count += len(x)
            if count == self.numws:
                return True
            else:
                return False

        def calc_error(self, roboti, neighbors, load, avg_load):
            sum_of_diff = abs(load[roboti] - avg_load[roboti]) #remeber each robot calculated average indivisually

            for neigh in neighbors[roboti]:
                sum_of_diff += abs(load[neigh] - avg_load[roboti])
            
            return sum_of_diff/(len(neighbors[roboti])+1) #+1 for roboti

        def check_repeats(self,roboti, robotj):
            #for debugging
            testset1 = list(self.robotqs[roboti].keys())
            testset2 = list(self.robotqs[robotj].keys())
            testset = set(testset1+testset2)
            testsetlen = len(testset)
            dictlength = len(self.robotqs[roboti].keys()) + len(self.robotqs[robotj].keys())
            if testsetlen != dictlength:
                return False
            else:
                return True


if __name__ == '__main__':
    print("in main")
    test = concept()
    test.set_up()
    test.start_sim()