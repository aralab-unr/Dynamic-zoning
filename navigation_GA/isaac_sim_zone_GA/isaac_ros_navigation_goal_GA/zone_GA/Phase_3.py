import numpy as np
import pandas as pd
import math
import copy
import random
import matplotlib.pyplot as plt
from .Phase_1 import Phase1
from .Phase_2 import Phase2
from itertools import combinations 
import os
import rospkg

#setting up transfer stations and assigning remaining critical segments
class Phase3(Phase2):
    def __init__(self, zone_ws, crit_seg, allAdjMatrix):
        self.zone_ws = copy.deepcopy(zone_ws)
        self.zone_cs = copy.deepcopy(crit_seg)
        self.num_zones = len(zone_ws)
        self.phase2 = Phase2() #need this to find workload Lp
        self.phase1 = Phase1() #want to use the shortest path function
        self.phase1.create_map()

        self.critical_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_GA'), 'isaac_ros_navigation_goal_GA/zone_GA/data/LE','Critical_Points.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        self.workstation_loc = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_GA'), 'isaac_ros_navigation_goal_GA/zone_GA/data/LE','Workstation_Loaction.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        self.workstation_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_GA'), 'isaac_ros_navigation_goal_GA/zone_GA/data/LE','Workstation_points.csv'), sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        workstation_dist_mtx = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_GA'), 'isaac_ros_navigation_goal_GA/zone_GA/data/LE','WS_dist_mtx.csv'), sep=',')

        self.all_adj_matrix = copy.deepcopy(allAdjMatrix)

        self.workstation_dist_mtx = workstation_dist_mtx.to_numpy()
        self.adj_matrix = copy.deepcopy(self.phase1.adjacency_Mtx) #map with distances
        self.all_points = copy.deepcopy(self.critical_points.index.to_list()) #for testing

        self.adj_dist = self.phase2.adj_dist #allow for more transfer stations to connect to differnt zones

    def setting_transfer_stations(self):
        #for SA and GA we will find transfer stations between all zones
        all_tip_ws = [[] for i in range(self.num_zones)]
        for zone in range(0,self.num_zones):
            all_tip_ws[zone].append(self.finding_tip_ws(self.zone_ws[zone],self.zone_cs[zone]))

        size_of_comb = []
        for x in range(0,self.num_zones): size_of_comb.append(x)

        self.neighboring_zones = combinations(size_of_comb, 2)
        self.neighboring_zones = list(self.neighboring_zones)
        #neighboring_zone example
        #[(0,1),(0,2),(1,2)]

        self.transfer_stations = [[] for i in range(len(self.neighboring_zones))]
        pool_tipws = []
        for zone in range(self.num_zones):
            pool_tipws.append(self.finding_tip_ws(self.zone_ws[zone],self.zone_cs[zone]))

        #og_zone_cs = copy.deepcopy(self.zone_cs)

        #select two neighboring zones that have not already gone through the process
        for pair_index in range(0,len(self.neighboring_zones)):
            alpha = self.neighboring_zones[pair_index][0] #alpha and beta are indexs
            beta = self.neighboring_zones[pair_index][1]

            #print("alpha: ", alpha)
            #print("beta: ", beta)

            #tip workstations adjacent to zone
            #A
            alpha_ws = self.zone_ws[alpha]
            #B
            beta_ws = self.zone_ws[beta]

            #Atip and Btip
            alpha_tip = pool_tipws[alpha]
            beta_tip = pool_tipws[beta]

            #contains TWA and TWB
            TW = self.find_TW(alpha_ws, beta_ws, alpha_tip, beta_tip)

            SPC_TW_dist = []
            SPC_TW_path = []

            #collect all valid paths from TW(alpha) and TW(beta)
            #from TW find that closest workstation in beta or alpha
            #TW[0] alpha,  TW[1] beta is in form WS1, WS2 etc...
            if TW[0] != None and TW[1] != None:
                #get adj matrix of each zone
                #ABP = union matrix #this matrix contains all points in alpha and beta zone as well as points that are available to take
                union_mtx = self.union_matrixs(self.zone_cs, self.adj_matrix, alpha, beta, self.transfer_stations)

                #SPC = SPC_TW_path and SPC_TW_dist
                for wsa in TW[0]:
                    wsa_crit = self.ws_crit_point(wsa)
                    for wsb in TW[1]:
                        wsb_crit = self.ws_crit_point(wsb)
                        path, dist = self.phase1.shortest_dist(wsa_crit, wsb_crit, union_mtx)
                        if path != None:
                            SPC_TW_dist.append(dist)
                            SPC_TW_path.append(path)
                
                i=0
                #condition if there was no feasible path
                while len(SPC_TW_path) != 0:
                    #print(self.zone_ws)
                    #print(self.zone_cs)

                    #select the shortest path out of SPC will be used create CSP
                    SPC = SPC_TW_path[SPC_TW_dist.index(min(SPC_TW_dist))]

                    WS_alpha = self.crit_point_ws(SPC[0]) #first tip ws connected by SP
                    WS_beta = self.crit_point_ws(SPC[len(SPC)-1]) #last tip ws connected by SP
                    if((WS_alpha in self.transfer_stations[pair_index]) or (WS_beta in self.transfer_stations[pair_index])):
                        SPC_TW_path.remove(SPC)
                        SPC_TW_dist.remove(min(SPC_TW_dist))
                        continue
                       
                    CSP = copy.deepcopy(SPC)
                    CSP = np.array(CSP)

                    """
                    #figuring out if a portion of the path is in alpha or beta
                    numAlpha = 0 #counting the number of points in a crit segment
                    numBeta = 0
                    SCSP = np.array([])
                    GCSP = np.array([])

                    #need to find out if there are crit segments in CSP belonging to alpha or beta
                    for point in CSP:
                        for Asegment in self.zone_cs[alpha]:
                            if point in Asegment:
                                numAlpha += 1
                                SCSP = np.append(SCSP, point) #subset of points belonging to alpha
                        for Bsegment in self.zone_cs[beta]:
                            if point in Bsegment:
                                numBeta += 1
                                GCSP = np.append(GCSP, point) #subset of points belonging to beta
                    """

                    Lalpha = self.phase2.zone_load(self.zone_ws[alpha], all_tip_ws[alpha])
                    Lbeta = self.phase2.zone_load(self.zone_ws[beta], all_tip_ws[beta])
                    if Lalpha >= Lbeta:
                        self.transfer_stations[pair_index].append(WS_alpha)

                        #assign crit segment to beta
                        self.zone_cs[beta].append(CSP.tolist())
                        self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zone_cs, self.adj_matrix, self.transfer_stations)

                    if Lalpha < Lbeta:
                        self.transfer_stations[pair_index].append(WS_beta)

                        #assign crit segemnt to alpha
                        self.zone_cs[alpha].append(CSP.tolist())
                        self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zone_cs, self.adj_matrix, self.transfer_stations)

                    SPC_TW_path.remove(SPC)
                    SPC_TW_dist.remove(min(SPC_TW_dist))
                    i += 1 #return to step 6

        print("final map with transfer stations")
        print(self.zone_cs)
        print(self.zone_ws)
        
        print("Transfer stations (0,1),(0,2),(1,2)")
        print(self.transfer_stations)

        #self.print_map()
        
        return

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
    
    #A tip workstaion is defined as a ws that only has one branch connecting to it
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
            
            
            row = self.adj_matrix[self.crit_point_index(ws_crit_point)]
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
        try:
            i_index = ws_list.index(i)
        except:
            return None
        return ws_crit_point[i_index]
    
    def crit_point_ws(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        ws_crit_point = self.workstation_points.loc[:,'critical_points'].to_list()
        try:
            i_index = ws_crit_point.index(i)
        except:
            return None

        return ws_list[i_index]
    
    def crit_point_index(self, i):
        #row_names = self.critical_points.index.to_list()
        return self.all_points.index(i)
    
    def index_crit_point(self, i):
        row_names = self.critical_points.index.to_list()
        return row_names[i]

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
                        union_mtx = self.adj_remove_seg(combinedcs, path,union_mtx,transfer_stations)

        return union_mtx
    
    def adj_remove_seg(self, zonecs, path, adj_matrixs, transfer_stations):
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
            index_of_pt = self.all_points.index(pt)
            #have to remove the point from being used 
            adj_matrixs[index_of_pt, :] = 0
            adj_matrixs[ :, index_of_pt] = 0

        return adj_matrixs
    
    def update_adj_matrixs(self, all_adj_matrixs, crit_segments, map, transfer_stations):
        
        for zone in range(0,len(all_adj_matrixs)):
            #make adj matrix the same as map to start
            all_adj_matrixs[zone] = copy.deepcopy(map)
            
            #now remove every point in every other crit segments
            #points as transferstaions can be treated as if they are not apart of a zone i.e ignored
            for other_zone in range(0,len(crit_segments)):
                if other_zone != zone:
                    for path in crit_segments[other_zone]:
                            all_adj_matrixs[zone] = self.adj_remove_seg(crit_segments[zone], path,all_adj_matrixs[zone],transfer_stations)

        return all_adj_matrixs

    #getters
    def get_zone(self):
        return self.zone_cs
    
    def get_transferstations(self):
        return self.transfer_stations
    
    def print_map(self):
        #ploting 
        #first plot points
        x_points = np.array([self.critical_points.get('x')])
        y_points = np.array([self.critical_points.get('y')])

        #plt.plot(x_points, y_points, 'ob')
        #now draw lines where there is a segment in a zone
        zone_color = np.array(['#0000FF','#FF7F50', '#FF1493', '#FFD700', '#7CFC00'])
        line_ty = np.array(['-', ':', '--', '-.'])
        index = 0
        for adj_matrix in self.all_adj_matrix:
            plt.plot(x_points, y_points, 'ob')
            for row in range(len(adj_matrix)):
                for col in range(len(adj_matrix)):
                    if adj_matrix[row,col] != 0:
                        x_line = np.array([])
                        y_line = np.array([])
                        x_line = np.append(x_line,x_points[0][row])
                        y_line = np.append(y_line,y_points[0][row])
                        x_line = np.append(x_line,x_points[0][col])
                        y_line = np.append(y_line,y_points[0][col])
                        plt.plot(x_line,y_line, ls = '-', color = zone_color[index])
            index += 1
            plt.show()
        #display transfer stations
        for ws_seg in self.transfer_stations:
            for ws in ws_seg:
                if ws == None:
                    break
                else:
                    point = self.ws_crit_point(ws)
                    point_index = self.crit_point_index(point)
                    plt.plot(x_points[0][point_index], y_points[0][point_index], 'Dr')

        plt.show()


if __name__ == '__main__':
    print("in main")

    test_zone_ws = [['WS9', 'WS8', 'WS10'], ['WS3', 'WS5', 'WS1', 'WS7'], ['WS2', 'WS4', 'WS6', 'WS11']]
    test_crit_segments = [[['c', 'W', 'V', 'd'], ['d', 'e', 'f', 'g', 'b']], [['P', 'Q', 'R', 'S'], ['P', 'Q', 'H', 'D', 'E'], ['S', 'T', 'U']], [['I', 'J', 'K'], ['K', 'M', 'L', 'X'], ['X', 'Y', 'Z', 'a']]]
    #workstation_points = pd.read_csv(r'Workstation_points.csv', sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
    #workstation_loads = pd.read_csv(r'Workstation_Loads.csv', sep=',', header=0, names=['WS1','WS2','WS3','WS4','WS5','WS6','WS7','WS8','WS9','WS10','WS11'], encoding = 'utf-8')
    #adjacency_Mtx_pd = pd.read_csv(r'Adjacency_matrix.csv', sep=',', header=0, names=['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f','g','h','i'], encoding = 'utf-8')
    #adj_matrix = adjacency_Mtx_pd.to_numpy()
    #ws_loads = workstation_loads.to_numpy()

    #(self,zone_work_stations,zone_crit_segments,work_station_loads)
    print(test_zone_ws)
    print(test_crit_segments)
    test = Phase3(test_zone_ws,test_crit_segments)
    test.setting_transfer_stations()
    test.assigning_crit_seg()
    #test.SA_improvement()
    #test.read_csv()
    #test.create_map()
    #test.start_phase1()
