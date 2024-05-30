import numpy as np
import pandas as pd
import math
import copy
import random
import matplotlib.pyplot as plt
from .Phase_1 import Phase1
from .Phase_2 import Phase2
from itertools import combinations 

#setting up transfer stations and assigning remaining critical segments
class Phase3(Phase2):
    def __init__(self, zone_ws, crit_seg, allAdjMatrix):
        self.zone_ws = copy.deepcopy(zone_ws)
        self.zone_cs = copy.deepcopy(crit_seg)
        self.num_zones = len(zone_ws)
        self.phase2 = Phase2() #need this to find workload Lp
        self.phase1 = Phase1() #want to use the shortest path function
        self.phase1.create_map()

        self.critical_points = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/Critical_Points.csv', sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        self.workstation_loc = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/Workstation_Loaction.csv', sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        self.workstation_points = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/Workstation_points.csv', sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        workstation_dist_mtx = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/WS_dist_mtx.csv', sep=',')
        #self.all_adj_matrix = []
        #for x in range(0, len(zone_ws)):
            #filepath = 'data/adj'+str(x)+'.csv'
            #mtx = pd.read_csv(filepath, sep=',')
            #self.all_adj_matrix.append(mtx.to_numpy())

        self.all_adj_matrix = copy.deepcopy(allAdjMatrix)

        self.workstation_dist_mtx = workstation_dist_mtx.to_numpy()
        self.adj_matrix = copy.deepcopy(self.phase1.adjacency_Mtx) #map with distances
        self.all_points = copy.deepcopy(self.critical_points.index.to_list()) #for testing
        self.adj_dist = 350

    def setting_transfer_stations(self):
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

        #step 1
        #select two neighboring zones that have not already gone through the process
        for pair_index in range(0,len(self.neighboring_zones)):
            alpha = self.neighboring_zones[pair_index][0] #alpha and beta are indexs
            beta = self.neighboring_zones[pair_index][1]

            #print("alpha: ", alpha)
            #print("beta: ", beta)

            #step 2 and 3 find TW(alpha and beta)
            #tip workstations adjacent to zone
            alpha_ws = self.zone_ws[alpha]
            alpha_crit = self.zone_cs[alpha]

            beta_ws = self.zone_ws[beta]
            beta_crit = self.zone_cs[beta]

            alpha_tip = self.finding_tip_ws(alpha_ws,alpha_crit)
            beta_tip = self.finding_tip_ws(beta_ws,beta_crit)

            TW = self.find_TW(alpha_ws, beta_ws, alpha_tip, beta_tip)

            SPC_TW_dist = []
            SPC_TW_path = []

            #step 5 collect all valid paths from TW(alpha) and TW(beta)
            #from TW find that closest workstation in beta or alpha
            #TW[0] alpha,  TW[1] beta is in form WS1, WS2 etc...
            if TW[0] != None and TW[1] != None:
                #get adj matrix of each zone
                union_mtx = self.union_matrixs(self.zone_cs, self.adj_matrix, alpha, beta, self.transfer_stations)

                #this matrix contains all points in alpha and beta zone as well as points that are available to take
                #testing
                #df_union = pd.DataFrame(union_mtx)
                #df_union.to_csv('/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/union_mtx.csv', header=self.all_points)

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

                    
                    #step 6
                    #select the shortest path out of SPC
                    SPC = SPC_TW_path[SPC_TW_dist.index(min(SPC_TW_dist))]

                    #step 7
                    A = self.crit_point_ws(SPC[0]) #first
                    B = self.crit_point_ws(SPC[len(SPC)-1]) #last
                    CSP = SPC.copy()
                    CSP = np.array(CSP)

                    numAlpha = 0 #counting the numbe of points in a crit segment
                    numBeta = 0
                    SCSP = np.array([])
                    GCSP = np.array([])
                    #if statements
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
                    
                    #numAlpha and numBeta should have at least 1
                    #step 8
                    if numBeta == 1 and numAlpha == 1:
                        Lalpha = self.phase2.zone_load(self.zone_ws[alpha], all_tip_ws)
                        Lbeta = self.phase2.zone_load(self.zone_ws[beta], all_tip_ws)
                        if Lalpha >= Lbeta:
                            self.transfer_stations[pair_index].append(A)
                            #assign crit segment to beta
                            self.zone_cs[beta].append(CSP)
                            self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zone_cs, self.adj_matrix, self.transfer_stations)

                        if Lalpha < Lbeta:
                            self.transfer_stations[pair_index].append(B)
                            #assign crit segemnt to alpha
                            self.zone_cs[alpha].append(CSP)
                            self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zone_cs, self.adj_matrix, self.transfer_stations)

                        SPC_TW_path.remove(SPC)
                        SPC_TW_dist.remove(min(SPC_TW_dist))
                        i += 1 #return to step 6

                    #step 9
                    elif numBeta == 1 and numAlpha > 1:
                        self.transfer_stations[pair_index].append(B)
                        #assign crit segment to alpha
                        first = True
                        RCSP = np.array([])
                        for i in range(0,len(CSP)):
                            if(CSP[i] not in SCSP):
                                if first:
                                    RCSP = np.append(RCSP, CSP[i-1])
                                    first = False
                                RCSP = np.append(RCSP, CSP[i])
                        self.zone_cs[alpha].append(RCSP.tolist())
                        self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zone_cs, self.adj_matrix, self.transfer_stations)

                        SPC_TW_path.remove(SPC)
                        SPC_TW_dist.remove(min(SPC_TW_dist))
                        i += 1 #return to step 6
                    
                    #step 10
                    elif numBeta > 1 and numAlpha == 1:
                        self.transfer_stations[pair_index].append(A)
                        #assign crit segment to beta
                        last = False
                        FCSP = np.array([])
                        for i in range(0,len(CSP)):
                            if(CSP[i] not in GCSP):
                                FCSP = np.append(FCSP, CSP[i])
                            else:
                                FCSP = np.append(FCSP, CSP[i])
                                break

                        self.zone_cs[beta].append(FCSP.tolist())
                        self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zone_cs, self.adj_matrix, self.transfer_stations)

                        SPC_TW_path.remove(SPC)
                        SPC_TW_dist.remove(min(SPC_TW_dist))
                        i += 1 #return to step 6
                    
                    elif numBeta > 1 and numAlpha > 1:
                        SPC_TW_path.remove(SPC)
                        SPC_TW_dist.remove(min(SPC_TW_dist))
                        i += 1 #return to step 6


        print("final map with transfer stations")
        print(self.zone_cs)
        print(self.zone_ws)
        print("Transfer stations (0,1),(0,2),(1,2)")
        print(self.transfer_stations)

        return

    def assigning_crit_seg(self):
        #assign type 1 crit segments to thier repective zone 
        #type 1 is a crit segment that can only be assigned to one zone
        #testing
        #self.zone_ws = [['WS8', 'WS9', 'WS10'], ['WS3', 'WS5', 'WS1', 'WS7'], ['WS2', 'WS4', 'WS6', 'WS11']]
        #self.zone_cs = [[['d','V','W','c'], ['c','f','g','b'], ['V', 'U'], ['g', 'h', 'a']], [['P', 'Q', 'R', 'S'], ['P', 'Q', 'H', 'D', 'E'], ['S', 'T', 'U'], ['H', 'I']], [['I', 'J', 'K'], ['K', 'M', 'L', 'X'], ['X', 'Y', 'Z', 'a'], ['b', 'Y'], ['E', 'F', 'G', 'Z']]]

        #print("starting type 1 crit segs")
        for zone in range(0,self.num_zones):
            list_points = []
            prev = None
            #want just a list of points in zone that are not in a 2D array
            for segment in self.zone_cs[zone]:
                for pt in segment:
                    if pt != prev:
                        list_points.append(pt)
                        prev = pt
            
            for i in range(0,len(list_points)-1):
                pt_i = list_points[i] #current point
                for j in range(i+1,len(list_points)):
                    if i == j:
                        continue
                    pt_j = list_points[j] #next point
                    path, dist = self.phase1.shortest_dist(pt_i, pt_j, self.all_adj_matrix[zone])
                    if path != None: #if there is a path
                        if not self.isin(path,list_points): #if path is not already in crit segments
                            path.reverse()
                            if not self.isin(path,list_points): 
                                if len(path) < 3: #simple point to point case when both points are in the same zone
                                    self.zone_cs[zone].append(path)
                                #now we need to check if the path could have a segment attached to another zone
                                #recursive function
                                elif self.if_path_isolated(path, path[0], self.zone_cs, zone, self.adj_matrix):
                                    self.zone_cs[zone].append(path)
                                    #only time adj needs to be updated
                                    self.all_adj_matrix = self.update_adj_matrixs(self.all_adj_matrix, self.zone_cs, self.adj_matrix, self.transfer_stations)
        
        #print("After adding Type 1 segments")
        #print(self.zone_cs)
        #print(self.zone_ws)

        #adding type 2 segments
        #find GS the critical segments not assigned to a zone
        #print("starting type 2 crit segs")
        GS = []
        in_zone = False
        for point in self.all_points:
            for zone in self.zone_cs:#used to figure out if a point is in a zone
                if self.is_pt_in_zone(point, zone):
                    in_zone = True
                    break
                else:
                    in_zone = False
                    
            if not in_zone:
                neighbor = self.phase1.get_neighbors(point,self.adj_matrix)
                for neigh_pt in neighbor:
                    seg=[point,neigh_pt]
                    if not seg in GS: 
                        if not seg.reverse() in GS:
                            GS.append(seg)
        
        #from GS random select a segment that is directly linked to a zone
        #print("start of while loop")
        #print(self.zone_cs)
        while len(GS) > 0:
            #print("at the top of loop")
            #print("len of GS:",len(GS))
            #print(GS)
            H = ''
            while H == '':
                #print("top of inside loop")
                #print("H:", H)
                segment = GS[random.randint(0,len(GS)-1)]
                point_found = False
                #print(segment)
                for zone in self.zone_cs:
                    if self.is_pt_in_zone(segment[0], zone) or self.is_pt_in_zone(segment[1], zone):
                        point_found = True
                        H = segment
                        break
                    if point_found:
                        break
                if point_found:
                    break
        
            #from the selected point find all segments that are attached to zones
            #pt1_neighbors = self.phase1.get_neighbors(H[0],self.adj_matrix)
            #pt2_neighbors = self.phase1.get_neighbors(H[1],self.adj_matrix)
            DLC = {}
            
            for zone in range(len(self.zone_cs)):
                if self.is_pt_in_zone(H[1], self.zone_cs[zone]):
                    DLC.setdefault(H[1], zone)
                if self.is_pt_in_zone(H[0], self.zone_cs[zone]):
                    DLC.setdefault(H[0], zone)
            #from DLC select a random segment and add to zone
            segment_pt, zone = random.choice(list(DLC.items()))
            self.zone_cs[zone].append(H)
            #testing
            #print("zone 0")
            #print(self.zone_cs[0])
            #print("zone 1")
            #print(self.zone_cs[1])
            #print("zone 2")
            #print(self.zone_cs[2])            
            GS.remove(H)

        #testing
        #print("zone 0")
        #print(self.zone_cs[0])
        #print("zone 1")
        #print(self.zone_cs[1])
        #print("zone 2")
        #print(self.zone_cs[2]) 

        #fix adjacency matrix
        #reset each adj matrix
        #print("resting each adj matrix")
        for zone in range(len(self.zone_cs)): self.all_adj_matrix[zone] = self.adj_matrix

        for zone in range(len(self.zone_cs)):
            for other_zone in range(len(self.zone_cs)):
                if zone != other_zone:
                    crit_seg = self.zone_cs[other_zone]
                    for segment in crit_seg:
                        for pt in range(len(segment)-1):
                            pt_index = self.crit_point_index(segment[pt])
                            next_pt_index = self.crit_point_index(segment[pt+1])
                            matrix = copy.deepcopy(self.all_adj_matrix[zone])
                            matrix[pt_index,next_pt_index] = 0
                            matrix[next_pt_index,pt_index] = 0
                            self.all_adj_matrix[zone] = copy.deepcopy(matrix)
                                    #testing
                            #df_adj1 = pd.DataFrame(self.all_adj_matrix[0])
                            #df_adj2 = pd.DataFrame(self.all_adj_matrix[1])
                            #df_adj3 = pd.DataFrame(self.all_adj_matrix[2])
                        
                            #df_adj1.to_csv('data/adj0.csv', header=self.all_points, index=self.all_points)
                            #df_adj2.to_csv('data/adj1.csv', header=self.all_points, index=self.all_points)
                            #df_adj3.to_csv('data/adj2.csv', header=self.all_points, index=self.all_points)

        #testing
        #df_adj1 = pd.DataFrame(self.all_adj_matrix[0])
        #df_adj2 = pd.DataFrame(self.all_adj_matrix[1])
        #df_adj3 = pd.DataFrame(self.all_adj_matrix[2])
    
        #df_adj1.to_csv('/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/adj0.csv', header=self.all_points, index=self.all_points)
        #df_adj2.to_csv('/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/adj1.csv', header=self.all_points, index=self.all_points)
        #df_adj3.to_csv('/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/adj2.csv', header=self.all_points, index=self.all_points)

        '''
        #ploting 
        #first plot points
        x_points = np.array([self.critical_points.get('x')])
        y_points = np.array([self.critical_points.get('y')])

        plt.plot(x_points, y_points, 'ob')
        #now draw lines where there is a segment in a zone
        zone_color = np.array(['#0000FF','#FF7F50', '#FF1493', '#FFD700', '#7CFC00'])
        line_ty = np.array(['-', ':', '--', '-.'])
        index = 0
        for adj_matrix in self.all_adj_matrix:
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
        '''

        return
    
    #function used to see if path has a segment attached to another zone
    def if_path_isolated(self, path, start_pt, crit_seg, zone, adj_mtx):
        newPath = path.copy()
        while(len(newPath) > 0): #used to iterate through the neighbors
            if self.is_pt_in_zone(newPath[0], crit_seg[zone]): #if the point is within the same zone
                newPath.remove(newPath[0])
            else:
                #check if point is in another zone
                for other_zone in range(0,len(crit_seg)):
                    if other_zone != zone:
                        check = self.is_pt_in_zone(newPath[0],crit_seg[other_zone]) #if neighbor point in other zone then path is not isolated
                        if check:
                            return False
                if not check: #if all neighbor pt not in other zone
                    neighbors = self.phase1.get_neighbors(newPath[0], adj_mtx)
                    neighbors = [i for i in neighbors if i not in path] #want to explore only unvisted points
                    try:
                        neighbors.remove(start_pt)
                    except:
                        start_pt = newPath[0]
                    #newPath.remove(newPath[0])
                    check2 = self.if_path_isolated(neighbors,newPath[0],crit_seg, zone, adj_mtx) #check to see if a point was found in another zone
                    newPath.remove(newPath[0])
                    if check2:
                        continue #move to next element in newPath
                    else:
                        return check2 #found a point in another zone
        return True
                
    def is_pt_in_zone(self,pt,crit_seg):
        for segment in crit_seg:
            if pt in segment:
                return True
        return False

    #function to see is A list in in B list
    def isin(self, A, B):
        first = True
        for i in range(len(A)):
            element = A[i]
            try:
                index = B.index(element)
            except:
                index = None

            if index == None or first:
                first = False
                continue
            elif B[index-1] == A[i-1]:
                return True
            elif index+1 != len(B):
                if B[index+1] == A[i-1]:
                    return True
        return False

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

            #now filter for those worksations that not in alpha zone
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

        for zone in range(0,len(crit_segments)):
            #now remove every point in every crit seg not alpha or beta
            if zone != alpha and zone != beta:
                for path in crit_segments[zone]:
                        union_mtx = self.adj_remove_seg(path,union_mtx, transfer_stations)

        return union_mtx
    
    def adj_remove_seg(self, path, adj_matrixs, transfer_stations):
        restart= False
        for pt in path:
            if pt == path[0] or pt == path[len(path)-1]:
                compare_ws = self.crit_point_ws(pt)
                for x in transfer_stations:
                    if (compare_ws in x):
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
        
        for x in range(0,len(all_adj_matrixs)):
            #make adj matrix the same as map to start
            all_adj_matrixs[x] = copy.deepcopy(map)
            
            #now remove every point in every other crit segments
            #points as transferstaions can be treated as if they are not apart of a zone i.e ignored
            for i in range(0,len(crit_segments)):
                if i != x:
                    for path in crit_segments[i]:
                            all_adj_matrixs[x] = self.adj_remove_seg(path,all_adj_matrixs[x],transfer_stations)

        return all_adj_matrixs
    #getters
    def get_zone(self):
        return self.zone_cs
    
    def get_transferstations(self):
        return self.transfer_stations

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
