import numpy as np
import copy
import pandas as pd
import os
import heapq
import math
import random
import rospkg

class Zone_func():
    def __init__(self, num_zone, adj_dist):
        variables_set = pd.read_csv(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data/LE', 'robot_parameters.csv'), sep=',', header=0, names=['value'], encoding = 'utf-8')
        self.critical_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data/LE','critical_points.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        self.workstation_loc = pd.read_csv(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data/LE','station_location.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        self.adjacency_Mtx_pd = pd.read_csv(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data/LE','map.csv'), sep=',', header=0, names=list(self.critical_points.index.values), encoding = 'utf-8')
        self.workstation_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data/LE', 'station_points.csv'), sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        self.workstation_loads = pd.read_csv(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data/LE','station_loads.csv'), sep=',', header=0, names=list(self.workstation_loc.index.values), encoding = 'utf-8')
        values = variables_set['value']

        self.Wd = values.get('Wd')
        self.Wf = values.get('Wf')
        self.V = values.get('V')
        self.tl = values.get('tl')
        self.tu = values.get('tu')
        self.T = values.get('T')
        self.lt = values.get('lt')
        self.Ca = values.get('Ca')
        self.Cd = values.get('Cd')

        #number of workstations
        self.numws = len(self.workstation_points['workstation'])

        self.num_zone = num_zone
        self.adj_dist = adj_dist

        self.create_map()

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

        self.ws_neighbors = self.WS_neighbors(self.adj_dist)

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
        if (prev[j] is None):
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
    
    def WS_neighbors(self, adj_dist):
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
    
    def crit_point_pos(self, i):
        x = self.critical_points.at[i,'x']
        y = self.critical_points.at[i,'y']
        return np.array((x,y))

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

    def find_transfer_stations(self, zonews, zonecs, all_adj_matrix, roboti, robotj, loadij):
        #loadij = the load of i and j = [53.0, 96.3]
        self.all_adj_matrix = all_adj_matrix

        zonecs_all = copy.deepcopy(zonecs)

        all_tip_ws = [[] for i in range(self.num_zone)]
        for zone in range(0,self.num_zone):
            all_tip_ws[zone].append(self.finding_tip_ws(zonews[zone], zonecs_all[zone]))

        size_of_comb = []
        for x in range(0, self.num_zone): 
            size_of_comb.append(x)

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
                    if path is not None and dist <= self.adj_dist:
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
                if ((loadij[0] == 0) and (loadij[1] == 0)) or ((loadij[0] is None) or (loadij[1] is None)):
                    Lroboti = roboti
                    Lrobotj = robotj
                else:
                    Lroboti = copy.deepcopy(loadij[0])
                    Lrobotj = copy.deepcopy(loadij[1])

                if Lroboti >= Lrobotj:
                    ts.append(A)

                    #assign crit segment to beta
                    zonecs_all[robotj].append(CSP)
                    self.all_adj_matrix = self.update_adj_matrixs_ts(self.all_adj_matrix, zonecs_all, self.adjacency_Mtx, ts)

                elif Lroboti < Lrobotj:
                    ts.append(B)

                    #assign crit segemnt to alpha
                    zonecs_all[roboti].append(CSP)
                    self.all_adj_matrix = self.update_adj_matrixs_ts(self.all_adj_matrix, zonecs_all, self.adjacency_Mtx, ts)

                SPC_TW_path.remove(SPC)
                SPC_TW_dist.remove(min(SPC_TW_dist))
                i += 1 #return to step 6

        #print("final map with transfer stations")
        #print(zonecs_all)
        #print(zonews)
        
        #print("Transfer stations (0,1),(0,2),(1,2)")
        #print(ts)

        #self.print_map()
        
        return ts, zonecs_all, self.all_adj_matrix
    
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