# Import libraries
import numpy as np
import pandas as pd
from collections import defaultdict
import math
import heapq
import copy
import os
import rospkg

class Phase1:
      def __init__(self):
            self.read_csv()

      def read_csv(self):
            #var = os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone', 'Variables_of_Zone_Warehouse_Phase1.csv')
            #print(var)
            variables_set = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data', 'Variables_of_Zone_Warehouse_Phase1.csv'), sep=',', header=0, names=['value'], encoding = 'utf-8')
            self.critical_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','Critical_Points.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
            self.workstation_loc = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','Workstation_Loaction.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
            self.adjacency_Mtx_pd = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','Adjacency_matrix.csv'), sep=',', header=0, names=['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f','g','h','i'], encoding = 'utf-8')
            self.workstation_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','Workstation_points.csv'), sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
            self.workstation_loads = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','Workstation_Loads.csv'), sep=',', header=0, names=['WS1','WS2','WS3','WS4','WS5','WS6','WS7','WS8','WS9','WS10','WS11'], encoding = 'utf-8')
            
            values = variables_set['value']

            self.Wd = values.get('Wd')
            self.Wf = values.get('Wf')
            self.V = values.get('V')
            self.tl = values.get('tl')
            self.tu = values.get('tu')
            self.T = values.get('T')
            self.lt = values.get('lt')

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
            
            #adjacency matrix now consists of distances to neighbors not just 0 and 1

      
      def start_phase1(self):
            #RWS is the set of worksations not assigned to a zone
            RSW = self.workstation_points.loc[:,'workstation'].to_list()

            #all_points is the set of all points in grid
            #all_points = self.critical_points.index.to_list()

            #matrix to find distances of all points
            #could make this faster by only computing dist from x->y and not repeating the same dist from y->x
            self.num_Work_Stations = len(RSW)
            self.workstation_dist_mtx = np.zeros((self.num_Work_Stations,self.num_Work_Stations))
            for x in range(0,self.num_Work_Stations):
                  nodex = self.workstation_points.at[x,'critical_points']
                  for y in range(0,self.num_Work_Stations):
                        if(x != y):
                              nodey = self.workstation_points.at[y,'critical_points']
                              dist_path,dist = self.shortest_dist(nodex,nodey,self.adjacency_Mtx)
                              self.workstation_dist_mtx[x,y] = dist
                        else:
                              self.workstation_dist_mtx[x,y] = 0

            #get MAXSD and MINSD
            self.MAXSD = self.workstation_dist_mtx.max()
            self.MINSD = math.inf
            for x in range(0,self.num_Work_Stations):
                  temp_list = self.workstation_dist_mtx[x,:].tolist()
                  temp_list.sort()
                  if self.MINSD > temp_list[1]:
                        self.MINSD = temp_list[1]

            #turn worksation loads into a matrix
            self.ws_loads = self.workstation_loads.to_numpy()

            #maximum and minum flow in the system
            self.MAXF = self.ws_loads.max()
            self.MINF = self.ws_loads.min()

            #define RC as a matrix
            RC = np.zeros((self.num_Work_Stations,self.num_Work_Stations))
            UC = np.zeros((self.num_Work_Stations,self.num_Work_Stations))

            #get the number of zones
            self.nz = self.number_of_zones()
            print(self.nz)
            self.nz = round(self.nz)

            #calculate the unrelated coefficient between every two workstations
            #unrelated coefficient = 1 - RCij
            for i in range(0,len(RSW)):
                  for j in range(0,len(RSW)):
                        if i != j:
                              fij = int(self.ws_loads[i,j]) #get the number of loads between workstations
                              sdij = self.workstation_dist_mtx[i,j] #shoartest dist betwen two points

                              FRij = (fij - self.MINF)/(self.MAXF - self.MINF)
                              DRij = (self.MAXSD - sdij)/(self.MAXSD - self.MINSD)

                              RC[i,j] = (self.Wd * DRij)+(self.Wf * FRij)
                              UC[i,j] = 1 - ((self.Wd * DRij)+(self.Wf * FRij))
            
            #find nz number of worstations whose unrealated coefficients is highest
            highest_UC = np.sum(UC,axis=0)
            #SWz = np.zeros((self.nz, self.num_Work_Stations))
            SWz=[[] for i in range(self.nz)]
            #assign the top nz to zones and remove from RWS
            for x in range(0,self.nz):
                  node_index = np.argmax(highest_UC)
                  workstation = self.workstation_points.at[node_index,'workstation']
                  SWz[x].append(workstation)
                  highest_UC[node_index] = 0
                  RSW.remove(SWz[x][0])
            
            #expand the area of each zone
            #GWz = worksations in zone z
            self.GW = copy.deepcopy(SWz)
            prev_GW = copy.deepcopy(self.GW)

            #GSz = critical segments of zone z
            self.GS = [[] for i in range(self.nz)]

            K=0 #zone selector

            #each zone must have its own adjcacency matrix because crit segments can use the segments already assigned to the zone
            self.all_adj_matrix = [[] for i in range(self.nz)]
            for x in range(0,self.nz):
                  self.all_adj_matrix[x] = copy.deepcopy(self.adjacency_Mtx)
  
            #step 4
            #for K in range(0,self.nz):
            while (K<self.nz):
                  #make a copy of GW that is not nested
                  prev_GW = copy.deepcopy(self.GW)

                  RC_matrix = np.zeros((len(prev_GW[K]), len(RSW)))
                  #Calculate the relationship coefficient between every workstation in GW(K) and every workstation in RWS.
                  for i in range(0,len(prev_GW[K])):
                        for j in range(0,len(RSW)):
                              ws_GW=prev_GW[K][i]
                              ws_RSW = RSW[j]
                              RC_matrix[i,j] = self.find_RC(ws_GW,ws_RSW,self.all_adj_matrix[K])

                  #check to make sure that not every number is 0 in RC Matrix
                  if RC_matrix.sum() > 0:

                        i_indcies = np.argmax(RC_matrix,axis=0) #4.4
                        j_indcies = np.argmax(RC_matrix,axis=1)
                        i_indcies = max(i_indcies)
                        j_indcies = max(j_indcies)

                        ws_GWK = prev_GW[K][i_indcies]
                        ws_RSW = RSW[j_indcies]

                        point_GWK = self.ws_crit_point(ws_GWK)
                        point_RWS = self.ws_crit_point(ws_RSW)

                        shortest_path,dist = self.shortest_dist(point_GWK,point_RWS,self.all_adj_matrix[K])#4.5
                        #self.GS[K] = self.add_cs(self.GS[K],shortest_path)
                        self.GS[K].append(shortest_path)

                        for x in range(0,self.nz): #update the other adj matrix
                              if (x != K):
                                    self.all_adj_matrix[x] = self.update_adj(shortest_path,self.all_adj_matrix[x])
                        
                        self.GW[K].append(ws_RSW)
                        RSW.remove(ws_RSW)

                        if len(RSW) == 0:   #4.6
                              break

                  if(K>=self.nz-1):   #4.7
                        K = 0
                  else:
                        K+=1

                  if len(RSW) == 0:   #4.6 to break out of the while loop
                        break
                  
                  self.Wd += 0.015
                  self.Wf -= 0.015
            #print(self.GW)
            #print(self.GS)
            
      def add_cs(self, crit_seg, path):
            for point in path:
                  if point not in crit_seg:
                        crit_seg.append(point)
            return crit_seg

      def update_adj(self, path, adj_matrix):

            all_points = self.critical_points.index.to_list()
            for pt in path:
                  index_of_pt = all_points.index(pt)
                  #have to remove the point from being used 
                  adj_matrix[index_of_pt, :] = 0
                  adj_matrix[ :, index_of_pt] = 0

            return adj_matrix

      def find_RC(self,i,j,adj_matirix):
            #i,j are in WS
            workstations = self.workstation_points.loc[:,'workstation'].to_list()
            crit_workstations = self.workstation_points.loc[:,'critical_points'].to_list()

            index_of_pt = workstations.index(i)
            index_of_next_pt = workstations.index(j)
            crit_point = crit_workstations[index_of_pt]
            next_crit_point = crit_workstations[index_of_next_pt]

            fij = int(self.ws_loads[index_of_pt,index_of_next_pt]) #get the number of loads between workstations

            path, sdij = self.shortest_dist(crit_point,next_crit_point,adj_matirix) #shortest dist betwen two points

            if(sdij == 0 or sdij < 1): #condition if there is not fesiable path
                  return 0

            FRij = (fij - self.MINF)/(self.MAXF - self.MINF)
            DRij = (self.MAXSD - sdij)/(self.MAXSD - self.MINSD)

            return (self.Wd * DRij)+(self.Wf * FRij)
            
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

      def number_of_zones(self):
            g = np.zeros((self.num_Work_Stations,self.num_Work_Stations))
            DA = np.zeros((self.num_Work_Stations,self.num_Work_Stations))
            DB = np.zeros((self.num_Work_Stations,self.num_Work_Stations))

            going_in = np.sum(self.ws_loads, axis=0)
            going_out = np.sum(self.ws_loads, axis=1)
            total_number_of_loads = np.sum(self.ws_loads)

            for i in range(self.num_Work_Stations):
                  nodei = self.workstation_points.at[i,'critical_points']
                  for j in range(self.num_Work_Stations):
                        nodej = self.workstation_points.at[j,'critical_points']
                        g[i,j]=going_in[i]*going_out[j]/(total_number_of_loads)
                        
                        distij = self.get_distance(nodei,nodej)
                        DA[i,j]=g[i,j]*distij
                        DB[i,j]=self.ws_loads[i,j]*distij

            total_dist_traveled = (np.sum(DA)+np.sum(DB))
            TT = (total_dist_traveled/self.V) + (total_number_of_loads)*(self.tu + self.tl)
            self.nv = TT/(60*self.T - self.lt)
            return self.nv
      
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
      
      def get_distance(self,i,j):
            #i and j are characters
            #get row as a dataframe and then convert to numpy
            point1 = self.critical_points.loc[i]
            point1 = point1.to_numpy()
            point2 = self.critical_points.loc[j]
            point2 = point2.to_numpy()
            #distance = math.dist(point1,point2)
            return math.dist(point1,point2)

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



if __name__ == '__main__':
    print("in main")
    test = Phase1()
    test.read_csv()
    test.create_map()
    test.start_phase1()