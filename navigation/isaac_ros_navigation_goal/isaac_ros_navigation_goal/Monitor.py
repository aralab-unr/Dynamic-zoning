#!/usr/bin/env python
import pandas as pd
import numpy as np
import math
import rospy
import copy
from zone.Phase_2 import Phase2
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from isaac_ros_navigation_goal.msg import stringarr
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool


class Monitoring_module(Phase2):
    
    def __init__(self):
        #get data from phase2 module
        super().__init__()
        self.current_ws = []
        self.current_cs = []

        #pub sub to talk to robot_tasks
        self.status_pub = rospy.Publisher("status", String, queue_size = 10)
        rospy.Subscriber("/rec_loads",numpy_msg(Floats),self.set_loads)
        rospy.Subscriber("/zones",stringarr,self.decode_zones)
        rospy.Subscriber("/critseg",stringarr,self.decode_critseg)

        #pub sub to talk to control
        rospy.Subscriber("/monitor_action",String,self.action) #for NA and A control
        rospy.Subscriber("/send_status",Bool,self.send_status) #signal to send status
        self.take_action = "NA"
        self.status = "NA"
        
        self.workstation_points = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/Workstation_points.csv', sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        self.num_WS = len(self.workstation_points)
        self.rec_loads = np.zeros((self.num_WS,self.num_WS),dtype=np.float32)

        self.zonecount = 0

        self.ph2 = Phase2()
    
    def monitor_status(self):

        print(self.rec_loads)
        print("current zones")
        print(self.current_ws)
        print("current crit segments")
        print(self.current_cs)

        self.num_of_zones = len(self.current_ws)
        self.all_tip_ws = [[] for i in range(self.num_of_zones)]
        self.ph2.set_wsloads(self.rec_loads)

        self.all_path()

        for zone in range(0,self.num_of_zones):
            self.all_tip_ws[zone].append(self.ph2.finding_tip_ws(self.current_ws[zone],self.current_cs[zone]))

        #calculate how balenced the system is
        if(self.rec_loads.sum() > 0):
            SVp = self.ph2.calc_SVp(self.current_ws,self.current_cs)
        else:
            self.status = "NA"
            return

        if(SVp > 0.2 or SVp == 0):
            #entering intesive monitoring scheme
            print("system is not balanced SVp:",SVp)
            self.status = "imbalenced"
        else:
            print("system is balanced SVp:",SVp)
            self.status = "balenced"
    '''
    def calc_SVp(self ,all_ws, all_tip):
        TZLDp = 0
        Lpz = []
        #first calulate Lpz for every zone
        for zone in range(0,self.num_of_zones):
            Lpz.append(self.zone_load(all_ws[zone], all_tip))

        for zprime in range(0,self.num_of_zones-1):
            for zdprime in range(zprime+1,self.num_of_zones):
                TZLDp += abs(Lpz[zprime] - Lpz[zdprime])

        SVp = TZLDp/(sum(Lpz) * (self.num_of_zones-1))

        return(SVp)

    def zone_load(self, ws_in_zone, zone_tip):
        #equations 11-14
        #finding Lpz
        #to find Lpz you need to find fpzij with is the load specifically in the zone
        #print("finding Lpz")

        all_fpzij = self.finding_zone_all_fpzij(ws_in_zone, zone_tip)

        sum_DA_DB = 0

        #finding get gpzij
        for i in ws_in_zone:
            for j in ws_in_zone:
                if (i != j):
                    gpzij = self.finding_gpzij(i,j,ws_in_zone,all_fpzij, zone_tip)
                    
                    index_i = self.ws_to_index(i)
                    index_j = self.ws_to_index(j)
                    dpzij = self.workstation_dist_mtx[index_i,index_j]
                    DApzij = gpzij * dpzij

                    fpzij = self.finding_fpzij(i,j, zone_tip)
                    DBpzij = fpzij*dpzij

                    sum_DA_DB += DApzij + DBpzij
        
        Lpz = (sum_DA_DB/self.Vel) + (all_fpzij*(self.tu + self.tl))
        
        if math.isnan(Lpz):
            Lpz = 0

        return Lpz

    def finding_gpzij(self,i,j,ws_in_zone, all_fpzij, zone_tip):
        #finding fpzki and fpzjk
        fpzki = 0
        fpzjk = 0
        for k in ws_in_zone:
            if(k!=i):
                fpzki += self.finding_fpzij(k,i, zone_tip)
            if(k!=j):
                fpzjk += self.finding_fpzij(j,k, zone_tip)
        return (fpzki*fpzjk)/all_fpzij

    def finding_zone_all_fpzij(self,ws_in_zone, zone_tip):
        all_fpzij = 1
        for i in ws_in_zone:
            for j in ws_in_zone:
                if(i != j):
                    all_fpzij += self.finding_fpzij(i,j, zone_tip)
        return all_fpzij
    
    def finding_fpzij(self,i,j, zone_tip):
        #If i is a tip workstation, then fpzij includes
        #not only fij (which is the flow originating from i and going to j),
        # - but also the flow that originates from workstations in other zones,
        #   passes i, and arrives at j
        # - and the flow that originates from workstations in other zones, passes i, passes j, and arrives at workstations
        #   in other zones.
        # i and j are in the form 'ws1'


        #is i a tip workstation?
        is_tip = False
        #tip_ws = zone_tip
        for x in range(0,self.num_of_zones):
            x_list = zone_tip[x][0]
            if (i in x_list):
                is_tip = True
                break

        i_index = self.ws_to_index(i)
        j_index = self.ws_to_index(j)

        crit_i = self.ws_crit_point(i)
        crit_j = self.ws_crit_point(j)

        #assumed that i and j are already in the same zone
        fpzij = 0

        if is_tip:
            #case for when i is a tip ws and j is in the zone 
            #need to calculate the flow

            #this could be done better
            for wsi_index in range(0,len(self.workstation_points)):
                wsi = self.index_to_ws(wsi_index)

                for wsj_index in range(0,len(self.workstation_points)):
                    wsj = self.index_to_ws(wsj_index)

                    path = self.all_paths_mtx[wsi + ',' + wsj]
                    if (path != 0):
                        if (path[0] != crit_j):
                            if (crit_i in path and crit_j in path):
                                fpzij += self.rec_loads[wsi_index,wsj_index]
        else:
            fpzij = self.rec_loads[i_index,j_index]

        return fpzij

    def all_path(self):
        #calculate the path of all workstations that share a load
        #self.all_paths_mtx = np.zeros((len(self.ws_loads[0,:]),len(self.ws_loads[:,0])))
        #all_adj_matrix = [[] for i in range(self.nz)] 2D list
        #this makes a dictionary of paths that share a load between them
        self.all_paths_mtx = {}
        
        for i in range(0,len(self.workstation_points)):
            i_ws = self.index_to_ws(i)
            for j in range(0,len(self.workstation_points)):
                j_ws = self.index_to_ws(j)
                if (self.rec_loads[i,j] == 0):
                    continue
                    #self.all_paths_mtx[i_ws +','+ j_ws] = 0
                else:
                    crit_ws_i = self.ws_crit_point(self.index_to_ws(i))
                    crit_ws_j = self.ws_crit_point(self.index_to_ws(j))
                    path,dist = self.phase1.shortest_dist(crit_ws_i, crit_ws_j, self.adj_matrix)
                    #path = np.array(path)
                    self.all_paths_mtx[i_ws +','+ j_ws] = path
    '''

    def index_to_ws(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        return ws_list[i]

    def ws_to_index(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        i_index = ws_list.index(i)
        return i_index
    
    def ws_crit_point(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        ws_crit_point = self.workstation_points.loc[:,'critical_points'].to_list()
        i_index = ws_list.index(i)
        return ws_crit_point[i_index]
    
    def get_status(self):
        return self.status
    
    def set_loads(self,data):
        rospy.loginfo("rec loads msg recived")
        self.rec_loads = self.decode_numpy(data.data,self.num_WS,self.num_WS)

    def decode_numpy(self,nparray,numrow,numcol):
            np_recv = np.zeros((numrow,numcol),dtype=np.float32) 
            for x in range(0,numrow):
                for y in range(0,numcol):
                    np_recv[x,y] = nparray[(y+(numrow*x))]
            return np_recv

    def decode_zones(self,data):
        #rospy.loginfo("zone msg recived")
        zone = data.data
        self.append_list(zone,"ws",0)
        
    def decode_critseg(self,data):
        #rospy.loginfo("crit msg recived")
        crit_seg = data.data
        zonei = int(crit_seg[0])
        del crit_seg[0]
        self.append_list(crit_seg,"cs",zonei)

    def send_status(self,data):
        if data.data == True:
            msg = String()
            msg.data = self.status
            self.status_pub.publish(msg)

    def action(self,data):
        self.take_action = data.data

    def get_action(self):
        return self.take_action
    
    def reset_zones(self):
        self.current_ws.clear()
        self.current_cs.clear()

    def append_list(self,list,cs_ws,zone):
        if cs_ws == "ws":
            self.current_ws.append(list)
            self.current_cs.append([])
        elif cs_ws == "cs":
            self.current_cs[zone].append(list)


def main():
    rospy.init_node("monitor_node") #create node
    rospy.loginfo("monitoring module running")
    newMonitor = Monitoring_module()
    start_time = rospy.get_rostime()
    
    #need to set up how monitoring module interacts with control module
    
    newMonitor.reset_zones()
    send_load = rospy.Publisher("send_load", Bool, queue_size = 10)
    send_zone = rospy.Publisher("send_zone", Bool, queue_size = 10)
    load_flag = Bool()
    zone_flag = Bool()
    load_flag.data = False
    zone_flag.data = True

    #wait until monitor is activated by the contol module
    while(newMonitor.get_action() == "NA"):
        continue 

    while not rospy.is_shutdown():
        newaction = newMonitor.get_action()

        #update timer
        current_time = rospy.get_rostime()
        timer = (current_time.secs)-(start_time.secs)
        
        #when control tells monitor module to go into intensive monitoring
        if newaction == "a_IM" and (timer/60) >= 3: #change to 5 later
            #update every 5 min
            print("**********adopting intesive monitoring**********")
            newMonitor.reset_zones()
            send_zone.publish(zone_flag)
            rospy.sleep(1) 
            load_flag.data = True
            send_load.publish(load_flag)
            load_flag.data = False
            send_load.publish(load_flag)
            rospy.sleep(.1) 
            newMonitor.monitor_status()
            start_time = rospy.get_rostime() #reset clock
            rospy.sleep(1) 

        #when control tells monitor module to go into normal monitoring
        elif(newaction == "a_NM" and (timer/60) >= 6): 
            print("*********adopting normal monitoring*************")
            newMonitor.reset_zones()
            send_zone.publish(zone_flag)
            rospy.sleep(1) 
            load_flag.data = True
            send_load.publish(load_flag)
            load_flag.data = False
            send_load.publish(load_flag)
            rospy.sleep(.1) 
            start_time = rospy.get_rostime() #reset clock
            newMonitor.monitor_status()
            rospy.sleep(1) #only want to hit this once for timer
        
        
        rospy.sleep(1)
    
    
if __name__ == "__main__":
    main()