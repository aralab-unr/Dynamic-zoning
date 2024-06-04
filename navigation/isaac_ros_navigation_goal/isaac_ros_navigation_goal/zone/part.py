import numpy as np
import pandas as pd
import time
import rospy
from csv import writer, reader
import os
import rospkg

class part:

    def __init__(self,type,ID_num):
        self.type = type
        self.to_pickup = False
        self.istransfer = False
        self.count_inRoute = 0
        self.ID = ID_num
        self.starting = True

        times = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','processing_time_test.csv'), sep=',', header=0, names=['workstation','processing_time','deviation'], encoding = 'utf-8')
        self.workstations = times['workstation']
        self.processing_times = times['processing_time']

        processing_routes = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','processing_routes_test.csv'), sep=',', header=0, names=['part_type','route','qty'], encoding = 'utf-8')
        #index = processing_routes.loc[(processing_routes == self.type).any(axis=1)].index[0]
        qtylist = processing_routes['qty']
        index = 0
        sum = 0
        for qty in qtylist: 
            sum += qty
            if sum > self.ID:
                break
            index += 1
        print("ID:",self.ID,"index:",index)
        self.route = processing_routes.at[index,'route'].split(',')
        print(self.route)
        self.current_ws = self.route[self.count_inRoute]
        if len(self.route) > 1:
            self.next_ws = self.route[self.count_inRoute+1] #next_ws is just a number "1,2,3"
        else:
            self.next_ws = "-1"

        self.ts_thru = False

        self.start_time = 0

        with open(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','part_status.csv'), 'a') as file:
            line = writer(file)
            row = [ID_num,self.route,self.start_time]
            line.writerow(row)
            file.close()

    def at_ws(self):
        #self.count_inRoute is on the current ws
        self.count_inRoute += 1
        if (len(self.route) <= (self.count_inRoute + 1)): #part has reached the end
            self.next_ws = "-1"
            self.current_ws = self.route[self.count_inRoute]
        else:
            self.current_ws = self.route[self.count_inRoute]
            self.next_ws = self.route[self.count_inRoute + 1]
        self.to_pickup = False
        self.write_status()
    
    def start_processing(self):
        self.starting = False
        proc_time = self.processing_times.at[int(self.current_ws)-1]*60 #time is given in minutes
        if self.count_inRoute == 0:
            self.start_time = rospy.get_rostime()
        time.sleep(proc_time)
        self.to_pickup = True
    
    def at_ts(self):
        self.current_ws = self.next_ws
        self.next_ws = self.route[self.count_inRoute + 1]
        return self.current_ws, self.next_ws
    
    def going_to_ts(self,ts_ws):
        self.next_ws = ts_ws
        self.to_pickup = True

    def get_nextws(self):
        #nextws changes depending on whether it is a transfer part or not
        if self.next_ws == "-1":
            return self.next_ws
        else:
            return self.route[self.count_inRoute + 1]
    
    def get_currentws(self):
        return self.current_ws
    
    def get_pickup(self):
        return self.to_pickup
    
    def set_transfer(self,istrans):
        #istrans is bool
        self.istransfer = istrans
    
    def get_transfer(self):
        return self.istransfer
    
    def set_ID(self, num):
        self.ID = num
    
    def set_tsthru(self,bol):
        self.ts_thru = bol

    def get_ogpickup(self):
        return self.route[self.count_inRoute]

    def get_ID(self):
        return self.ID
    
    def get_tsthru(self):
        return self.ts_thru

    def get_ifstarting(self):
        return self.starting

    def set_startTime(self,timeobj):
        self.start_time = timeobj

    def get_time(self):
        #returns time from start in seconds
        currentTime = rospy.get_rostime()
        return currentTime.secs - self.start_time.secs

    def add_cycleTime(self):
        with open(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','cycle_times.csv'), 'a') as file:
            writerobj = writer(file)
            writelist = [self.get_time(),self.type,self.ID]
            writerobj.writerow(writelist)
            file.close()

    def write_status(self):
        myrow = []
        with open(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','part_status.csv'), 'r') as file:
            alllines = reader(file)
            for row in alllines:
                myrow.append(row)
            file.close()

        rest_route = self.route[self.count_inRoute:]
        myrow[self.ID + 1] = [self.ID, rest_route,self.get_time()]
        with open(os.path.join(rospkg.RosPack().get_path('isaac_ros_navigation_goal'), 'isaac_ros_navigation_goal/zone/data','part_status.csv'), 'w') as file:
            alllines = writer(file)
            for row in myrow:
                alllines.writerow(row)
            file.close()



if __name__ == '__main__':
    newpart = part('D',1)
    #newpart.start_processing()
    newpart.at_ws()
    newpart.at_ws()