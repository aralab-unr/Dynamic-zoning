import numpy as np
import pandas as pd
import time
import rospy
from csv import writer, reader
import os
import rospkg
import copy
#from proof_of_concept import concept #used only for shortest dist

class part:

    def __init__(self, type,ID_num):
        self.type = type
        self.to_pickup = False
        self.istransfer = False
        self.count_inRoute = 0
        self.ID = ID_num
        self.starting = True
        self.ts_thru = False
        self.start_time = 0
        #used to get a measurement on how much time a part has been in queue 
        self.age_start = 0
        self.age = 0
        self.Ca = .5
        self.Cd = 10
        self.dropoff = False
        self.next_zone = 0

        self.workstation_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data/LE','station_points.csv'), sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
 
        times = pd.read_csv(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data/LE','processing_time.csv'), sep=',', header=0, names=['workstation','processing_time'], encoding = 'utf-8')
        self.workstations = times['workstation']
        self.processing_times = times['processing_time']
        variables_set = pd.read_csv(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data/LE', 'robot_parameters.csv'), sep=',', header=0, names=['value'], encoding = 'utf-8')
        values = variables_set['value']
        self.V = values.get('V') #average velocity of the robot
        #get processing route by type and if they are ay diffent stages
        processing_routes = pd.read_csv(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data/LE','processing_routes_test.csv'), sep=',', header=0, names=['part_type','route','qty'], encoding = 'utf-8')
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

        
        with open(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data','part_status.csv'), 'a') as file:
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
        #self.write_status()
    
    def start_processing(self):
        self.starting = False
        proc_time = self.processing_times.at[int(self.current_ws)-1]*2#*60 #time is given in minutes
        if self.count_inRoute == 0:
            self.start_time = rospy.get_rostime()
        rospy.sleep(proc_time)

        self.to_pickup = True
        self.age_start = rospy.get_rostime()
    
    def at_ts(self):
        self.current_ws = self.next_ws #current ws goes to transfer ws
        self.next_ws = self.route[self.count_inRoute + 1] #next ws goes to the end target
        #self.write_status()
        return self.current_ws, self.next_ws
    
    def going_to_ts(self,ts_ws):
        self.next_ws = ts_ws
        self.to_pickup = True

    def get_nextws(self):
        #nextws changes depending on whether it is a transfer part or not. Returns the next ws in its route or the goal ws
        if self.next_ws == "-1" or self.istransfer:
            return self.next_ws
        elif not self.istransfer:
            return self.route[self.count_inRoute + 1]
        
    def get_currentws(self):
        return self.current_ws
    
    def get_pickup(self):
        return self.to_pickup

    def get_dropoff(self):
        return self.dropoff

    def set_dropoff(self,flag):
        self.dropoff = flag

    def set_transfer(self, istrans):
        #istrans is bool
        self.istransfer = istrans
        if not istrans:
            self.next_ws = self.route[self.count_inRoute + 1]
    
    def get_transfer(self):
        return self.istransfer
    
    def set_ID(self, num):
        self.ID = num
    
    def set_tsthru(self,bol):
        self.ts_thru = bol

    def set_nextzone(self,zone):
        self.next_zone = zone

    def get_nextzone(self):
        return self.next_zone

    def get_ogpickup(self):
        return self.route[self.count_inRoute]
    
    def get_ogdropoff(self):
        if self.next_ws != "-1":
            return self.route[self.count_inRoute + 1]
        else:
            return "-1"

    def get_ID(self):
        return self.ID
    
    def get_tsthru(self):
        return self.ts_thru

    def get_ifstarting(self):
        return self.starting

    def get_age(self):
        currentTime = rospy.get_rostime()
        return currentTime.secs - self.age_start.secs

    def set_startTime(self,timeobj):
        self.start_time = timeobj

    def get_time(self):
        #returns time from start in seconds is the cycle time of the part
        currentTime = rospy.get_rostime()
        return currentTime.secs - self.start_time.secs
    
    #recording data
    def add_cycleTime(self):
        with open(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data','cycle_times.csv'), 'a') as file:
            writerobj = writer(file)
            currentTime = rospy.get_rostime()
            writelist = [currentTime.secs, self.get_time(), self.type, self.ID]
            writerobj.writerow(writelist)
            file.close()

    def write_status(self):
        myrow = []
        with open(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data','part_status.csv'), 'r') as file:
            alllines = reader(file)
            for row in alllines:
                myrow.append(row)
            file.close()

        rest_route = self.route[self.count_inRoute:]
        rest_route[0] = copy.deepcopy(self.current_ws)
        myrow[self.ID + 1] = [self.ID, self.type, rest_route, self.age_start.secs, self.get_time(), self.istransfer, self.ts_thru, self.count_inRoute, self.next_ws]
        with open(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data','part_status.csv'), 'w') as file:
            alllines = writer(file)
            for row in myrow:
                alllines.writerow(row)
            file.close()

    #to help reset if robot crashes
    def resetfromstatus(self,ID):
        myrow = []
        with open(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data','part_status.csv'), 'r') as file:
            alllines = reader(file)
            for row in alllines:
                myrow.append(row)
            file.close()
        data = myrow[ID+1]
        self.type = data[1]
        route = data[2]
        self.current_ws = route[0]
        self.age = data[3]
        self.istransfer = data[5]
        self.ts_thru = data[6]
        self.count_inRoute = data[7]
        self.next_ws = data[8]
    
    def reconstruct(self, Map):
        self.ID
        self.type
        self.count_inRoute
        self.current_ws
        self.next_ws
        self.dropoff
        self.to_pickup
        self.age
        self.age_start
        self.ts_thru
        self.istransfer
        self.starting

    #hash and eq
    def __eq__(self, other):
        return other.ID and self.ID == other.ID
    
    def __hash__(self):
        return hash(self.ID)

if __name__ == '__main__':
    newpart = part('D',1)
    #newpart.start_processing()
    #newpart.at_ws()
    #newpart.at_ws()