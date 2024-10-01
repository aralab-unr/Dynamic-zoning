#!/usr/bin/env python
from __future__ import absolute_import
import rospkg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd
import os
import rospy
from queue import Queue
import concurrent.futures
import time
import heapq
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from thesis_sim.msg import Zone
from thesis_sim.msg import Reload
from thesis_sim.msg import Position
from thesis_sim.msg import Goal
from assests.part import part
from assests.zone_support import Zone_func
import copy
import yaml
import sys

class Map(Zone_func):
    def __init__(self):
        
        map_pd = pd.read_csv(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data/LE','map.csv'), sep=',', header=0, index_col=0, encoding = 'utf-8')
        self.critical_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data/LE','critical_points.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        self.station_loc = pd.read_csv(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data/LE','station_location.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        variables_set = pd.read_csv(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data/LE', 'robot_parameters.csv'), sep=',', header=0, names=['value'], encoding = 'utf-8')

        self.num_robot = rospy.get_param('/num_robots') #total number of robots in the system

        #inheit the zone support class
        super().__init__(self.num_robot, 200)

        #convert map to np matrix
        self.map = map_pd.to_numpy()

        #get required variables
        values = variables_set['value']
        self.V = values.get('V') #example

        #get number of stations
        self.num_s = len(self.station_loc)

        #intialize ws queues
        self.station_qs = {} #parts go into this before being procecced
        self.ws_part_holder = {} #place to hold parts that are waiting to be given to the next ws: {WS1:{partID: part obj}} 
        for s in self.station_loc.index:
            self.station_qs[s] = Queue(maxsize=0)
            self.ws_part_holder[s] = {} 

        #parameters
        self.num_robot = rospy.get_param('/num_robots')

        #publishers
        self.robot_part_pub = []
        for robotID in range(1,self.num_robot+1):
            self.robot_part_pub.append(rospy.Publisher("/robot"+str(robotID)+"/part_recieve", Goal, queue_size=50))#part publisher

        #suscribers
        rospy.Subscriber("/robot_claim", Zone, self.set_robot) #suscriber to have robot claim WS
        for roboti in range(1,self.num_robot+1): #suscribe to all other robot positions 
            ns = "/robot" + str(roboti) + "/robot_pos"
            rospy.Subscriber(ns, Position, self.update_robot_pos)
        rospy.Subscriber("/robot_part", Goal, self.update_part) #suscriber for when parts are dropped off at WS

    
        #local ws parameters
        self.robot_claim = {} #stores info about what robot owns what station
        self.num_fin = 0 #counting the number of finined parts
        self.robot_pos = {} #hold robot positions (robotID: (x,y))

        
    def load_parts(self):
        routes = pd.read_csv(os.path.join(rospkg.RosPack().get_path('thesis_sim'), 'scripts/data/LE', 'processing_routes_test.csv'), sep=',', header=0, names=['part_type','route','qty'], encoding = 'utf-8')

        part_types = routes['part_type']
        qty = routes['qty']
        ID_count = 0
        for i in range(len(part_types)): #create part objects and assign them to thier first WS
            route = routes.at[i,'route'].split(',')
            station = 'WS' + route[0] #station is written as "WS1"
            for j in range(int(qty[i])):
                self.station_qs[station].put(part(part_types[i], ID_count))
                ID_count += 1
                rospy.sleep(0.01)
        
        self.num_parts = ID_count #keep trak of the total number of parts in the system

    def display(self,i):

        plt.cla()

        #get map xy for every point and connect to neighbor
        x_points = np.array([self.critical_points.get('x')])
        y_points = np.array([self.critical_points.get('y')])

        #plot points
        plt.plot(x_points, y_points, 'ok', markersize = 3)

        for row in range(len(self.map)):
            for col in range(len(self.map)):
                if self.map[row,col] != 0:
                    x_line = np.array([])
                    y_line = np.array([])
                    x_line = np.append(x_line,x_points[0][row])
                    y_line = np.append(y_line,y_points[0][row])
                    x_line = np.append(x_line,x_points[0][col])
                    y_line = np.append(y_line,y_points[0][col])
                    plt.plot(x_line,y_line, ls = '-', color = 'k')
        
        #plot workstaions
        station_x = np.array([self.station_loc.get('x')])
        station_y = np.array([self.station_loc.get('y')])
        plt.plot(station_x, station_y, 'sb', markersize = 4)

        
        #plot robot positions
        robot_color = np.array(['#0000FF','#FF7F50', '#FF1493', '#FFD700', '#7CFC00'])
        for robot, pos in self.robot_pos.items():
            roboti = int(''.join(filter(lambda i: i.isdigit(), robot)))
            plt.plot(pos[0],pos[1], 'D' ,markersize = 5, color = robot_color[roboti])
        
        #plt.show()
    
    def rundisplay(self):
        self.ani = FuncAnimation(plt.gcf(), self.display)
        plt.show()

    def start_station(self, station):
        #function processes the part that is in the queue of the station
        #this continually runs parts in the queue
        #wsQ is the workstationQ
        #station is written as 'WS1'
        #stationq = self.station_qs[station] #pointer to queue
        print("creating thread for:", station)
        #print("compare",self.num_fin, self.num_parts)
        while(self.num_fin < self.num_parts): #need a better indicator for when to kill the thread
            stationq = self.station_qs[station] #update station q
            if not stationq.empty():
                newpart = stationq.get() #dequeues from ws Q 
                #if newpart.get_ifstarting(): #for measuring throughput
                    #self.procunits += 1
                #print("Start processing part:",newpart.get_ID(),"in", station)
                newpart.start_processing()
                #print("Done processing part:",newpart.get_ID(),"in", station)
                self.queue_part(newpart, station) 
                self.ws_part_holder[station][str(newpart.get_ID())] = newpart
            rospy.sleep(.01)    
    
    def queue_part(self, part, WS):
        print("queueing up part:", part.get_ID())

        if part.get_transfer():
            robotID = part.get_nextzone()
        else:
            robotID = int(''.join(filter(lambda i: i.isdigit(), self.robot_claim[WS])))

        #establish zonews and ts
        with open('robot'+str(robotID)+'.yaml','r') as file: #read yaml file containing zone information
            newmap = yaml.safe_load(file)
        
        #extract info about zone to build zonews
        zonews = [[] for _ in range(self.num_robot)]
        for neighbor, zone in newmap['neighbor'].items():
            zone_nID = int(''.join(filter(lambda i: i.isdigit(), neighbor))) #to extract the number in /robot1/
            zonews[zone_nID-1] = zone

        zonews[robotID-1] = newmap['stations'] #zonews is are the zones assciocated with the robot [[WS2,WS3],[],[WS9,WS8,WS10]]...
        trans_ws = newmap['transfer_stations'] #get zone transfer stations

        #functions adds part to robot Q assigned to that zone
        currentws = 'WS' + part.get_currentws() #part pick up ws
        nextws = 'WS' + part.get_nextws() #part drop off ws
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

        #if current_zonet != current_zone:
            #print("mybe not updated yaml correctly")

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
                current_zone = robotID #robotID is equal to the next zone
                next_zone = self.find_wszone(nextws, zonews)
                if current_zone == next_zone:
                    part.set_transfer(False)

            else: #case for when a part is a transfer part that has a end traget in next zone
                #print("recieving transfer part:",part.get_ID())
                currentws,nextws = part.at_ts()
                currentws = "WS" + currentws
                nextws = "WS" + nextws
                current_zone = robotID #robotID is equal to the next zone
                next_zone = self.find_wszone(nextws, zonews)
                part.set_transfer(False)

        #case for when part is to be dropped off at transfer station and is a transfer part
        elif (current_zone != next_zone or next_zone is None): 
            #print("recvieving part:",part.get_ID(),"that is to be dropped off at transfer station")
            part.set_transfer(True)

        #print("part:",part.get_ID(),"current zone:", current_zone)
        #print("part:",part.get_ID(),"next zone:", next_zone)

        if (not part.get_transfer()): #case for when the recieving part is not being transfered and being dropped in same zone 
            #print("recieving part:",part.get_ID(),"that is going to be droped off in same zone", next_zone)
            #add part into robot queue or update part in queue

            msg = Goal()
            msg.goal1 = currentws
            msg.goal2 = nextws
            msg.partID = str(part.get_ID())
            msg.age = part.get_age()
            self.robot_part_pub[next_zone-1].publish(msg)

            part.write_status()

        else:
            #print("processing transfer part:",part.get_ID())
            #find closest transfer station from current point
            tszone = list(trans_ws.keys())

            possible_ts = [] 
            if next_zone in tszone: #check to see if zone has a transfer station to the next zone
                possible_ts = copy.deepcopy(trans_ws[next_zone])

            #case for when a zone doesnt have a transfer station to the next zone
            #move part though to another zone
            #dont know neighbor transfer station, just drop off part closet to final WS
            part.set_tsthru(False)
            if possible_ts == []:
                part.set_tsthru(True)
                #print("part:",part.get_ID(),"is a thru part")
                for ts_set in trans_ws.values(): #add all ts to the list of possible transfer stations
                    possible_ts += ts_set #appending a list to a list

            if possible_ts == []:
                print("zone transfer stations")
                print(trans_ws)
                while(True):
                    continue

            #find best ts
            best_ts = self.find_closestTS(currentws, nextws, possible_ts, part.get_tsthru())

            #print("best ts:", best_ts)
            #set next zone
            for zonei in tszone:
                if best_ts in trans_ws[zonei]:
                    next_zone = zonei

            #set part as transfer part
            part.set_transfer(True)
            
            #special cases
            #case for when the transfer station is the next ws, just add part to current robot queue
            if(nextws == best_ts):
                #print("part:",part.get_ID(),"next ws is at transfer station")
                part.set_transfer(False)
            
            #case for when part that has been processed is at a transfer station, just add part to next robot queue
            elif(currentws == best_ts):
                #print("part:",part.get_ID(),"current ws is at the transfer station, just have robot in next zone pick up part")
                current_zone = next_zone
                if not part.get_tsthru():
                    best_ts = nextws
                    part.set_transfer(False)
                if currentws == best_ts:
                    print("is thru", part.get_tsthru())
                    print("is transfer", part.get_transfer())
                    print("robot ID", robotID)
                    print("next_zone", next_zone)
                    print("current zone", current_zone)
                    print("current ws", currentws)
                    print("zonews", zonews)
                    part.going_to_ts(best_ts.replace("WS",""))
                    part.set_nextzone(next_zone)
                    self.queue_part(part,currentws)
                    return
            
            part.going_to_ts(best_ts.replace("WS",""))
            #send part to appropiate robot Q

            #print("size of queue:", len(self.robotqs[next_zone-1]))
            #self.robotqs[current_zone-1].update({part.get_ID():part})
            #print("adding transfer part", part.get_ID(), "to robot",current_zone,"queue")
            #print("next zone", next_zone)
            part.set_nextzone(next_zone)
            msg = Goal()
            msg.goal1 = currentws
            msg.goal2 = best_ts
            msg.partID = str(part.get_ID())
            msg.age = part.get_age()
            try:
                self.robot_part_pub[current_zone-1].publish(msg)
            except:
                print("could not find the zone that this part is currently in")
                print(robotID)
                print(current_zone)
                print(next_zone)
                print(msg.partID)
                print(WS)
                print(msg.goal1)
                print(msg.goal2)
                print(robotID)
                print(self.robot_claim)
                print(zonews)
                while(True):
                    continue
            
            part.write_status()

    def find_closestTS(self, currentws, nextws, possibleTS, isthru):
        #find closest ts station
        ws_dist_mtx = self.workstation_dist_mtx
        try:
            best_ts = possibleTS[0] #default
        except IndexError:
            print("///////////couldn't find closest TS//////////")
            print(isthru)
            print(possibleTS)
            

        ts_dist = math.inf
        if len(possibleTS) > 1: #assuming zone is not completely disconnected 
            if (not isthru): #if the next transfer station is in the next zone
                for ws in possibleTS: #return the closet transfer station to the current ws
                    new_ts_dist = ws_dist_mtx[self.ws_to_index(currentws)][self.ws_to_index(ws)]
                    if new_ts_dist < ts_dist:
                        best_ts = ws
                        ts_dist = new_ts_dist
            else:
                for ws in possibleTS: #return the closet transfer station to the final ws
                    new_ts_dist = ws_dist_mtx[self.ws_to_index(nextws)][self.ws_to_index(ws)]
                    if new_ts_dist < ts_dist:
                        best_ts = ws
                        ts_dist = new_ts_dist

        return best_ts

    def find_wszone(self, ws, zonews):
        for x in range(len(zonews)):
            if ws in zonews[x]:
                return x + 1
        return None #case for when current zone is not connected to next zone

    #helper functions
    def get_num_s(self):
        return self.num_s
    
    def get_stations(self):
        return list(self.station_loc.index)

    #callbacks
    def set_robot(self, data):
        self.robot_claim[data.stations[0]] = data.robot

    def update_robot_pos(self, pos_msg):
        self.robot_pos[pos_msg.robot] = [pos_msg.x, pos_msg.y]
        #print(self.robot_pos)

    def update_part(self, part_msg):
        partID = part_msg.partID
        prevws = part_msg.goal1
        currentws = part_msg.goal2
        reloadtf = part_msg.reload

        print("updating part:", partID)
        #get part
        try:
            part = copy.deepcopy(self.ws_part_holder[prevws][partID])
            del self.ws_part_holder[prevws][partID] #delete from ws part holder
        except:
            print("!!!!!!!key error!!!!!!!!")
            print(prevws)
            print(currentws)
            print(partID)
            print(self.ws_part_holder[prevws])
            print("break")

        #case for when part is being reloaded
        if reloadtf:
            #print("reloading part:", partID)
            part.set_transfer(False) #reset transfer and through status
            part.set_tsthru(False)
            currentws = part_msg.goal1 #reset current ws
            self.queue_part(part, currentws) #queue part for next robot
            self.ws_part_holder[currentws][partID] = part#place in current ws part holder

        elif (part.get_transfer()):
            #print("queuing transfer part")
            self.queue_part(part, currentws) #queue part for next robot
            self.ws_part_holder[currentws][partID] = part#place in current ws part holder
        else: #if part has reached is next ws
            #print("adding part to station q")
            part.at_ws()
            #print(currentws)
            self.station_qs[currentws].put(part) #insert part in to the WSQ

        return


def main():
    rospy.init_node("map_node")
    print("warming up ><")
    rospy.sleep(10) #wait until robots start up
    print("done warming up")
    active_map = Map()
    active_map.load_parts() #load parts on onto station queues

    #each station is on own thread
    #parts are loaded onto each station by thier queue,  stations[station].put(part)
    stations = active_map.get_stations()
    while not rospy.is_shutdown():
        pool = concurrent.futures.ThreadPoolExecutor(max_workers=active_map.get_num_s() + 2)#add one for data recording
        for station in stations: #threads are created here and call Process part function
            pool.submit(active_map.start_station, station)
            time.sleep(0.1)
        pool.submit(active_map.rundisplay)
        #pool.submit(active_map.record_throughputdata)

        pool.shutdown(wait=True)
        rospy.signal_shutdown("parts are finished processing")


if __name__ == "__main__":
    main()