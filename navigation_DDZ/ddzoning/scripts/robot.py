#!/usr/bin/env python
import rospy
from ddzoning.msg import Zone
from ddzoning.msg import Position
from ddzoning.msg import Goal
from ddzoning.msg import Row
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from assets.zone_support import Zone_func
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import concurrent.futures
import copy
import yaml
import math
import random
from csv import writer, reader
import os
import rospkg

class Robot(Zone_func):
    def __init__(self):

        #get parameters
        self.zone_stations = rospy.get_param('starting_stations') #this hold the stations that this robot owns
        self.zone_segments = rospy.get_param('starting_segments') #this hold the segments that this robot owns
        carter_home = rospy.get_param('initial_pose') #current position of the robot
        self.position = [carter_home[0], carter_home[1]]
        self.currentws = rospy.get_param('initial_station')
        #self.avgvel = rospy.get_param('/robot_vel')
        #self.vel = 3.937 #(1.2 meters / sec)
        self.num_robot = rospy.get_param('/num_robots') #total number of robots in the system
        self.range = rospy.get_param('/range') #communication range of the robot
        self.zoneID = int(''.join(filter(lambda i: i.isdigit(), rospy.get_namespace())))
        if self.zoneID == 1:
            self.DDZStarter = True
        else:
            self.DDZStarter = False

        #inheit the zone support class
        self.adj_dist = 300.00
        super().__init__(self.num_robot, self.adj_dist)

        #Publishers 
        self.robotpospub = rospy.Publisher("robot_pos", Position, queue_size = 10) #create publisher for sending new goal   
        self.robotzonepub = rospy.Publisher("robot_zone", Zone, queue_size = 10) #sends robot zone information
        self.ws_pub = rospy.Publisher("/robot_claim", Zone, queue_size=50) #sends info about was robot owns what station
        self.robotpartpub = rospy.Publisher("/robot_part", Goal, queue_size = 100) #send part back to ws
        self.load_pub = rospy.Publisher("robot_load", Position, queue_size = 10) #to send load to neighboring robot
        self.numNi_pub = rospy.Publisher("num_neighbor",Float32, queue_size=10 ) #publish the number of neighbors a robot has(used for Metropolis weights)
        self.load_avg_pub = rospy.Publisher("avg_load", Float32, queue_size=10) #publisher for sending out the calculated avg load(avg consensus) 
        self.DD_exit_pub = rospy.Publisher("ready_exit", Int32, queue_size=10) #tell neighboring robots that it is ready to exit 
        self.robot_pub = rospy.Publisher("newgoal", MoveBaseGoal, queue_size = 10) #goal message to send to set_goal to set a new target 
        self.stop_robot_pub = rospy.Publisher("DD_zoning_active", Bool, queue_size=1) #to send to set_goal to stop/start robot
        self.confrim_newts = rospy.Publisher("confirm_ts", Bool, queue_size=5)
        self.zone_confirm = rospy.Publisher("zone_confirm", Bool, queue_size=5)
        self.zone_cspub = rospy.Publisher("robot_primary_zone", Zone, queue_size=10)

        self.update_zone_pub = {}
        self.update_neigh_load_pub = {}
        self.update_ts_pub = {}
        self.reload_parts_pub = {}
        for robotID in range(1,self.num_robot+1):
            if (robotID != self.zoneID):
                self.update_zone_pub["/carter"+str(robotID)+"/"] = rospy.Publisher("/carter"+str(robotID)+"/adopt_new_zone", Zone, queue_size=10)
                self.update_neigh_load_pub["/carter"+str(robotID)+"/"] = rospy.Publisher("/carter"+str(robotID)+"/get_load", Zone, queue_size=10)
                self.update_ts_pub["/carter"+str(robotID)+"/"] = rospy.Publisher("/carter"+str(robotID)+"/correct_ts", Bool, queue_size=10)
                self.reload_parts_pub["/carter"+str(robotID)+"/"] = rospy.Publisher("/carter"+str(robotID)+"/reload_parts", Bool, queue_size=10)

        #Suscribers
        rospy.Subscriber("part_recieve", Goal, self.update_q)
        rospy.Subscriber("avgc_start", Bool, self.update_avgc_flag) #suscriber to signal to other neighbors that avg consensus has started
        rospy.Subscriber("balence", Bool, self.update_balance_flag) #flag to say that system is out of balence to start zone repair
        rospy.Subscriber("zone_claimed", Int32, self.update_availability) #During DDzoning, this allows another robot to claim this robot as a follower
        rospy.Subscriber("reload_parts", Bool, self.send_reload) #to have another robot call to reload parts
        rospy.Subscriber("get_load", Zone, self.send_load) #During DD zoning, this is used from when a robot needs to calculate load based on a new zone design
        rospy.Subscriber("adopt_new_zone", Zone, self.update_zone) #replace zone that a robot has assigned to it
        rospy.Subscriber("correct_ts", Bool, self.adjust_ts) #for when zones are set and yaml ts needs to be adjusted
        rospy.Subscriber("ready_flag", Bool, self.robot_ready)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.set_pose)
        for roboti in range(1,self.num_robot+1): #other robot pos suscribers, suscribe to all other robot positions 
            ns1 = "/carter" + str(roboti) + "/robot_pos"
            if (ns1 != (rospy.get_namespace() + "robot_pos")):
                rospy.Subscriber(ns1, Position, self.update_neighbors)
        
        #data
        self.neigh_pos = {}
        self.neigh_zone = {} #{"/robot1/": (["WS1","WS9","WS12"],[["a","v","b","u",],["u","l","y","i"]])}
        self.neigh_load = {} #("/robot1/": 65.8)
        self.prior_nload = {} #used to rebuild zones
        self.nimbalence = {}
        self.prior_load = 0
        self.neigh_availibility = {} #place to store neighbors ready flag
        
        self.robot_q = {} #stores robot part queue info (ID: part msg)
        self.position = np.array(self.position) #change position to a numpy array
        self.robot_start_time = rospy.get_rostime()
        self.recorded_load = np.zeros((self.numws,self.numws),dtype=np.float32)
        self.rec_pttimes = {} #keeps track of how long a pt has been recorded 
        for row in range(self.numws):
            for col in range(self.numws):
                self.rec_pttimes[row,col] = []
        self.robot_load = 0
        self.robot_all_zone = []
        self.alarm_time = 15 #time till robot declares out of balence (min)
        self.avgc_interval = 120 #time till average consensus (secs)
        self.bal_tolerance = 15  #balence tolerance (min)
        self.robot_avg = 0 #calculated robot avg load
        self.avgc_flag = False #flag to signal to other neighbors that avg has started
        self.imbalence = False #flag to signal that a zone been out of balence for too long
        self.max_episode = 2
        self.max_it = 10
        self.leader_flag = 0 #flag telling neighbors whetther it ready to be leader or not 1 -> leader, 0 -> not decided yet, -1 -> follower
        self.exit_ready = 1 #flag telling neighbors that it is ready to exit DD zoneing 
        self.DD_done = True #flag telling avg consensus and rolling window that DD zone has started
        self.repeat1 = 0 #for repeat zone updating (only what to update zone once but somtimes msg isnt sent)
        self.repeat2 = 0
        self.seen_neighbors = []
        self.startedDDZ = False

        #parameters for carter robot
        self.robot_pos = None
        self.partID = None
        self.currentpart = None
        self.pickup = False
        self.dropoff = True

        #for recording
        self.total_dist = 0
        self.visited_ws = []
        self.record_zone()
        self.robot_travel_dist = 0

    #publishers
    def pub_pos(self):
        #constantly pub position
        while not rospy.is_shutdown():
            msg = Position()
            msg.x = self.position[0]
            msg.y = self.position[1]
            msg.robot = rospy.get_namespace()
            msg.leader = self.leader_flag
            msg.imbalence  = self.imbalence
            msg.load = self.robot_load
            msg.prior_load = self.prior_load
            msg.qsize = len(self.robot_q) #used for debuging and checking
            self.robotpospub.publish(msg)
            rospy.sleep(0.05)
    
    def pub_zone(self):
        #canstantly pub zone
        while True:
            msg = Zone()
            msg.robot = rospy.get_namespace()
            msg.stations = self.zone_stations
            for seg in self.zone_segments: #Zone segements is a nested message file
                msg.segments.append(Row(seg))
            self.robotzonepub.publish(msg)
            rospy.sleep(0.05)
    
    def pub_ws_assignment(self):
        #send what WS assignemnt data to WSs
        stations = copy.deepcopy(self.zone_stations) #self.zone_stations changes
        for ws in stations:
            msg = Zone()
            msg.robot = rospy.get_namespace()
            msg.stations = [ws]
            self.ws_pub.publish(msg)
            rospy.sleep(0.05)

    def pub_part(self, part_msg):
        #update age
        current_time = rospy.get_rostime()
        part_msg.age += current_time.secs - part_msg.timestamp

        #for test2 no rolling window
        row = self.ws_to_index(part_msg.goal1)
        col = self.ws_to_index(part_msg.goal2)
        self.recorded_load[row, col] -= 1

        self.robotpartpub.publish(part_msg) #send part to WS

    def pub_numNi(self):
        numNi = len(self.neigh_pos)
        msg = Float32()
        msg.data = numNi
        self.numNi_pub.publish(msg)

    def pub_avg_load(self):
        while not rospy.is_shutdown():
            msg = Float32()
            msg.data = self.robot_avg
            self.load_avg_pub.publish(msg)
            rospy.sleep(0.01)

    def pub_avgc_flag(self):
        #publish only to neighbors
        for neigh in self.neigh_pos.keys():
            pub = rospy.Publisher(neigh+"avgc_start", Bool, queue_size=1)
            rospy.sleep(0.01)
            msg = Bool()
            msg.data = self.avgc_flag
            pub.publish(msg)

    def pub_exit(self):
        while not rospy.is_shutdown():
            msg = Int32()
            msg.data = self.exit_ready
            self.DD_exit_pub.publish(msg)
            rospy.sleep(0.01)

    def pub_robot_stop(self):
        msg = Bool()
        msg.data = not self.DD_done
        for _ in range(10):
            self.stop_robot_pub.publish(msg)
            rospy.sleep(0.25)

    def pub_primaryzones(self):
        zonemsg = Zone()
        for segment in self.zone_segments:
            zonemsg.segments.append(Row(segment))
        zonemsg.robot = rospy.get_namespace()
        self.zone_cspub.publish(zonemsg)

    #callbacks
    def update_neighbors(self, data):
        #update neighbor pos, load, and ready status
        npos = np.array((0,0))
        npos[0] = data.x
        npos[1] = data.y
        nname = data.robot
        nlead = data.leader
        nload = data.load
        pnload = data.prior_load
        nbal = data.imbalence

        rpos = self.position
        dist = np.linalg.norm(rpos-npos) #distance is in meters

        if dist <= self.range:
            self.neigh_pos[nname] = npos
            self.neigh_availibility[nname] = nlead
            self.prior_nload[nname] = pnload
            self.neigh_load[nname] = nload
            self.nimbalence[nname] = nbal
            #print(rospy.get_namespace(),": robot,",nname,"in range")
            #if a new neighbor is detected connect to it and reload parts
            #if (nname not in self.seen_neighbors):
                #print("connecting to new neighbor")
                #self.seen_neighbors.append(nname)
                #self.startup(1)
                #self.adjust_ts(False)
                #self.send_reload(True)
        else:
            if nname in self.neigh_pos.keys(): #delete neighbor if not in range
                del self.neigh_pos[nname]
                del self.neigh_availibility[nname]
                del self.neigh_load[nname]
                del self.prior_nload[nname]
                del self.nimbalence[nname]
                #if not self.DD_done: #reset seen neighbors if DDZ is active
                    #del self.seen_neighbors[nname]
                #print(rospy.get_namespace(),": robot,",nname,"no longer in range")
        
        self.pub_numNi() #publish the number of neighbors
    
    def update_q(self, part_msg):
        current_time = rospy.get_rostime()
        part_msg.timestamp = current_time.secs

        '''
        #determine if part going to ts or the next processing WS
        if(part_msg.goal2 != part_msg.endgoal):
            #only go to next ws if it is within a certain small distance from TS
            print("transfer part!")
            ts_end_dist = self.workstation_dist_mtx[self.ws_to_index(part_msg.goal2), self.ws_to_index(part_msg.endgoal)] * 0.3048 #convert to meters
            print("to end_dist", ts_end_dist)
            
            if (ts_end_dist < self.transferable_dist):
                part_msg.goal2 = copy.deepcopy(part_msg.endgoal)
                print("part",part_msg.partID,"now going to", part_msg.goal2)
                if part_msg.endgoal not in self.robot_all_zone:
                    self.robot_all_zone.append(part_msg.endgoal)
        '''

        self.robot_q[copy.deepcopy(part_msg.partID)] = copy.deepcopy(part_msg)
        self.rank_parts()

        #record part in part loads
        row = self.ws_to_index(part_msg.goal1)
        col = self.ws_to_index(part_msg.goal2)
        self.recorded_load[row, col] += 1
        #currentTime = rospy.get_rostime()
        #self.rec_pttimes[row, col].append(currentTime.secs) #recored time that the part has been in queue to pick up

    def update_avgc_flag(self, data):
        #this flag is set when a neighbor ro robot signals that its time to do average consensus
        self.avgc_flag = data.data

    def update_balance_flag(self, data):
        self.imbalence = data.data
    
    def update_availability(self, data):
        self.leader_flag = data.data
        if self.leader_flag == -1:
            self.DD_done = False
            self.pub_robot_stop()

    def send_load(self, zone_msg):
        print(rospy.get_namespace(),"in send load")
        return_r = zone_msg.robot #robot to send new calculated load

        self.prior_load = copy.deepcopy(self.robot_load)
        self.robot_load = self.zone_load(self.robot_all_zone)
        #print(rospy.get_namespace(), "queue size", len(self.robot_q))

        #pub = rospy.Publisher(return_r+"n_load", Float32, queue_size=10)
        #rospy.sleep(0.1) #give time for ros to create publisher
        #fmsg = Float32()
        #fmsg.data = copy.deepcopy(self.robot_load)
        #for _ in range(5): #have to send multiple time in order for wait_for_message to get it
            #pub.publish(fmsg)
            #rospy.sleep(0.01)
        #print("sent msg")

    def update_zone(self, zone_msg):

        #if zone_msg stations are the same as zone_stations then update yaml file
        robot_return  = zone_msg.robot
        
        if zone_msg.load != -1: #-1 indicates that we want to rebuild the zone with the current load
            self.prior_load = copy.deepcopy(self.robot_load)
            self.robot_load = zone_msg.load

        self.zone_stations = zone_msg.stations #set new zones and segments
        self.zone_segments = []
        for seg in zone_msg.segments:
            self.zone_segments.append(seg.data)
        self.startup(1) #rewrite yaml file and find transfer stations  
        self.pub_ws_assignment()
        print(rospy.get_namespace(), "all done updateing zone")
        print(rospy.get_namespace(), "new zone", self.zone_stations)
        rospy.sleep(0.075) #give time for map to recieve new zone

        #send confirm
        print("send confirm new zone")
        msg = Bool()
        msg.data = True
        for _ in range(3):
            self.zone_confirm.publish(msg)

    def robot_ready(self,data):
            #print("carter 1 signal to send goal:", data.data)
            while(not self.DD_done): #to avoid potential issues with a part is getting added when DD is active
                continue
            if (data.data is True):
                #print(rospy.get_namespace(),"len of q:",len(self.robot_q),"\nis currpart none:", self.currentpart is None)
                if (self.robot_q or (self.currentpart is not None)): #check if dictionary is empty
                    
                    print(rospy.get_namespace(),"num in queue",len(self.robot_q))
                    #*******to drop off part**********
                    if (self.currentpart is not None):
                        if(self.dropoff and not self.pickup): #drop off part
                            print(rospy.get_namespace(),"dropping off")
                            rospy.sleep(2.5) #time to drop off a part
                            self.currentpart.reload = False #for reload purposes (in this case we want the parts to continue on)
                            self.pub_part(self.currentpart)
                            self.rank_parts() #rerank parts
                            self.currentpart = None

                    #*******to go to drop off location*******
                    if (self.pickup and not self.dropoff): #go to drop off location
                        print(rospy.get_namespace(),"going to drop off")
                        self.robot_pos = self.currentpart.goal2 #record the drop off position as the current position
                        self.pickup = False #switch sign to get ready to pick-up again
                        self.dropoff = True #signal to pickup part
                        prevpos = copy.deepcopy(self.currentws)
                        self.currentws = copy.deepcopy(self.currentpart.goal2)
                        rospy.sleep(2.5) #time to pick up a part

                    #*******to go to pick up location********
                    else: #go to pick-up location
                        print(rospy.get_namespace(),"going to pick up")
                        if not self.robot_q: #check to make sure a part is available
                            return
                        self.partID, self.currentpart = self.robot_q.popitem() # get next part and save as current part
                        self.robot_pos = self.currentpart.goal1
                        self.dropoff = False
                        self.pickup = True # switch sign to get ready to drop-off again
                        prevpos = copy.deepcopy(self.currentws)
                        self.currentws = copy.deepcopy(self.currentpart.goal1)
                        print("going to pick up part:", self.partID)
                    
                    #****create a position and send message****
                    print(rospy.get_namespace(),"going to WS:", self.robot_pos)
                    #record the next position in file
                    path, dist = self.shortest_dist(self.ws_crit_point(prevpos), self.ws_crit_point(self.currentws), self.adjacency_Mtx)
                    print(rospy.get_namespace(), "dist from",self.ws_crit_point(prevpos), "to", self.ws_crit_point(self.currentws),":", dist)
                    self.robot_travel_dist += dist
                    newPosition = self.WS_position(self.robot_pos) #create position
                    goal_msg = MoveBaseGoal() #create a goal msg
                    goal_msg.target_pose.header.frame_id = rospy.get_param("frame_id", "map") #add parameters
                    goal_msg.target_pose.header.stamp = rospy.get_rostime()
                    pose = newPosition #from the GoalReader class generate goal pose is a list of floats

                    # couldn't sample a pose which is not close to obstacles. Rare but might happen in dense maps.
                    if pose is None:
                        rospy.logerr("Could not generate next goal for Carter 1. Returning. Possible reasons for this error could be:")
                        rospy.logerr(
                            "1. Stack is empty and waiting to be updated"
                        )
                        rospy.logerr(
                            "2. Goal is too close to a obstacle."
                        )
                        #carter1newgoal = None
                        self.robot_pub.publish(None)
                        return

                    #rospy.loginfo("Generated goal pose: {0}".format(pose))
                    goal_msg.target_pose.pose.position.x = pose[0]#set the pose in the goal msg
                    goal_msg.target_pose.pose.position.y = pose[1]
                    goal_msg.target_pose.pose.orientation.x = pose[2]
                    goal_msg.target_pose.pose.orientation.y = pose[3]
                    goal_msg.target_pose.pose.orientation.z = pose[4]
                    goal_msg.target_pose.pose.orientation.w = pose[5]
                    #rospy.loginfo("Publishing goal c1")
                    self.robot_pub.publish(goal_msg)
                    rospy.sleep(1)

    def set_pose(self, data):
        self.position = [data.pose.pose.position.x, data.pose.pose.position.y] #set positions from carter robot (converted to feet)
        self.position[0] = self.position[0] * 3.2808
        self.position[1] = self.position[1] * 3.2808
    
    #class functions
    def get_neigh_zone(self):
        try:
            self.neigh_zone.clear()
            for neighbor in self.neigh_pos.keys():
                zone_msg = rospy.wait_for_message(neighbor+"robot_zone", Zone)

                #decode Row part of zone_msg segaments
                neigh_seg = []
                for seg in zone_msg.segments:
                    neigh_seg.append(seg.data)

                self.neigh_zone[zone_msg.robot] = (zone_msg.stations, neigh_seg)

        except RuntimeError:
            print("neigh_pos changed during iteration")

    def get_lead_availability(self):
        #returns if all neighbors are availble or not(robot is ready to lead)
        #listen for a sec
        neigh_avail = []
        for it in range(10):
            neigh_avail += list(self.neigh_availibility.values())
            rospy.sleep(0.1)
        if (all(neigh_avail) == 0):
            return True
        else:
            return False

    def send_neigh_availability(self, flag, neigh):
        #this function will switch the leader flag of a neighbor
        print(rospy.get_namespace(),"sending updated availibility to", list(self.neigh_pos.keys()), "to", flag)
        pub = rospy.Publisher(neigh+"zone_claimed", Int32, queue_size=5)
        rospy.sleep(0.1)
        msg = Int32()
        msg.data = copy.deepcopy(flag)
        print("publishing to", neigh+"zone_claimed")
        for _ in range(5): #send it 5 times
            pub.publish(msg)
            rospy.sleep(0.1)
        rospy.sleep(3)

    def get_neigh_exit(self):
        exitready_list = []
        for neigh in self.neigh_pos.keys():
            msg = rospy.wait_for_message(neigh+'ready_exit', Int32)
            exitready_list.append(msg.data)
        
        if (-1 in exitready_list):
            return -1
        elif (0 in exitready_list):
            return 0
        else:
            return 1

    def get_imabalence(self):
        imbal = list(self.nimbalence.values())
        if (any(imbal) is True) or self.imbalence:
            return True
        else:
            return False

    def adjust_ts(self, data):
        #this function reads a neighbors yaml file and the shared transfer stations so that they are the same
        #print(rospy.get_namespace(), "in adjust ts()")

        try: #firgure if another robot is calling the function 
            sendconfirm = data.data
            #self.repeat2 += 1
        except AttributeError:
            sendconfirm = data
            #self.repeat2 = 1
        
        #if self.repeat2 == 1:
        for neigh in self.neigh_pos.keys():
            neigh_ID = int(''.join(filter(lambda i: i.isdigit(), neigh)))
            with open('robot' + str(neigh_ID) +'.yaml','r') as file: #read yaml file containing zone information
                filemap = yaml.safe_load(file)
                try:
                    ts_map = copy.deepcopy(filemap["transfer_stations"])   
                    neigh_ts = ts_map[self.zoneID] #shared ts between robot zone and neighbor zone from neighbor
                except:
                    print("error in adjust ts, cant find transfer_stations")
                    print("returning orginal transfer station config")
                    break
                #print("neigh ts:", neigh_ts)
                #print("self ts", self.zonets)

                #remove transfer stations that are not shared
                neigh_ID = int(''.join(filter(lambda i: i.isdigit(), neigh)))
                try:
                    zone_copy = copy.deepcopy(self.zonets[neigh_ID])
                except:
                    print("error in finding self.zonets[neigh_ID]")
                    print("ID error prolly")
                    print("self.zonets", self.zonets)
                    print("neighID", neigh_ID, neigh)
                    print("continuing without updating yaml")
                    return

                for ts in zone_copy:
                    if ts not in neigh_ts:
                        self.zonets[neigh_ID].remove(ts)
                        print("removing ts:", ts)

                #to avoid zones being completely disconnected
                #case for when another zone has found a transfer stations between the two zones
                if self.zonets[neigh_ID] == [] and len(neigh_ts) > 1:
                    self.zonets[neigh_ID]= copy.deepcopy(neigh_ts) #append one ts to zone

        #update robot all zone
        self.robot_all_zone = copy.deepcopy(self.zone_stations)
        for zs in self.zonets.values():
            for s in zs:
                if s not in self.zone_stations:
                    self.robot_all_zone.append(copy.deepcopy(s))

        #update the yaml file
        #print("rewriting file")
        neighbor_station = {robot:zone[0] for robot, zone in self.neigh_zone.items()}
        zonedata = {"ID":self.zoneID, "stations":self.zone_stations, "neighbor":neighbor_station, "transfer_stations": self.zonets}
        with open('robot'+str(self.zoneID)+'.yaml','w') as file:
            yaml.dump(zonedata, file)
        #print(rospy.get_namespace(), "all done in adjust_ts()")
        
        
        print("send confirm ts:", sendconfirm)
        #if sendconfirm:
        if sendconfirm:
            msg = Bool()
            msg.data = True
            for _ in range(7):
                self.confrim_newts.publish(msg)
                rospy.sleep(0.075)

        #if self.repeat2 == 3:
            #self.repeat2 = 0

    def startup(self, iterations):
        #performed after self.neigh_pos and self.neighzone are set
        #looks at neighbor ws and cs and updates ts and yaml

        for iteration in range(0, iterations): #takes a few iterations to collect all neighbor positions 
            #self.pub_primaryzones()
            #print("startup iteration",iteration)
            self.get_neigh_zone()
            #collect known segemts
            zonecs = [[] for i in range(self.num_robot)]
            zonews = [[] for i in range(self.num_robot)]

            for neighbor, zone in self.neigh_zone.items():
                zone_n = int(''.join(filter(lambda i: i.isdigit(), neighbor))) #to extract the number in /robot1/
                zonews[zone_n-1] = copy.deepcopy(zone[0])
                zonecs[zone_n-1] = copy.deepcopy(zone[1])
            
            zonews[self.zoneID-1] = copy.deepcopy(self.zone_stations)
            zonecs[self.zoneID-1] = copy.deepcopy(self.zone_segments)

            #create all adj matrix for itself and neighboring zones
            all_adj_matrix = [copy.deepcopy(self.adjacency_Mtx) for _ in range(self.num_robot)]
            all_adj_matrix = self.update_adj_matrixs(all_adj_matrix, zonecs, self.adjacency_Mtx)

            #find transfer stations between each neighbor
            self.zonets = {}
            self.robot_all_zone = copy.deepcopy(self.zone_stations)
            neighbors = copy.deepcopy(list(self.neigh_zone.keys()))
            for neighbor in neighbors:
                n_load = self.neigh_load[neighbor]
                neigh_ID = int(''.join(filter(lambda i: i.isdigit(), neighbor)))
                ts, zonecs_c, all_adj_matrix_c = self.find_transfer_stations(zonews, copy.deepcopy(zonecs), all_adj_matrix, self.zoneID-1, neigh_ID-1, [self.robot_load, n_load])
                self.zonets[neigh_ID] = ts
                #include new ts in all zone

                for s in ts:
                    if s not in self.zone_stations:
                        self.robot_all_zone.append(copy.deepcopy(s))
                #self.zone_segments = copy.deepcopy(zonecs[self.zoneID-1])

            #populate YAMl file to send to work stations
            #prepare neighbor data to only write neighbor WS
            neighbor_station = {robot:zone[0] for robot, zone in self.neigh_zone.items()}
            data = {"ID":self.zoneID, "stations":self.zone_stations, "neighbor":neighbor_station, "transfer_stations": self.zonets}
            with open('robot'+str(self.zoneID)+'.yaml','w') as file:
                yaml.dump(data, file)
            
            #print("robot",self.zoneID)
            #print(newmap["ID"])
        
            rospy.sleep(0.2)

    def run(self):
        #this continously runs moving the robot the next pickup and drop off in queue
        if self.robot_q: #check if robotq is empty
            partID, currentpart = self.robot_q.popitem() #next part in list
            pickup = currentpart.goal1
            dropoff = currentpart.goal2
            print("*****",rospy.get_namespace(),"picking up part:",partID,"at:",pickup,"**********")
            self.goto(pickup)
            print("*****",rospy.get_namespace(),"dropping off part:",partID,"at:",dropoff,"**********")
            self.goto(dropoff)
            currentpart.reload = False #for reload purposes (in this case we want the parts to continue on)
            self.pub_part(currentpart)
            self.rank_parts() #rerank parts
        rospy.sleep(0.1)

    def rank_parts(self):
        #re-score each part in a dictionary and sort
        partsList = list(self.robot_q.values()) #stores part objects
        self.robot_q.clear()

        sorted_parts = {}
        for part in partsList: #re-score all parts
            self.score(part)
            sorted_parts[part.partID] = part.score

        robot_sorted_list = sorted(sorted_parts.items(), key=lambda x:x[1]) #sort list by item (low to high)
        sorted_parts = dict(robot_sorted_list)

        #insert back into robot queue going from (partID, score) to (partID, partobj)
        for partID in sorted_parts.keys():
            for part in partsList:
                if partID == part.partID:
                    self.robot_q[partID] = part

        #printing 
        """
        print(rospy.get_namespace())
        for part in self.robot_q.values():
            print("score:", part.score,"part ID:",part.partID)
        """

    def score(self, part_msg):
        #update age
        current_time = rospy.get_rostime()
        part_msg.age += current_time.secs - part_msg.timestamp

        #if self.currentws is None:
            #self.currentws = part_msg.goal1 #set the first part pickup as the first goal

        Rpos = self.currentws
        Pp = part_msg.goal1
        Pd = part_msg.goal2

        jobdist = self.get_shortest_dist(Rpos, Pp, self.workstation_dist_mtx) + self.get_shortest_dist(Pp, Pd, self.workstation_dist_mtx)
        score = self.Ca*part_msg.age - self.Cd*((jobdist/self.V)*60) #the bigger the score the higher priority
        part_msg.score = score
    
    def get_shortest_dist(self,i,j,ws_distmtx):
        #i and j are in form "WS1"
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()

        i_index = ws_list.index(i)
        j_index = ws_list.index(j)
        return ws_distmtx[i_index, j_index]
    
    def goto(self, goalws):
        #function moves to position of robot until it reaches the goal position
        start = self.ws_crit_point(self.currentws)
        finish = self.ws_crit_point(goalws)
        path, dist = self.shortest_dist(start, finish, self.adjacency_Mtx)
        if path is None: #robot is already at goal location
            return
        for point in path:
            if point == path[0]:
                continue #skip the fisrt point
            #print(rospy.get_namespace(),"going to point:",point)
            nextxy = self.crit_point_pos(point)
            
            distdir = nextxy - self.position #(0,7)
            dir = distdir/(np.linalg.norm(distdir)) #get the unit vector(0,-1)

            timediff = 0.1
            tolerance = 0.5
            target_dist = 1
            while target_dist > tolerance:
                self.position = self.position + dir*(self.V*timediff) #move to the next target
                target_dist = np.linalg.norm(self.position-nextxy)
                rospy.sleep(0.1)
        self.currentws = goalws

    def monitor(self):
        #this functions monitors the current load of the robot
        #it also publishs out of tolerance if load is too far from the average
        #print("monitoring load")
        s1 = rospy.get_rostime() #timer keeping track of when to start DDZ
        s2 = rospy.get_rostime() #timer to keep track how how long system has reamined balenced or im balenced
        
        balence_status = True #flag to signal if system is imbalence

        status = True #flag to indicate whether robot is out of tolerance
        previous_status = True

        while not rospy.is_shutdown():
            curr_time = rospy.get_rostime() #get current time

            #get the current status
            #get all zone which includes transfer stations 
            self.prior_load = copy.deepcopy(self.robot_load)            
            self.robot_load = self.zone_load(self.robot_all_zone)
            #s2 = rospy.get_rostime() #restart timer
            print(rospy.get_namespace(),"load:",self.robot_load)
            self.record_error_history()
            
            #determine if system is out of tolerance
            previous_status = copy.deepcopy(status)
            status = ((self.robot_load < (self.robot_avg + self.bal_tolerance)) and (self.robot_load > (self.robot_avg - self.bal_tolerance)))
            #true if in tol
            #false if out of tol

            #get the current time
            timer = curr_time.secs - s2.secs

            #if the current load is out of tolerence start the timer or a imbalence has been detected
            #do not reset timer if aveage consensus is active
            detect_imbalence = self.get_imabalence()
            if (not balence_status or detect_imbalence):
                if (((curr_time.secs - s1.secs) > (self.alarm_time*60)) or detect_imbalence):
                    print(rospy.get_namespace(),"imbalence detected")

                    #do not go into DD zoning unless this robot is connected to neighbors
                    while(len(self.neigh_pos) == 0):
                        self.DD_done = True
                        #self.pub_robot_stop()
                    self.DD_done = False
                    self.pub_robot_stop() 
                    
                    #designate the first one to start DDZ as leader
                    #if not detect_imbalence:
                        #self.leader_flag = 1
                        #self.started_DDZ = True
                    #else:
                        #self.started_DDZ = False
                        #self.leader_flag = 0
                    
                    if self.DDZStarter: #designate the robot 1 as leader
                        self.leader_flag = 1
                    else:
                        self.leader_flag = 0

                    self.imbalence = True #flag telling neighbors that a imabalence has been detected
                    self.DD_done = False #once DD_done turns false then robots stops moving
                    
                    #self.pub_robot_stop()
                    
                    #reset recorded times
                    #delete history
                    #print("deleting history")
                    #print("deleting times")
                    #self.rec_pttimes = {} #keeps track of how long a pt has been recorded 
                    #for row in range(self.numws):
                        #for col in range(self.numws):
                            #self.rec_pttimes[row,col] = []
                    
                    #print("reseting recorded load")
                    #self.recorded_load = np.zeros((self.numws,self.numws), dtype=np.float32)
                    #rq = copy.deepcopy(self.robot_q)
                    #for part in rq.values():
                        #print(rospy.get_namespace(),"adding part:",part.partID)
                        #row = self.ws_to_index(part.goal1)
                        #col = self.ws_to_index(part.goal2)
                        #self.recorded_load[row, col] += 1
                    #rospy.sleep(15) #give time for robots to enter DDZ
                    print("going into DD zoning")
                    self.DD_zoning() #start zone repair
                    self.started_DDZ = False
                    s1 = rospy.get_rostime() #reset DDZ timer
            
            if(status and previous_status): #if system has remain in balence
                #is system has remained balenced for 5min reset DDZ timer and declare system balenced
                if timer/60 > 4.5:
                    balence_status = True
                    s1 = rospy.get_rostime()
            
            elif(status != previous_status): #balence change detected
                #reset timer
                s2 = rospy.get_rostime()

            elif(not status and not previous_status):
                if timer/60 > 4.5:
                    balence_status = False

            print(rospy.get_namespace(), "balsts, status, prevssts", balence_status, status, previous_status)

            rospy.sleep(10)
    
    def zone_load(self, ws_in_zone):
        #equations 11-14
        #finding Lpz
        #to find Lpz you need to find fpzij with is the load specifically in the zone

        self.fpzijD = {}

        all_fpzij = self.finding_zone_all_fpzij(ws_in_zone)

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

        #print(rospy.get_namespace())
        #print(ws_in_zone)
        #print(rospy.get_namespace(),"sum_DA_DB",sum_DA_DB)
        #print(rospy.get_namespace(),"all_fpzij",all_fpzij)

        Lpz = (sum_DA_DB/self.V) + (all_fpzij*(self.tu + self.tl))
        self.fpzijD.clear()

        #if Lpz < 1:
            #print("rut ro")

        return Lpz

    def finding_gpzij(self, i, j, ws_in_zone, all_fpzij):
        #finding fpzki and fpzjk
        fpzki = 0
        fpzjk = 0
        for k in ws_in_zone:
            if(k!=i):
                fpzki += self.fpzijD[k + ',' + i]
            if(k!=j):
                fpzjk += self.fpzijD[j + ',' + k]

        if all_fpzij == 0:
            return 0
        else:
            return (fpzki*fpzjk)/all_fpzij

    def finding_fpzij(self,i,j):
        #If i is a tip workstation, then fpzij includes
        #not only fij (which is the flow originating from i and going to j),
        # - but also the flow that originates from workstations in other zones,
        #   passes i, and arrives at j
        # - and the flow that originates from workstations in other zones, passes i, passes j, and arrives at workstations
        #   in other zones.
        # i and j are in the form 'ws1'

        i_index = self.ws_to_index(i)
        j_index = self.ws_to_index(j)

        fpzij = self.recorded_load[i_index][j_index]

        return fpzij

    def finding_zone_all_fpzij(self,ws_in_zone):
        all_fpzij = 0
        for i in ws_in_zone:
            for j in ws_in_zone:
        #for i in len(self.ws_loads[robot]): # i is index
            #for j in len(self.ws_loads[robot][i]): # j is index
                if(i != j):
                    fpzij = self.finding_fpzij(i, j)
                    all_fpzij += fpzij
                    self.fpzijD[i + ',' + j] = fpzij
        return all_fpzij

    def rolling_window(self):
        while not rospy.is_shutdown():
            while not self.DD_done: #do not delete any parts off the record while DD zoning is active 
                rospy.sleep(5)
                
            currentTime = rospy.get_rostime()
            for pt, times in self.rec_pttimes.items(): #retrieves key value pair
                copytimes = copy.deepcopy(times)
                for time in copytimes:
                    if ((currentTime.secs - time) > self.window*60):
                        times.remove(time)
                        self.recorded_load[pt[0],pt[1]] -= 1
                        print(rospy.get_namespace())
                        print("removed 1 from:", pt[0],pt[1])
            #print(rospy.get_namespace())
            #print(self.recorded_load)
            rospy.sleep(10)

    def WS_position(self,ws):
        
        index = self.ws_to_index(ws)
        x = self.workstation_loc['x']
        y = self.workstation_loc['y']

        x_pos = float(x[index])/3.281
        y_pos = float(y[index])/3.281

        #print("goal (x, y): (",x_pos,",",y_pos,")")
        
        #convert to meters
        return [x_pos, y_pos ,0 ,0 ,1 ,0]

    def valid_zone(self,zonews1,zonesws2):
        count1 = 0
        count2 = 0
        for x in zonews1:
            count1 += len(x)
        for y in zonesws2:
            count2 += len(y)

        if count1 == count2:
            return True
        else:
            return False
    
    #functions for finding average consesus 
    def avg_consenus(self):
        while not rospy.is_shutdown():
            #spin until timer hits or flag is raised
            start_time = rospy.get_rostime()
            timer = 0
            while ((timer < self.avgc_interval) and (not self.avgc_flag)): #wait until self.avgc_flag is set to true and enough to time has past 
                #timer
                currtime = rospy.get_rostime()
                timer = currtime.secs - start_time.secs 
                if not self.DD_done: #dont run average consensus while DD zoning is active
                    start_time = rospy.get_rostime()
            if not self.DD_done:
                continue

            print("******starting avg consensus********")
            priori_load = copy.deepcopy(self.robot_avg)
            self.robot_load = self.zone_load(self.robot_all_zone)
            self.robot_avg = copy.deepcopy(self.robot_load) #set initial measurement
            self.robot_avg += 0.1  #small constant added for when load is zero
            self.avgc_flag = True
            self.pub_avgc_flag() #publish flag to other neighbors to start avg consensus

            #self.vel = 1 #slow down velocity of each robot to keep current neighbors
            print(rospy.get_namespace(),"before calculated average:", self.robot_avg)
            max_it = 100

            #neighborList = self.findneighbors(self.num_zone, self.robotxy, 200)
            for it in range(max_it):
                #calculate mean square error
                #get current neighbors
                neighbors = list(self.neigh_pos.keys()) #this is constantly updated ["/robot1/","/robot2/"]

                #metropolis weights
                weights = self.metropolisweights(neighbors) #[0.6, 0.1, 0.3] last is current robot weight
                #calculate sum of neighbors weights
                Niweight=0
                #calucate neighbor part of measurement
                if len(neighbors) > 0:
                    for ni_index in range(len(neighbors)):
                        ni = copy.deepcopy(neighbors[ni_index])
                        avgmasg = rospy.wait_for_message(ni+"avg_load",Float32) #get nighbor calc avg
                        ni_avg = avgmasg.data
                        Niweight += weights[ni_index]*ni_avg
                
                #calculate next measurement
                self.robot_avg = self.robot_avg*weights[len(weights)-1] + Niweight

            print(rospy.get_namespace(),"calculated average:", self.robot_avg)
            #if DD zoning is active do not speed up
            #if self.DD_done:
                #self.vel = 8.5
            #else:
                #self.vel = 0
            self.avgc_flag = False #set flag so that it cant loop again

            #case for when the robot had no neighbors (rarely happens)
            if self.robot_load == self.robot_avg:
                self.robot_avg = copy.deepcopy(priori_load)

    def metropolisweights(self, NiList):
        #nodeNeighbor=NiList[nodeIndex] NiList is a list of neighbors
        nodedegree=len(NiList)
        weights=[0]*nodedegree
        weightsum=0
        
        if NiList is not None:
            for niindex in range(len(NiList)):
                ni = copy.deepcopy(NiList[niindex])
                #get neighbor degree
                nummsg = rospy.wait_for_message(ni+"num_neighbor",Float32) #listen to neighpub and get the number of neighbors
                Nidegree = nummsg.data #number of neighbors a neighbor has

                weights[niindex]=1/(1+max(nodedegree, Nidegree))
                weightsum+=weights[niindex]
        
            weights.append(1-weightsum)
            return weights
        else:
            weights.append(1)
            return weights

    #decentrialized dynamic zoning
    def DD_zoning(self):
        print(rospy.get_namespace(), "starting DD zoning")
        self.DD_done = False #once DD_done is false the robot stop moveing
        #self.pub_robot_stop()
        #self.startup(1) #to update yaml and all zone
        #if self.started_DDZ: #if this robot started DDZ then this robot will be the leader
            #self.leader_flag = 1
        #rospy.sleep(10) #wait for things to 

        #self.send_reload(True) #reload parts in the queue
        #rospy.sleep(10) #sort of used to allow disconnected robots to connect to the system
        print(rospy.get_namespace(),"caculating load")
        self.prior_load = copy.deepcopy(self.robot_load)
        self.robot_load = self.zone_load(self.robot_all_zone) #get a nore accurte zone load
        print(rospy.get_namespace(),"load going in:", self.robot_load)
        
        error = []
        self.exit_ready = -1
        #SA parameters
        itcount = 0
        Tf = 0.3
        Ti = 4.5
        k = self.max_it

        rospy.sleep(self.zoneID*2) #sleep relative to ID amount

        '''
        if self.leader_flag == 1:
            if self.get_lead_availability(): #if all neighbors are ready 
                for neigh in self.neigh_pos.keys():
                    self.send_neigh_availability(-1, neigh) #-1 signifies follower claiming neighbors
                rospy.sleep(3) #give time to send message
            else:
                self.leader_flag = -1 #if not all neighbors are ready then this robot is not the leader
        '''
        if(self.leader_flag == 1):
            rospy.sleep(11) #wait for neighbors to catch up

        for ep in range(1, self.max_episode+1):
            self.current_neighbors = copy.deepcopy(self.neigh_pos)
            
            print(rospy.get_namespace(),"epsisode:",ep)

            if not self.neigh_pos:
                print(rospy.get_namespace(),"got no neighbors")
                self.DD_done = True
                self.pub_robot_stop()
                while(not self.neigh_pos):
                    self.DD_done = True

            self.DD_done = False
            self.pub_robot_stop()
                    #self.vel = 8.5
                #self.vel = 0
            print(rospy.get_namespace(),"got neighbors")
            
            #stay in loop if not all neighbors are ready or itself is not ready
            while(self.leader_flag != 1):
                #rospy.sleep(random.uniform(0,1)*10)
                rospy.sleep(0.1)
            '''
            back_waiting = False
            if 1 in self.neigh_availibility.values(): #case for if there are two leaders comming out of waiting
                for neighbor in self.neigh_availibility.keys():
                    if 1 == self.neigh_availibility[neighbor]:
                        zone_n = int(''.join(filter(lambda i: i.isdigit(), neighbor)))
                        if self.zoneID > zone_n:
                            self.leader_flag = -1 #not the leader
                            back_waiting = True
            if back_waiting:
                continue
            '''
            for neigh in self.neigh_pos.keys():
                self.send_neigh_availability(-1, neigh) #-1 signifies follower

            #a robot gets here by having each neighbor be ready and itself be ready
            #send a non-ready status to all neighbors to claim them and keep them in the while loop above
            print(rospy.get_namespace(),"is leader") #signal that this robot is ready to lead 

            #robot neighbors
            print("robot neighbors", list(self.current_neighbors))
            
            #collect known segemts
            self.get_neigh_zone() #to update self.neigh_zone
            zonecs = [[] for i in range(self.num_robot)]
            zonews = [[] for i in range(self.num_robot)]

            for neighbor in self.current_neighbors.keys():
                zone = self.neigh_zone[neighbor]
                zone_n = int(''.join(filter(lambda i: i.isdigit(), neighbor))) #to extract the number in /robot1/
                zonews[zone_n-1] = copy.deepcopy(zone[0])
                zonecs[zone_n-1] = copy.deepcopy(zone[1])
            
            zonews[self.zoneID-1] = copy.deepcopy(self.zone_stations)
            zonecs[self.zoneID-1] = copy.deepcopy(self.zone_segments)

            best_ws = copy.deepcopy(zonews)
            best_cs = copy.deepcopy(zonecs)
            best_rload = copy.deepcopy(self.robot_load)
            best_neigh_load = copy.deepcopy(self.neigh_load)
            best_build_load = copy.deepcopy(self.prior_nload)
            best_build_load[rospy.get_namespace()] = copy.deepcopy(self.prior_load)
            best_error = copy.deepcopy(self.calc_stddev())
            starterror = copy.deepcopy(best_error)#for debuging

            for it in range(1,self.max_it+1):
                print(rospy.get_namespace(),"iteration:",it)
                itcount += 1
                #get current neighbors zone design and create adj matrixs
                #//////
                #print("updated neighbor zone")

                #collect known segemts
                self.get_neigh_zone() #to update self.neigh_zone
                zonecs = [[] for i in range(self.num_robot)]
                zonews = [[] for i in range(self.num_robot)]

                for neighbor in self.current_neighbors.keys():
                    zone = self.neigh_zone[neighbor]
                    zone_n = int(''.join(filter(lambda i: i.isdigit(), neighbor))) #to extract the number in /robot1/
                    zonews[zone_n-1] = copy.deepcopy(zone[0])
                    zonecs[zone_n-1] = copy.deepcopy(zone[1])
                
                zonews[self.zoneID-1] = copy.deepcopy(self.zone_stations)
                zonecs[self.zoneID-1] = copy.deepcopy(self.zone_segments)

                #create all adj matrix for itself and neighboring zones
                all_adj_matrix = [copy.deepcopy(self.adjacency_Mtx) for _ in range(self.num_robot)]
                all_adj_matrix = self.update_adj_matrixs(all_adj_matrix, zonecs, self.adjacency_Mtx)
                #//////////

                #select a random neighbor
                robotj = random.choice(list(self.current_neighbors.keys())) #name "/robot1/"
                print("selected neighbor:", robotj)
                jzone = self.neigh_zone[robotj] #jzone (stations, segements)
                j_stations = jzone[0]
                j_segments = jzone[1]

                #get tip ws from both zones
                i_tip = self.finding_tip_ws(self.zone_stations, self.zone_segments)
                j_tip = self.finding_tip_ws(j_stations, j_segments)

                #**********figure out if i is giving or taking and exchange**********
                #print("start to exchnge ws")
                #number IDs of zones
                robotj_index = int(''.join(filter(lambda i: i.isdigit(), robotj)))-1
                roboti_index = self.zoneID-1

                #get the current load from each robot
                #print("all zone", self.robot_all_zone)
                i_load = copy.deepcopy(self.robot_load)
                j_load = copy.deepcopy(self.neigh_load[robotj])
                #j_load = self.get_neigh_load([],robotj)
                old_prior_load = copy.deepcopy(self.prior_nload) #for recording purposes
                old_prior_load[rospy.get_namespace()] = self.prior_load

                curr_error = self.calc_stddev() #get the error with the current zone design

                if i_load >= j_load:
                    #roboti i is giving
                    print("i_load >= j_load")

                    #select a random WS to give to robot j that is adjacent
                    At = self.find_adjtip(i_tip, j_tip) #find tip ws in i and j that are adjacent
                    if At == {}:
                        print("unable to exchange a tip ws")
                        continue

                    #exchange random tip ws to j robot
                    zonews_c, zonecs_c, alladj_c = self.exchange_ws(At, robotj_index, zonews, zonecs, all_adj_matrix)

                elif i_load < j_load:
                    #robot i taking
                    print("i_load < j_load")

                    #select a random WS to give to robot i that is adjacent
                    At = self.find_adjtip(j_tip,i_tip) #find tip ws in i and j that are adjacent
                    if At == {}:
                        print("unable to exchange a tip ws")
                        continue

                    #exchange random tip ws to i robot
                    zonews_c, zonecs_c, alladj_c = self.exchange_ws(At, roboti_index, zonews, zonecs, all_adj_matrix)

                #check to see where ws got yonked
                if not self.valid_zone(zonews_c, zonews):
                    print("invalid zone design")
                    print("continuing")
                    continue

                if zonews_c == zonews:
                    print("couldn't exchange a tipws")
                    print("continuing")
                    continue

                #reload parts in each zone
                #print("load parts")
                self.zone_stations = copy.deepcopy(zonews_c[roboti_index])
                self.zone_segments = copy.deepcopy(zonecs_c[roboti_index])
                self.pub_ws_assignment()
                rospy.sleep(0.25)
                #tell neighbors to update zone order goes like this [n1,n2,n3] -> n1,n2,n3,n1,n2,n1
                neighlist = list(self.current_neighbors.keys())
                starti = 0
                for _ in range(len(neighlist)): 
                    for neighi in range(len(neighlist)-starti):
                        neigh = neighlist[neighi]
                        self.new_zone(neigh, zonews_c, zonecs_c, -1) #tell neighbor to update yaml file and to run startup() 
                        starti += 1     

                self.startup(1)

                self.adjust_neigh_ts()

                self.send_reload(True)
                self.all_reload()

                #get the new loads with new zone design
                i_load_c = self.zone_load(self.robot_all_zone)
                self.prior_load = copy.deepcopy(self.robot_load)
                self.robot_load = copy.deepcopy(i_load_c)
                
                self.update_neigh_loads() #also updates neighbor self.robot_load
                j_load_c = copy.deepcopy(self.neigh_load[robotj])
                #check
                if j_load_c != self.neigh_load[robotj]:
                    print("**inconsitancy with jload**")
                    print("somtimes this happens when zones are not updated fast enough before reload")
                    print("could also be a division by zero error when calculating load")
                    print("**returning to orginal**")

                    self.zone_stations = copy.deepcopy(zonews[roboti_index])
                    self.zone_segments = copy.deepcopy(zonecs[roboti_index])
                    self.pub_ws_assignment()
                    rospy.sleep(0.25)
                    self.prior_load = copy.deepcopy(self.robot_load)
                    self.robot_load = copy.deepcopy(old_prior_load[rospy.get_namespace()]) #important that this is set before calling new_zone

                    neighlist = list(self.current_neighbors.keys())
                    starti = 0
                    for _ in range(len(neighlist)): 
                        for neighi in range(len(neighlist) - starti):
                            neigh = neighlist[neighi]
                            j_load = old_prior_load[neigh]
                            self.new_zone(neigh, zonews, zonecs, j_load) #tell neighbor to update yaml file and to run startup() 
                        starti += 1
                    
                    self.startup(1)

                    self.adjust_neigh_ts()

                    self.send_reload(True)
                    self.all_reload()

                    self.prior_load = copy.deepcopy(self.robot_load)
                    self.robot_load = self.zone_load(self.robot_all_zone) #update roboti load
                    self.update_neigh_loads() #use this to update neighbor load

                    zonewsf = copy.deepcopy(zonews)
                    zonecsf = copy.deepcopy(zonecs)

                    continue

                #calculate error
                #print("calculating error")
                new_error = self.calc_stddev() #get the new error with the new zone design 
                print("new error:",new_error)
                print("old error:", curr_error)

                #accept if error is better than old error
                #p = math.log1p(15/it)*.7 #curve to accept random probabilities

                E = curr_error - new_error
                temp = Ti * ((Tf/Ti)**(1/(self.max_it*2)))**itcount
                if temp > Ti:
                    temp = Ti
                elif temp < Tf:
                    temp = Tf

                p = math.exp(E/(k*temp))

                r = random.random()

                #before accepting any sort of zone make sure the switch did not completely disconnect the zone 
                tssum = 0
                for zonets in self.zonets.values():
                    tssum += len(zonets)

                if (new_error <= curr_error or r < p) and tssum > 0:
                    print("keep changes")
                    #keep yaml files the same

                    zonewsf = copy.deepcopy(zonews_c)
                    zonecsf = copy.deepcopy(zonecs_c)

                else:
                    #return to curr config
                    #return robot qs to thier orginal 
                    print("@@@@@@@@return to orginal@@@@@@@@")
                    #print(self.zonets[robotj_index+1])
                    #in order to get back to the orginal zone design both loads need to revert back to what they were before the new zone
                    self.zone_stations = copy.deepcopy(zonews[roboti_index])
                    self.zone_segments = copy.deepcopy(zonecs[roboti_index])
                    self.pub_ws_assignment()
                    rospy.sleep(0.25)
                    self.prior_load = copy.deepcopy(self.robot_load)
                    self.robot_load = copy.deepcopy(old_prior_load[rospy.get_namespace()]) #important that this is set before calling new_zone

                    neighlist = list(self.current_neighbors.keys())
                    starti = 0
                    for _ in range(len(neighlist)): 
                        for neighi in range(len(neighlist) - starti):
                            neigh = neighlist[neighi]
                            j_load = old_prior_load[neigh]
                            self.new_zone(neigh, zonews, zonecs, j_load) #tell neighbor to update yaml file and to run startup() 
                        starti += 1
                    
                    self.startup(1)

                    self.adjust_neigh_ts()

                    self.send_reload(True)
                    self.all_reload()

                    self.prior_load = copy.deepcopy(self.robot_load)
                    self.robot_load = self.zone_load(self.robot_all_zone) #update roboti load
                    self.update_neigh_loads() #use this to update neighbor load

                    zonewsf = copy.deepcopy(zonews)
                    zonecsf = copy.deepcopy(zonecs)
                    

                #///////////////record best/////////////
                if ((new_error < best_error) and (it > 1)):
                    rospy.sleep(0.1)
                    best_ws = copy.deepcopy(zonewsf)
                    best_cs = copy.deepcopy(zonecsf)
                    best_rload = copy.deepcopy(self.robot_load)
                    best_neigh_load = copy.deepcopy(self.neigh_load) #for debugging and checking

                    best_build_load = copy.deepcopy(self.prior_nload) #used to reconstruct zone
                    best_build_load[rospy.get_namespace()] = copy.deepcopy(self.prior_load)

                    #for checking
                    print("*******best*******")
                    print("load")
                    print(rospy.get_namespace(), self.robot_load)
                    print("neigh load:", self.neigh_load)
                    print("zone")
                    print(best_ws)
                    print(best_cs)
                    best_error = copy.deepcopy(new_error)
                    if it == 1:
                        starterror = copy.deepcopy(best_error)
            
            #out of iterations
            #adopt best zone design
            print("out of iterations")
            #update self first
            self.zone_stations = copy.deepcopy(best_ws[self.zoneID-1])
            self.zone_segments = copy.deepcopy(best_cs[self.zoneID-1])
            self.pub_ws_assignment()
            rospy.sleep(0.1)
            self.prior_load = copy.deepcopy(self.robot_load)
            self.robot_load = copy.deepcopy(best_build_load[rospy.get_namespace()])

            #print("1")
            #update neighbor zone
            neighlist = list(self.current_neighbors.keys())
            starti = 0
            for _ in range(len(neighlist)): 
                for neighi in range(len(neighlist) - starti):
                    neigh = neighlist[neighi]
                    self.new_zone(neigh, best_ws, best_cs, best_build_load[neigh]) #tell neighbor to update yaml file and to run startup() 
                starti += 1

            #print("2")
            #update self yaml
            self.startup(1)
            rospy.sleep(0.1)

            #print("3")
            self.adjust_neigh_ts()

            #print("4")
            self.send_reload(True)
            #reload parts from all neighbors
            #print("5")
            self.all_reload()

            #print("6")
            #calculate self zone load
            self.prior_load = copy.deepcopy(self.robot_load) #records 
            self.robot_load = self.zone_load(self.robot_all_zone)
            #tell neighbors to calculate new load
            #print("7")
            self.update_neigh_loads()

            #to test
            print("to check")
            print("starting error:", starterror)
            print("best error:", best_error)
            print("*******best*******")
            print("load")
            print(rospy.get_namespace(), best_rload)
            print(best_neigh_load)
            print("zone")
            print(best_ws)
            print(best_cs)
            print("******compare****")
            print("self load", self.robot_load)
            print(rospy.get_namespace(), "zone\n", self.zone_stations,"\n",self.zone_segments)
            self.get_neigh_zone()
            print("neighbors")
            print(self.neigh_zone)
            print("neigh load")
            print(self.neigh_load)
            print("build loads")
            print(best_build_load)

            #outside of check
            error.append(best_error)
            #release neighbors and get back into top while loop
            self.leader_flag = -1
            #print(rospy.get_namespace(),"releasing neighbors")
            #for neigh in self.neigh_pos.keys():
                #self.send_neigh_availability(0, neigh) #-1 signifies follower claiming neighbors
            #rospy.sleep(1)
            #have robot pick next leader
            neighbors = list(self.current_neighbors.keys())
            neigh = random.choice(neighbors)
            self.send_neigh_availability(1, neigh) #select a robot to be leader
            print("end of iterations")
            rospy.sleep(0.1)
        
        #out of episodes 
        print(rospy.get_namespace(),"out of episodes")
        self.write_error(error)
        self.exit_ready = 0
        all_ready = self.get_neigh_exit()
        while(all_ready != 1):
            all_ready = self.get_neigh_exit()

            if all_ready == -1: #still have a neighbor still iterating
                self.exit_ready = 0
            if all_ready == 0: #a neighbors neighbor is not ready
                self.exit_ready = 1
            
            #bounce availability back towrards a neighbor
            if self.leader_flag == 1:
                self.leader_flag = -1
                neighbors = list(self.neigh_pos.keys())
                neigh = random.choice(neighbors)
                self.send_neigh_availability(1, neigh)
            
            rospy.sleep(0.1)
        
        self.imbalence = False
        print(rospy.get_namespace(),"END OF DD ZONING")
        self.exit_ready = 1 #once all neighbors are ready to exit this robot can exit
        self.DD_done = True
        #self.vel = 8.5
        self.leader_flag = 0
        self.pub_robot_stop()
        rospy.sleep(self.zoneID*2) #to allow time to write to zone record 
        self.record_zone()

    def exchange_ws(self, Atog, robotj, zonews, zonecs, all_adj_matrix):
        #At = {"WS2": ["WS6","WS8","WS9"]} tipws, neighboring tip in other zone
        At = copy.deepcopy(Atog)
        while(True):
            print(At)
            print(zonews)
            if len(At) == 0 or At == {}:
                print("nothing in At")
                break

            tipws = random.choice(list(At.keys())) #choose a random ws to give
            if At[tipws] == []:
                del At[tipws]
                continue
            rand_neigh = random.choice(At[tipws])

            #remove tip from zone
            #do not remove a tipws from a zone that only has less than 3 workstations in it
            breakloop = False
            for zone in zonews:
                if (tipws in zone) and (len(zone) < 3):
                    breakloop = True
            if breakloop:
                print("breaking loop")
                print("cannot remove tipws:", tipws)
                return copy.deepcopy(zonews), copy.deepcopy(zonecs), copy.deepcopy(all_adj_matrix)

            print("removing tipws", tipws)
            copy_solws, copy_solcs = self.remove_tip_ws(copy.deepcopy(zonews), copy.deepcopy(zonecs), copy.deepcopy(all_adj_matrix), tipws)
            
            #update adj matrix
            cadj = self.update_adj_matrixs(copy.deepcopy(all_adj_matrix), copy_solcs, self.adjacency_Mtx)

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
                cadj = self.update_adj_matrixs(copy.deepcopy(all_adj_matrix), copy_solcs, self.adjacency_Mtx)

                #self.valid_zone(self.zonews)
                #self.valid_zone(copy_solws)
                
                print("exchanged tipws:", tipws)
                print("is zone design valid:", self.valid_zone(zonews, copy_solws))
                return copy_solws, copy_solcs, cadj
        
        #self.valid_zone(self.zonews)
        print("is zone design valid:", self.valid_zone(zonews, copy_solws))
        return copy.deepcopy(zonews), copy.deepcopy(zonecs), copy.deepcopy(all_adj_matrix)
                
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
    
    def calc_stddev(self):
        sum_of_diff_squared = (self.robot_load - self.robot_avg)**2 #remeber each robot calculated average indivisually

        for neigh in self.current_neighbors.keys():
            sum_of_diff_squared += (self.neigh_load[neigh] - self.robot_avg)**2
        
        return (sum_of_diff_squared/(len(self.current_neighbors)+1))**(1/2) #+1 for roboti
    
    def all_reload(self):
        #tell neighbors to reload parts
        #print("telling neighbor",neighbor, "to reload parts")
        for neigh in self.current_neighbors.keys():
            msg2 = Bool()
            msg2.data = True
            self.reload_parts_pub[neigh].publish(msg2)
        rospy.sleep(5.25) #wait to send and recieve requeued parts

    def send_reload(self, data):
        #reload parts in queue
        print(rospy.get_namespace(),"sending",len(self.robot_q),"parts")
        #cant send parts from an empty list
        if len(self.robot_q) == 0:
            return
        #copy robot q
        to_sendq = copy.deepcopy(self.robot_q)
        self.robot_q.clear() #robotq will be populated as we send parts
        for _ in range(len(to_sendq)):
            #poppart
            partID, currentpart = to_sendq.popitem()
            #print("preparing to send",partID)

            #remove part from the recored load
            #row = self.ws_to_index(currentpart.goal1)
            #col = self.ws_to_index(currentpart.goal2)
            #times = self.rec_pttimes[row,col] #retrieve times list
            #if len(times) > 0:
                #times.pop() #pop part from times list
            #self.recorded_load[row,col] -= 1
            #print("removed 1 from:", pt[0],pt[1])
        
            #send part to stations to have it requeued
            currentpart.reload = True 
            self.pub_part(currentpart)
            rospy.sleep(0.05)
        rospy.sleep(.25) #wait to recieve requeued parts
        print(rospy.get_namespace())
        #print(self.recorded_load)

    def update_neigh_loads(self):
        #create publisher to send to neighbor zone
        #neighbor zone will recieve mmsg and return the load
        #tell neighbors to update thier self.robot_load
        for neigh in self.current_neighbors.keys():
            print("in get neigh loads")

            msg = Zone()
            msg.robot = rospy.get_namespace() #give neighbor the return robot
            self.update_neigh_load_pub[neigh].publish(msg)

        rospy.sleep(0.5) #wait to recieve requeued parts

    def new_zone(self, robotID, zonews, zonecs, j_load):
        #tell robotj to adpot its new zone
        #this will signal robotID to either change its yaml file or its zone design 
        #case for when the neighbor yaml already has the updated information
        print("telling", robotID,"to adopt new zone")
        msg = Zone()
        msg.robot = rospy.get_namespace()
        intID = int(''.join(filter(lambda i: i.isdigit(), robotID)))
        msg.stations = zonews[intID-1]
        for seg in zonecs[intID-1]: #Zone segements is a nested message file
            msg.segments.append(Row(seg))
        msg.load = j_load

        print("publishing msg")
        self.update_zone_pub[robotID].publish(msg)
        
        print("waiting for zone confirm")
        rospy.wait_for_message(robotID + "zone_confirm",Bool)
        rospy.sleep(0.15)

    def adjust_neigh_ts(self):
        #print("in adjust neigh ts")
        self.adjust_ts(False)
        for neigh in self.current_neighbors.keys():
            msg = Bool()
            msg.data = True
            self.update_ts_pub[neigh].publish(msg)

            print("waiting for:",neigh+"confirm_ts")
            rospy.wait_for_message(neigh+"confirm_ts",Bool)
        rospy.sleep(0.15)

    #recording data
    def write_error(self, new_error):
        myrow = []
        with open(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data','error_report.csv'), 'r') as file:
            alllines = reader(file)
            for row in alllines:
                myrow.append(row)
            file.close()

        myrow[self.zoneID] += new_error
        with open(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data','error_report.csv'), 'w') as file:
            alllines = writer(file)
            for row in myrow:
                alllines.writerow(row)
            file.close()

    def record_robot_pos(self):
        while not rospy.is_shutdown():
            #WS is writen as "WS1" and robot is a number:
            '''
            myrow = [['']]*self.num_robot
            with open(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data','visitedWS.csv'), 'r') as file:
                alllines = reader(file)
                myrow = list(alllines)
                file.close()
            
            myrow[self.zoneID] = [myrow[self.zoneID][0],  str(self.robot_travel_dist)]
            with open(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data','visitedWS.csv'), 'w') as file:
                alllines = writer(file)
                for row in myrow:
                    alllines.writerow(row)
                file.close()
            '''
            distfile = 'carter'+ str(self.zoneID) + 'dist.csv'
            current_time = rospy.get_rostime()
            with open(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data', distfile), 'a') as file:
                writerobj = writer(file)
                writelist = [current_time.secs, self.robot_travel_dist]
                writerobj.writerow(writelist)
                file.close()
            rospy.sleep(30)

    def record_zone(self):
        #record zone design
        row = [['']]*self.num_robot
        with open(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data','zonehistory.csv'), 'r') as file:
            alllines = reader(file)
            row = list(alllines)
            file.close()
        
        time = rospy.get_rostime()
        row[self.zoneID].append(self.zone_stations)
        if self.zoneID == 1:
            row[self.num_robot+1].append(time.secs) 
    
        with open(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data','zonehistory.csv'), 'w') as file:
            writerobj = writer(file)
            for r in row:
                writerobj.writerow(r)
            file.close()

    def record_error_history(self):
        current_time =rospy.get_rostime()
        with open(os.path.join(rospkg.RosPack().get_path('ddzoning'), 'scripts/data', str(self.zoneID) + 'load_history.csv'), 'a') as file:
            writerobj = writer(file)
            writelist = [current_time.secs, self.robot_load, self.robot_avg]
            writerobj.writerow(writelist)
            file.close()

def main():
    rospy.init_node("robot_node")
    name = rospy.get_namespace()
    wait = int(''.join(filter(lambda i: i.isdigit(), name)))
    rospy.sleep(wait) #dont want each node operating at the same time period
    print(name, "starting up")
    st_robot = Robot()

    pool = concurrent.futures.ThreadPoolExecutor(max_workers=7)
    pool.submit(st_robot.pub_zone)
    pool.submit(st_robot.pub_pos)
    #pool.submit(st_robot.rolling_window)
    pool.submit(st_robot.record_robot_pos)
    pool.submit(st_robot.monitor)
    pool.submit(st_robot.pub_avg_load)
    pool.submit(st_robot.pub_exit)
    pool.submit(st_robot.avg_consenus)
    pool.shutdown(wait=False)

    rospy.sleep(1)
    st_robot.startup(20)
    st_robot.adjust_ts(False)
    rospy.sleep(1)
    st_robot.pub_ws_assignment()
    print("after startup")
    
    rospy.spin()

    #while not rospy.is_shutdown():
        #st_robot.pub_pos()
        #print("running")
        #st_robot.run()
        #rospy.sleep(.1)


if __name__ == "__main__":
    main()