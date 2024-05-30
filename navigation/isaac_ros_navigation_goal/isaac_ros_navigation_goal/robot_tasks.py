#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Bool
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from isaac_ros_navigation_goal.msg import stringarr
#from rospy_tutorials.msg import HeaderString
from queue import Queue
import copy
import sys
import numpy as np
import pandas as pd
#from goal_generators import RandomGoalGenerator, GoalReader
from zone.ZoneControl import zone_control
from zone.part import part
from Solution import Solution_module
import concurrent.futures
import time
from rospy.numpy_msg import numpy_msg
from csv import writer


class robot_task(Solution_module):
    def __init__(self, zone_assignment):
        #this program will manage three stack and give the repective robots thier assigned task
        self.carter1Q = Queue(maxsize=0)
        self.carter2Q = Queue(maxsize=0)
        self.carter3Q = Queue(maxsize=0)
        self.carter1part_Q = Queue(maxsize=0)
        self.carter2part_Q = Queue(maxsize=0)
        self.carter3part_Q = Queue(maxsize=0)
        self.carter1_notfirst = False
        self.carter2_notfirst = False
        self.carter3_notfirst = False

        self.carter1pub = rospy.Publisher("/carter1/newgoal", MoveBaseGoal, queue_size = 10) #create publisher
        self.carter2pub = rospy.Publisher("/carter2/newgoal", MoveBaseGoal, queue_size = 10) #create publisher
        self.carter3pub = rospy.Publisher("/carter3/newgoal", MoveBaseGoal, queue_size = 10) #create publisher
        self.rec_loadpub = rospy.Publisher("rec_loads", numpy_msg(Floats), queue_size = 10)
        self.zonepub = rospy.Publisher("zones", stringarr, queue_size = 10)
        self.critpub = rospy.Publisher("critseg", stringarr, queue_size = 10)
        rospy.Subscriber("/carter1/ready_flag",Bool,self.carter1_ready) #create subsriber to see is robot is ready for a new goal
        rospy.Subscriber("/carter2/ready_flag",Bool,self.carter2_ready) #create subsriber to see is robot is ready for a new goal
        rospy.Subscriber("/carter3/ready_flag",Bool,self.carter3_ready) #create subsriber to see is robot is ready for a new goal
        rospy.Subscriber("/send_load",Bool,self.pub_rec_loads) #to send recored drop off loads to monitor module
        rospy.Subscriber("/send_zone",Bool,self.pub_zone) #to send current zone information to monitor module
        self.ws_qs = []
        self.zone = zone_assignment #is a object
        self.num_zones = len(self.zone.phase2_ws())
        self.ws_index = 0
        #to end program
        self.num_fin = 0
        self.num_parts = 0
        self.gen_intialws_q()
        self.prev_ready1 = False

        self.workstation_loc = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/Workstation_Loaction.csv', sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        self.workstation_points = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/Workstation_points.csv', sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        self.workstation_dist_mtx = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/WS_dist_mtx.csv', sep=',')
        self.numws = len(self.workstation_points)

        self.recorded_load = np.zeros((self.numws,self.numws),dtype=np.float32)
        #historic_recload = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/Workstation_Loads.csv', sep=',', header=0, names=['WS1','WS2','WS3','WS4','WS5','WS6','WS7','WS8','WS9','WS10','WS11'], encoding = 'utf-8')
        #self.recorded_load = historic_recload.to_numpy()
        '''
        self.recorded_load = np.array([ #continueing on
        #1,2,3,4,5,6,7,8,9,10,11
        [0,0,0,0,0,0,3,0,0,0,0], #1
        [0,0,5,0,0,0,0,0,0,0,0], #2
        [0,0,0,0,0,0,0,0,0,0,0], #3
        [0,0,0,0,0,0,0,0,0,0,0], #4
        [0,0,0,3,0,0,0,0,0,0,0], #5
        [0,0,0,0,0,0,0,0,0,0,0], #6
        [0,0,1,0,0,0,0,3,0,0,0], #7
        [0,0,0,0,0,0,3,0,0,0,0], #8
        [0,0,0,0,0,0,0,0,0,0,0], #9
        [0,0,0,0,0,0,0,0,0,0,0], #10
        [0,0,0,0,0,0,0,0,0,0,0], #11
        ])
        '''
        self.rec_loadpub = rospy.Publisher("rec_loads", numpy_msg(Floats), queue_size = 15)

        #for solution module
        rospy.Subscriber("/solution_action",String,self.call_solution) #to send recored drop off loads to monitor module
        self.solution_active = False
        self.LS_active = False
        self.ZR_active = False
        self.solution_mod = Solution_module()

        #for data recording
        self.cycletime = []
        self.procunits = 0
        self.startTime = rospy.get_rostime()

    #callback for when a robot has reached a destination
    def carter1_ready(self,data):
        #rospy.loginfo("signal to send goal")
        if self.LS_active: #to avoid potential issues with a part getting added when solution module is active
            while(self.LS_active):
                continue
        if self.ZR_active:
            while(self.ZR_active):
                continue
        if (data.data == True):# and (self.prev_ready1 == False or self.carter1count == 1)):
            if not self.carter1part_Q.empty():
                print("size of carter1_partQ:",self.carter1part_Q.qsize()) 
                print("size of carter1Q:",self.carter1Q.qsize())
                print("is first =",self.carter1_notfirst)
                if (((self.carter1Q.qsize() % 2) == 0) and self.carter1_notfirst):# and self.carter1count > 1: #if next point is start point
                    newpart = self.carter1part_Q.get() #part is being droped off
                    if newpart.get_transfer():
                        print("carter1 queueing transfer part:",newpart.get_ID())
                        self.queue_part(newpart)
                    else:
                        print("carter1 Adding part:",newpart.get_ID(),"to WS queue")
                        self.add_to_WSQ(newpart)

            if not self.carter1Q.empty():
                newPosition = self.carter1Q.get() #get first item in queue if one is available wait until one is 
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
                    self.carter1pub.publish(None)
                    return

                #rospy.loginfo("Generated goal pose: {0}".format(pose))
                goal_msg.target_pose.pose.position.x = pose[0]#set the pose in the goal msg
                goal_msg.target_pose.pose.position.y = pose[1]
                goal_msg.target_pose.pose.orientation.x = pose[2]
                goal_msg.target_pose.pose.orientation.y = pose[3]
                goal_msg.target_pose.pose.orientation.z = pose[4]
                goal_msg.target_pose.pose.orientation.w = pose[5]
                #carter1newgoal = goal_msg
                #rospy.loginfo("Publishing goal c1")
                rospy.sleep(1)
                self.carter1pub.publish(goal_msg)
                #self.carter1count += 1
                #rospy.sleep(.1)

            if(self.carter1Q.qsize()+1 >= 2):
                self.carter1_notfirst = True
            elif(self.carter1Q.qsize() == 0 and self.carter1part_Q.qsize() == 0):
                self.carter1_notfirst = False
            #print("is first =",self.carter1_notfirst)

    #callback for when a robot has reached a destination
    def carter2_ready(self, data):
        #rospy.loginfo("signal to send goal")
        if self.LS_active: #to avoid potential issues with a part getting added when solution module is active
            while(self.LS_active):
                continue
        if self.ZR_active:
            while(self.ZR_active):
                continue
        if data.data == True:
            if not self.carter2part_Q.empty():
                print("size of carter2_partQ:",self.carter2part_Q.qsize()) 
                print("size of carter2Q:",self.carter2Q.qsize())
                print("is first =",self.carter2_notfirst)
                if (((self.carter2Q.qsize() % 2) == 0) and self.carter2_notfirst): #if next point is start point
                    newpart = self.carter2part_Q.get()
                    if newpart.get_transfer():
                        print("carter2 queueing transfer part:",newpart.get_ID())
                        self.queue_part(newpart)
                        self.carter2count = 1
                    else:
                        print("carter2 Adding part:",newpart.get_ID(),"to WS queue")
                        self.add_to_WSQ(newpart)
                        self.carter2count = 1

            if not self.carter2Q.empty():
                newPosition = self.carter2Q.get() #get first item in queue if one is available wait until one is            
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
                    #carter2newgoal=None
                    self.carter2pub.publish(None)
                    return

                #rospy.loginfo("Generated goal pose: {0}".format(pose))
                goal_msg.target_pose.pose.position.x = pose[0]#set the pose in the goal msg
                goal_msg.target_pose.pose.position.y = pose[1]
                goal_msg.target_pose.pose.orientation.x = pose[2]
                goal_msg.target_pose.pose.orientation.y = pose[3]
                goal_msg.target_pose.pose.orientation.z = pose[4]
                goal_msg.target_pose.pose.orientation.w = pose[5]
                #carter2newgoal = goal_msg
                #rospy.loginfo("Publishing goal c2")
                rospy.sleep(1)
                self.carter2pub.publish(goal_msg)

            if(self.carter2Q.qsize()+1 >= 2):
                self.carter2_notfirst = True
            elif(self.carter2Q.qsize() == 0 and self.carter2part_Q.qsize() == 0):
                self.carter2_notfirst = False

    #callback for when a robot has reached a destination
    def carter3_ready(self,data):
        #rospy.loginfo("signal to send goal")
        if self.LS_active: #to avoid potential issues with a part getting added when solution module is active
            while(self.LS_active):
                continue
        if self.ZR_active:
            while(self.ZR_active):
                continue
        if data.data == True:
            if not self.carter3part_Q.empty():
                print("size of carter3_partQ:",self.carter3part_Q.qsize()) 
                print("size of carter3Q:",self.carter3Q.qsize())
                print("is first =",self.carter3_notfirst)
                if (((self.carter3Q.qsize() % 2) == 0) and self.carter3_notfirst): #if next point is start point
                    newpart = self.carter3part_Q.get()
                    if newpart.get_transfer():
                        # at transfer station
                        print("carter3 queueing transfer part:",newpart.get_ID())
                        self.queue_part(newpart)
                    else:
                        print("carter3 Adding part:",newpart.get_ID(),"to WS queue")
                        self.add_to_WSQ(newpart)
        
                
            if not self.carter3Q.empty():
                newPosition = self.carter3Q.get()   
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
                    #carter3newgoal=None
                    self.carter3pub.publish(None)
                    return

                #rospy.loginfo("Generated goal pose: {0}".format(pose))
                goal_msg.target_pose.pose.position.x = pose[0]#set the pose in the goal msg
                goal_msg.target_pose.pose.position.y = pose[1]
                goal_msg.target_pose.pose.orientation.x = pose[2]
                goal_msg.target_pose.pose.orientation.y = pose[3]
                goal_msg.target_pose.pose.orientation.z = pose[4]
                goal_msg.target_pose.pose.orientation.w = pose[5]
                #carter3newgoal = goal_msg
                #rospy.loginfo("Publishing goal c3")
                self.carter3pub.publish(goal_msg)
                rospy.sleep(1)

            if(self.carter3Q.qsize()+1 >= 2):
                self.carter3_notfirst = True
            elif(self.carter3Q.qsize() == 0 and self.carter3part_Q.qsize() == 0):
                self.carter3_notfirst = False

    def gen_intialws_q(self):
        #this priotizes tasks that take the shortest time first (simple)
        routes = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/processing_routes_test.csv', sep=',', header=0, names=['part_type','route','qty'], encoding = 'utf-8')
        time = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/processing_time_test.csv', sep=',', header=0, names=['workstation','processing_time','deviation'], encoding = 'utf-8')
        ws_array = time['workstation'].tolist()
        self.numws = len(ws_array)
        
        for i in range(0,self.numws):
            self.ws_qs.append(Queue(maxsize=0))

        part_types = routes['part_type']
        qty = routes['qty']
        ID_count = 0
        for i in range(0,len(part_types)):
            route = routes.at[i,'route'].split(',')
            ws_index = int(route[0])-1
            for j in range(0,int(qty[i])):
                self.ws_qs[ws_index].put(part(part_types[i], ID_count))
                ID_count += 1
        
        self.num_parts = ID_count
        #print("number of parts",self.num_parts)
        #rospy.loginfo("Done creating initial WS maps")
    
    def queue_part(self, part):
        #functions adds part to robot Q assigned to that zone
        currentws = 'WS' + part.get_currentws()
        nextws = 'WS' + part.get_nextws()
        trans_ws = self.zone.transfer_ws()
        current_zone = -1
        next_zone = -1
        
        if nextws == "WS-1": # check for when part has reach the end
            print("part finished")
            self.num_fin += 1
            part.add_cycleTime()
            self.cycletime.append(part.get_time())
            return

        #get the current zone that part is in
        #if part is not a transfer part then the next_zone will be in the same zone as the current_ws
        current_zone = self.find_wszone(currentws)
        next_zone = self.find_wszone(nextws)

        #if part has been dropped off at transfer station then the 
        #next_zone will be in the same zone as the current_ws
        print("part going through zone:", part.get_tsthru())
        if(part.get_transfer()):
            if(part.get_tsthru()):
                print("recieving thru part:", part.get_ID())
                currentws,nextws = part.at_ts()
                currentws = "WS" + currentws
                nextws = "WS" + nextws
                part.set_transfer(True)
                current_zone = self.find_wszone(currentws)
                next_zone = self.find_wszone(nextws)
                if current_zone == next_zone:
                    part.set_transfer(False)

            else:
                print("recieving transfer part:",part.get_ID())
                current_zone = next_zone
                currentws,nextws = part.at_ts()
                currentws = "WS" + currentws
                nextws = "WS" + nextws
                part.set_transfer(False)

        #case for when part is to be dropped off at transfer station and is a transfer part
        elif (current_zone != next_zone): 
            print("recvieving part:",part.get_ID(),"that is to be dropped off at transfer station")
            part.set_transfer(True)

        print("part:",part.get_ID(),"current zone:", current_zone)
        print("part:",part.get_ID(),"next zone:", next_zone)

        if (not part.get_transfer()): #case for when the recieving part is not being transfered 
            print("recieving part:",part.get_ID()," that is going to be droped off in same zone")
            #add part into robot queue
            if current_zone == 1:
                print("adding part:",part.get_ID(),"to robot Q1")
                self.carter1Q.put(self.WS_position(currentws))
                if(nextws != "WS-1"):
                    self.carter1Q.put(self.WS_position(nextws))
                else:
                    self.carter1Q.put(self.WS_position(currentws))
                self.carter1part_Q.put(part)

            elif current_zone == 2:
                print("adding part:",part.get_ID(),"to robot Q2")
                self.carter2Q.put(self.WS_position(currentws))
                if(nextws != "WS-1"):
                    self.carter2Q.put(self.WS_position(nextws))
                else:
                    self.carter2Q.put(self.WS_position(currentws))
                self.carter2part_Q.put(part)

            elif current_zone == 3:
                print("adding part:",part.get_ID(),"to robot Q3")
                self.carter3Q.put(self.WS_position(currentws))
                if(nextws != "WS-1"):
                    self.carter3Q.put(self.WS_position(nextws))
                else:
                    self.carter3Q.put(self.WS_position(currentws))
                self.carter3part_Q.put(part)

        else:
            print("processing transfer part:",part.get_ID())
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
                    if (next_zone in zonepair) and (not (current_zone in zonepair)):
                        possible_zones = copy.deepcopy(zonepair)
                        if possible_zones[0] == next_zone:
                            possible_zone = possible_zones[1]
                        else:
                            possible_zone = possible_zones[0]
                        zonec = 0
                        for zonep in tszone: 
                            if (possible_zone in zonep) and (current_zone in zonep):
                                possible_ts = trans_ws[zonec]
                                print("part going through one zone and into another")
                                next_zone = possible_zone
                                part.set_tsthru(True)
                                break
                            zonec += 1
                        break
            
            #find closest ts_station
            best_ts = self.find_closestTS(currentws,possible_ts)

            print("beast ts:", best_ts)
            #set part as transfer part
            part.set_transfer(True)
            
            #special cases
            #case for when the transfer station is the next ws, just add part to ws queue
            if(nextws == best_ts):
                print("part:",part.get_ID(),"next ws is at transfer station")
                part.set_transfer(False)
            
            #case for when part that has been processed is at a transfer station
            #need to handle current zone and next zone for when part is a transfer thru part 
            elif(currentws == best_ts):
                print("part:",part.get_ID(),"current ws is at the transfer station, just have robot in next zone pick up part")
                current_zone = next_zone
                if not part.get_tsthru():
                    best_ts = nextws
                    part.set_transfer(False)
                else:
                    #need to find the next possibleTS in the next zone
                    print("next possible_ts:", trans_ws[tszone.index(possible_zones)])
                    best_ts = self.find_closestTS(currentws, trans_ws[tszone.index(possible_zones)])

            part.going_to_ts(best_ts.replace("WS",""))

            if current_zone == 1:
                print("adding part:",part.get_ID(),"to robot Q1")
                self.carter1Q.put(self.WS_position(currentws))
                self.carter1Q.put(self.WS_position(best_ts))
                self.carter1part_Q.put(part)
            elif current_zone == 2:
                print("adding part:",part.get_ID(),"to robot Q2")
                self.carter2Q.put(self.WS_position(currentws))
                self.carter2Q.put(self.WS_position(best_ts))
                self.carter2part_Q.put(part)
            elif current_zone == 3:
                print("adding part:",part.get_ID(),"to robot Q3")
                self.carter3Q.put(self.WS_position(currentws))
                self.carter3Q.put(self.WS_position(best_ts))
                self.carter3part_Q.put(part)

    def find_closestTS(self,currentws,possibleTS):
            #find closest ts station
            ws_dist_mtx = self.workstation_dist_mtx.to_numpy()
            best_ts = possibleTS[0] #default
            ts_dist = 5000
            if len(possibleTS) > 1:
                for ws in possibleTS:
                    new_ts_dist = ws_dist_mtx[self.ws_to_index(currentws)][self.ws_to_index(ws)]
                    if new_ts_dist < ts_dist:
                        best_ts = ws
                        ts_dist = new_ts_dist
            return best_ts

    def find_wszone(self, ws):
        current_zones = self.zone.phase2_ws()
        for x in range(0,len(current_zones)):
            if ws in current_zones[x]:
                return x + 1

    def ws_to_index(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        i_index = ws_list.index(i)
        return i_index

    def WS_position(self,ws):
        
        index = self.ws_to_index(ws)
        x = self.workstation_loc['x']
        y = self.workstation_loc['y']

        x_pos = float(x[index])/3.281
        y_pos = float(y[index])/3.281

        #print("goal (x, y): (",x_pos,",",y_pos,")")
        
        #convert to meters
        return [x_pos, y_pos ,0 ,0 ,1 ,0]

    def add_throughput(self):
        with open('/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/thoughput.csv', 'a') as file:
            writerobj = writer(file)
            currenttime = rospy.get_rostime()
            if len(self.cycletime) != 0:
                average = (sum(self.cycletime)/len(self.cycletime))/60
                writelist = [self.procunits/average, self.procunits, average, currenttime.secs/60]
            else:
                writelist = [0]
            writerobj.writerow(writelist)
            self.cycletime = []
            file.close()

    def process_part(self):
        #function processes the part that is in the queue of the ws 
        #this continually runs parts in the queue
        #wsQ is the workstationQ
        wsQ = self.ws_qs[self.ws_index]
        self.ws_index = (self.ws_index+1) % (self.numws+1)
        index = self.ws_index
        print("creating thread:", index)
        #print("compare",self.num_fin, self.num_parts)
        while(self.num_fin < self.num_parts): #need a better indicator for when to kill the thread
            if not wsQ.empty():
                newpart = wsQ.get() #dequeues from ws Q 
                if newpart.get_pickup() == False:
                    if newpart.get_ifstarting(): #for measuring throughput
                        self.procunits += 1
                    print("Start processing part:",newpart.get_ID(),"in ws:", index)
                    newpart.start_processing()
                    print("Done processing part:",newpart.get_ID(),"in ws:", index)
                    if self.LS_active: #to avoid potential issues with a part getting added when solution module is active
                        while(self.LS_active):
                            continue
                    if self.ZR_active:
                        while(self.ZR_active):
                            continue
                    self.queue_part(newpart)
            rospy.sleep(.01)            
            
    def add_to_WSQ(self, prepart):

        #record where part in load matrix
        row = prepart.get_ogpickup()
        print("previous ws: WS",row)
        col = prepart.get_nextws()
        self.recorded_load[int(row)-1,int(col)-1] += 1
        #self.rec_load_pub(self.recorded_load)
        prepart.at_ws()

        wsq_index = int(prepart.get_currentws()) - 1
        print("part:",prepart.get_ID(),"added to WS:",wsq_index + 1)
        self.ws_qs[wsq_index].put(prepart)

    def get_WS_queue(self):
        return self.ws_qs
    
    def get_numws(self):
        return self.numws
    
    def set_prev_readyFlag(self):
        self.prev_ready1 == False
    
    def pub_rec_loads(self,data):
        if data.data == True:
            print("**********publishing rec loads**************")
            #can only send 1D arrays, encode first
            np_send = self.encode_numpy(self.recorded_load)
            self.rec_loadpub.publish(np_send)
    
    def encode_numpy(self,nparray):
            num_row,num_col = nparray.shape
            np_send = np.zeros(num_row*num_col,dtype=np.float32) 
            #print(np_send)
            for x in range(0,num_row):
                for y in range(0,num_col):
                    np_send[(y+(num_row*x))] = nparray[x,y]

            return np_send

    def call_solution(self,data):
        print("******in solution action*********")
        if data.data == "A":
            self.solution_active = True
        elif data.data == "NA" or data.data == "s_LS":
            print("*****stoping load sharing*******")
            self.solution_active = False
        elif data.data == "a_LS":
            if not self.LS_active:
                self.load_share()
        elif data.data == "a_ZR":
            print("******starting zone repair********")
            #currentzone = copy.deepcopy(self.zone)
            recData = copy.deepcopy(self.recorded_load)
            self.zone = self.solution_mod.zone_repair(self.zone, recData)
            print("new zones self.zone:\n", self.zone.phase2_ws())
            #adjust current Qs to the new zones
            self.ZR_active = True
            print("****update carter 1 qs****")
            adjustQ_part, adjustQ_pos, self.carter1part_Q, self.carter1Q = self.Q_safe_send(self.carter1part_Q,self.carter1Q)
            self.update_qs(adjustQ_part)
            print("****update carter 2 qs****")
            adjustQ_part, adjustQ_pos, self.carter2part_Q, self.carter2Q = self.Q_safe_send(self.carter2part_Q,self.carter2Q)
            self.update_qs(adjustQ_part)
            print("****update carter 3 qs****")
            adjustQ_part, adjustQ_pos, self.carter3part_Q, self.carter3Q = self.Q_safe_send(self.carter3part_Q,self.carter3Q)
            self.update_qs(adjustQ_part)

            #self.zone = copy.deepcopy(currentzone)
            self.ZR_active = False
            
    def load_share(self):
        self.LS_active = True
        print("****start load sharing****")
        #depended on there being 3 robots
        partQ_list = []
        posQ_list = []
        print("****adding carter 1 qs****")
        adjustQ_part, adjustQ_pos, self.carter1part_Q, self.carter1Q = self.Q_safe_send(self.carter1part_Q, self.carter1Q)
        print("size of posQ:", self.carter1part_Q.qsize(), "should be size 2 or less")
        print("size of partQ:", self.carter1Q.qsize(),"should be size 2 or less")
        partQ_list.append(adjustQ_part)
        posQ_list.append(adjustQ_pos)
        print("****adding carter 2 qs****")
        adjustQ_part, adjustQ_pos, self.carter2part_Q, self.carter2Q = self.Q_safe_send(self.carter2part_Q,self.carter2Q)
        print("size of posQ:", self.carter2part_Q.qsize(), "should be size 2 or less")
        print("size of partQ:", self.carter2Q.qsize(),"should be size 2 or less")
        partQ_list.append(adjustQ_part)
        posQ_list.append(adjustQ_pos)
        print("****adding carter 3 qs****")
        adjustQ_part, adjustQ_pos, self.carter3part_Q,self.carter3Q = self.Q_safe_send(self.carter3part_Q,self.carter3Q)
        print("size of posQ:", self.carter3part_Q.qsize(), "should be size 2 or less")
        print("size of partQ:", self.carter3Q.qsize(),"should be size 2 or less")
        partQ_list.append(adjustQ_part)
        posQ_list.append(adjustQ_pos)
        #print("about go into solution mod")
        #print("size of list part q:", len(partQ_list))
        #print("size of list pos q:", len(posQ_list))
        partQ_list, posQ_list = self.solution_mod.load_sharing(partQ_list, posQ_list, self.zone.phase2_ws())
        self.add_adjustQ(self.carter1part_Q,partQ_list[0])
        self.add_adjustQ(self.carter2part_Q,partQ_list[1])
        self.add_adjustQ(self.carter3part_Q,partQ_list[2])
        self.add_adjustQ(self.carter1Q,posQ_list[0])
        self.add_adjustQ(self.carter2Q,posQ_list[1])
        self.add_adjustQ(self.carter3Q,posQ_list[2])

        print("after adjustment")
        print("size of carter1 part Q:", self.carter1part_Q.qsize(), "pos Q:",  self.carter1Q.qsize())
        print("size of carter2 part Q:", self.carter2part_Q.qsize(), "pos Q:",  self.carter2Q.qsize())
        print("size of carter3 part Q:", self.carter3part_Q.qsize(), "pos Q:",  self.carter3Q.qsize())

        self.LS_active = False
        #potential issue
        #if a part is added to the Q while this happening the part will be lost 
        print("****done load sharing****")
        #`rospy.sleep(90) #wait a minute thirty

    def Q_safe_send(self,partQ,posQ):
        #adjust robot Qs so that the current part is not affected
        #only works if Q are not empty
        tempposQ = Queue() #these will be sent
        temppartQ = Queue()

        #to be through
        copyPartQ = Queue()
        copyPosQ = Queue()
        
        if partQ.qsize() <= 2: #when robot only had one part in Q
            print("2 or less parts in Q, returning")
            return temppartQ,tempposQ,partQ,posQ #return empty Queues
    
        elif posQ.qsize() % 2 == 0: #robot on its way to drop off
            print("robot is dropping off")
            pos1 = posQ.get() #these two point corespond to the next part in queue 
            pos2 = posQ.get()
            copyPosQ.put(pos1)
            copyPosQ.put(pos2)
            part1 = partQ.get() #this part is the the part that is going to be droped off
            part2 = partQ.get() #this part corresponds to pos1 and pos2
            #print("adding part:", part1.get_ID(),"and",part2.get_ID(),"to og Q drop off")
            copyPartQ.put(part1)
            copyPartQ.put(part2)

        elif posQ.qsize() % 2 == 1:
            print("robot is picking up")
            pos1 = posQ.get() #robot is picking up
            copyPosQ.put(pos1)

            part1 = partQ.get()
            #print("adding part:", part1.get_ID(),"to og Q pickup")
            copyPartQ.put(part1)

        print("should be even")
        print("size of posQ:", posQ.qsize())
        print("size of partQ:", partQ.qsize())

        while(not partQ.empty()):
            pos1 = posQ.get()
            pos2 = posQ.get()

            part = partQ.get()
            
            #print("adding part:", part.get_ID(),"to tempQ")
            print("part:", part.get_ID(), "pickup:",pos1, "dropoff", pos2)

            tempposQ.put(pos1)
            tempposQ.put(pos2)

            temppartQ.put(part)

        return temppartQ,tempposQ,copyPartQ,copyPosQ
             
    def add_adjustQ(self,pQ,adjustQ):
        while(not adjustQ.empty()):
            temp = adjustQ.get()
            pQ.put(temp)

    def update_qs(self,partQ):
        while(not partQ.empty()):
            part = partQ.get()
            part.set_transfer(False)
            self.queue_part(part)

    def pub_zone(self,data):
        if (len(self.ws_qs) != 0):
            self.encode_zones()

    def encode_zones(self):
        for zone in self.zone.phase2_ws():
            msg = stringarr()
            msg.data = zone
            self.zonepub.publish(msg)
            rospy.sleep(.01)
        
        zonei = 0
        for zone in self.zone.phase2_cs():
            for critseg in zone:
                #critseg = critseg.tolist()
                segment = copy.deepcopy(critseg)
                if isinstance(segment,np.ndarray):
                    segment = list(segment)
                segment.insert(0,str(zonei))
                msg = stringarr()
                msg.data = segment
                self.critpub.publish(msg)
                rospy.sleep(.01)          
            zonei +=1
    
    def record_throughputdata(self):
        print("starting throughput thread")
        max_period = 50
        time_period = 10
        current_period = 0
        while(current_period < max_period):
            currenttime = rospy.get_rostime()
            time = (currenttime.secs - self.startTime.secs)/60
            if(time > time_period):
                self.add_throughput()
                self.startTime = rospy.get_rostime()
                current_period += 1
            
            rospy.sleep(1)
        

def main():
    sys.path.insert(0,'/home/russell/thesis_ws/src/navigation/zone_assignment')
    rospy.init_node("robot_tasks_py") #create node
    rospy.loginfo("running robot tasks")

    best_zone = zone_control()
    '''
    new_loads = np.array([ #continueing on
        #1,2,3,4,5,6,7,8,9,10,11
        [0,0,0,0,0,0,6,0,0,0,0], #1
        [0,0,7,0,0,0,0,0,0,0,0], #2
        [0,0,0,3,7,0,0,0,0,0,0], #3
        [0,0,0,0,0,5,0,1,0,0,0], #4
        [0,0,0,5,0,5,5,0,0,0,0], #5
        [0,0,0,0,0,0,0,0,5,0,0], #6
        [0,0,6,0,0,0,0,5,4,0,0], #7
        [0,0,0,0,1,0,4,0,0,0,0], #8
        [0,0,0,0,0,0,0,0,0,4,0], #9
        [0,0,0,0,0,0,0,0,0,0,0], #10
        [0,0,0,0,0,0,0,0,0,0,0], #11
    ])
    best_zone.zone_reparation(new_loads)
    '''
    print("creating robot tasks")
    tasks = robot_task(best_zone)
    
    while not rospy.is_shutdown():
        #multithreading the WS queues
        pool = concurrent.futures.ThreadPoolExecutor(max_workers=tasks.get_numws() +1 )#add one for data recording
        #rospy.loginfo("creating threads")
        for x in range(0,tasks.get_numws()): #threads are created here and call Process part function
            pool.submit(tasks.process_part)
            time.sleep(0.1)
        
        pool.submit(tasks.record_throughputdata)

        pool.shutdown(wait=True)
        rospy.signal_shutdown("parts are finished processing")
    
    
if __name__ == "__main__":
    main()