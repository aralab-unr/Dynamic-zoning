#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Bool
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from isaac_sim_zone_SA.msg import stringarr
#from rospy_tutorials.msg import HeaderString
from queue import Queue
import copy
import numpy as np
import pandas as pd
#from goal_generators import RandomGoalGenerator, GoalReader
from zone_SA.ZoneControl import zone_control
from zone_SA.part import part
from Solution import Solution_module
import concurrent.futures
import time
from rospy.numpy_msg import numpy_msg
import os
import rospkg
from csv import writer, reader
import math


class robot_task(Solution_module):
    def __init__(self, zone_assignment):
        print("*********in new robot tasks********")
        #instead of a pos queue and a part queue have a sorted dictionary
        self.carter1D = {} #hold score and part
        self.carter2D = {} #structured from worst -> best
        self.carter3D = {}
        self.carter1_pos = '1' #holds current position that the robot was sent too
        self.carter2_pos = '1' 
        self.carter3_pos = '1'
        self.carter1_pickup = True #flag to tell robot to move to drop off location
        self.carter2_pickup = True
        self.carter3_pickup = True
        self.carter1_part = None #current part that robot is processesing
        self.carter2_part = None
        self.carter3_part = None

        self.solution_active = False
        self.LS_active = False
        self.ZR_active = False

        self.carter1pub = rospy.Publisher("/carter1/newgoal", MoveBaseGoal, queue_size = 10) #create publisher for sending new goal
        self.carter2pub = rospy.Publisher("/carter2/newgoal", MoveBaseGoal, queue_size = 10)
        self.carter3pub = rospy.Publisher("/carter3/newgoal", MoveBaseGoal, queue_size = 10) 
        self.rec_loadpub = rospy.Publisher("/rec_loads", numpy_msg(Floats), queue_size = 10)
        self.zonepub = rospy.Publisher("/zones", stringarr, queue_size = 10) #publishers for sending zones and crit data
        self.critpub = rospy.Publisher("/critseg", stringarr, queue_size = 10)
        rospy.Subscriber("/carter1/ready_flag",Bool,self.carter1_ready) #create subsriber to see is robot is ready for a new goal
        rospy.Subscriber("/carter2/ready_flag",Bool,self.carter2_ready) #create subsriber to see is robot is ready for a new goal
        rospy.Subscriber("/carter3/ready_flag",Bool,self.carter3_ready) #create subsriber to see is robot is ready for a new goal
        rospy.Subscriber("/send_load",Bool,self.pub_rec_loads) #to send recored drop off loads to monitor module
        rospy.Subscriber("/send_zone",Bool,self.pub_zone) #to send current zone information to monitor module
        self.ws_qs = []
        self.zone = zone_assignment #is an object
        self.num_zones = len(self.zone.phase2_ws())
        self.ws_index = 0
        #to end program
        self.num_fin = 0
        self.num_parts = 0

        #filepath = os.path.realpath("robot_tasks.py")
        #print(filepath)

        self.workstation_loc = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','Workstation_Loaction.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
        self.workstation_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','Workstation_points.csv'), sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        self.workstation_dist_mtx = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','WS_dist_mtx.csv'), sep=',')
        self.numws = len(self.workstation_points['workstation'])

        self.recorded_load = np.zeros((self.numws,self.numws),dtype=np.float32)
        #historic_recload = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal_SA/isaac_ros_navigation_goal_SA/zone_SA/Workstation_Loads.csv', sep=',', header=0, names=['WS1','WS2','WS3','WS4','WS5','WS6','WS7','WS8','WS9','WS10','WS11'], encoding = 'utf-8')
        #self.recorded_load = historic_recload.to_numpy()
        '''
        self.recorded_load = np.array( #continueing on
            [[ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  2.,  0.,  0.],
            [14.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0., 15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0., 15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0., 15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  3.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  3.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  4.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 10.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  3.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 10.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.]]
        )
        '''
        self.rec_loadpub = rospy.Publisher("rec_loads", numpy_msg(Floats), queue_size = 15)

        #for solution module
        rospy.Subscriber("/solution_action",String,self.call_solution) #to send recored drop off loads to monitor module
        self.solution_mod = Solution_module()

        #for data recording
        self.startTime = rospy.get_rostime()
        self.window = 20 #defines how long a data point stays recorded
        self.rec_pttimes = {} #keeps track of how long a pt has been recorded 
        for row in range(self.numws):
            for col in range(self.numws):
                self.rec_pttimes[row,col] = []
        pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        pool.submit(self.rolling_window)

        #record initial SVp
        with open(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data','SVphistory.csv'), 'a') as file:
            writerobj = writer(file)
            writelist = [0,self.zone.get_SVp()]
            writerobj.writerow(writelist)
            file.close()

        self.gen_intialws_q()

    #callback for when a robot has reached a destination
    def carter1_ready(self,data):
        #print("carter 1 signal to send goal:", data.data)
        if self.LS_active: #to avoid potential issues with a part getting added when solution module is active
            while(self.LS_active):
                continue
        if self.ZR_active:
            while(self.ZR_active):
                continue
        if (data.data == True):# and (self.prev_ready1 == False or self.carter1count == 1)):
            if (self.carter1D or self.carter1_part is not None): #check if dictionary is empty
                print("size of carter1D:",len(self.carter1D))
                print("picking up =",self.carter1_pickup)
                prevpos = 'WS' + self.carter1_pos
                #*******to drop off part**********
                if (self.carter1_part != None):
                    if(self.carter1_part.get_dropoff() and self.carter1_pickup): #drop off part
                        print("carter 1 dropping off")
                        rospy.sleep(2.5) #time to drop off a part
                        if self.carter1_part.get_transfer():
                            print("carter1 queueing transfer part:",self.carter1_part.get_ID())
                            self.queue_part(self.carter1_part) #dropping off transfer part
                        else:
                            print("carter1 Adding part:",self.carter1_part.get_ID(),"to WS queue")
                            self.add_to_WSQ(self.carter1_part) #dropping off part at workstation 
                        self.carter1_part = None
                        self.update_part_status(self.carter1D)

                #*******to go to drop off location*******
                if (not self.carter1_pickup): #go to drop off location
                    print("carter 1 going to drop off")
                    self.carter1_pos = self.carter1_part.get_nextws() #record the drop off position as the current position
                    self.carter1_pickup = True #switch sign to get ready to pick-up again
                    self.carter1_part.set_dropoff(True) #signal to dropoff part
                    rospy.sleep(2.5) #time to pick up off a part

                #*******to go to pick up location********
                else: #go to pick-up location
                    print("Carter 1 going to pick up")
                    if not self.carter1D: #check to make sure a part is available
                        return
                    self.carter1D = self.rank_parts(self.carter1D, self.carter1_pos)
                    item = self.carter1D.popitem() # get next part and save as current part
                    self.carter1_part = item[0]
                    self.carter1_pos = self.carter1_part.get_currentws()
                    self.carter1_part.set_dropoff(False) #signal to not dropoff part yet
                    self.carter1_pickup = False # switch sign to get ready to drop-off again
                    print("going to pick up part:", self.carter1_part.get_ID())
                
                #****create a position and send message****
                print("carter 1 going to WS:",'WS' + self.carter1_pos)
                #record the next position in file
                self.record_robot_pos('WS'+self.carter1_pos,1,self.zone.shortest_dist(prevpos, 'WS' + self.carter1_pos))
                newPosition = self.WS_position('WS' + self.carter1_pos) #create position
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
                #rospy.loginfo("Publishing goal c1")
                self.carter1pub.publish(goal_msg)
                rospy.sleep(1)

    #callback for when a robot has reached a destination
    def carter2_ready(self, data):
        #print("carter 1 signal to send goal:", data.data)
        if self.LS_active: #to avoid potential issues with a part getting added when solution module is active
            while(self.LS_active):
                continue
        if self.ZR_active:
            while(self.ZR_active):
                continue
        if (data.data == True):# and (self.prev_ready1 == False or self.carter2count == 1)):
            if (self.carter2D or self.carter2_part != None): #check if dictionary is empty
                print("size of carter2D:",len(self.carter2D))
                print("picking up =",self.carter2_pickup)
                prevpos = 'WS' + self.carter2_pos
                #*******to drop off part**********
                if (self.carter2_part != None):
                    if(self.carter2_part.get_dropoff() and self.carter2_pickup): #drop off part
                        print("carter 2 dropping off")
                        rospy.sleep(2.5) #time to drop off a part
                        if self.carter2_part.get_transfer():
                            print("carter2 queueing transfer part:",self.carter2_part.get_ID())
                            self.queue_part(self.carter2_part) #dropping off transfer part
                        else:
                            print("carter2 Adding part:",self.carter2_part.get_ID(),"to WS queue")
                            self.add_to_WSQ(self.carter2_part) #dropping off part at workstation 
                        self.carter2_part = None
                        self.update_part_status(self.carter2D)

                #*******to go to drop off location*******
                if (not self.carter2_pickup): #go to drop off location
                    print("carter 2 going to drop off")
                    self.carter2_pos = self.carter2_part.get_nextws() #record the drop off position as the current position
                    self.carter2_pickup = True #switch sign to get ready to pick-up again
                    self.carter2_part.set_dropoff(True) #signal to dropoff part
                    rospy.sleep(2.5) #time to pick up off a part

                #*******to go to pick up location********
                else: #go to pick-up location
                    print("Carter 2 going to pick up")
                    if not self.carter2D: #check to make sure a part is available
                        return
                    self.carter2D = self.rank_parts(self.carter2D, self.carter2_pos)
                    item = self.carter2D.popitem() # get next part and save as current part
                    self.carter2_part = item[0]
                    self.carter2_pos = self.carter2_part.get_currentws()
                    self.carter2_part.set_dropoff(False) #signal to not dropoff part yet
                    self.carter2_pickup = False # switch sign to get ready to drop-off again
                    print("going to pick up part:", self.carter2_part.get_ID())
                
                #****create a position and send message****
                print("carter 2 going to WS:",'WS' + self.carter2_pos)
                #record the next position in file
                self.record_robot_pos('WS'+self.carter2_pos,2,self.zone.shortest_dist(prevpos, 'WS' + self.carter2_pos))
                newPosition = self.WS_position('WS' + self.carter2_pos) #create position
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
                    #carter2newgoal = None
                    self.carter2pub.publish(None)
                    return

                #rospy.loginfo("Generated goal pose: {0}".format(pose))
                goal_msg.target_pose.pose.position.x = pose[0]#set the pose in the goal msg
                goal_msg.target_pose.pose.position.y = pose[1]
                goal_msg.target_pose.pose.orientation.x = pose[2]
                goal_msg.target_pose.pose.orientation.y = pose[3]
                goal_msg.target_pose.pose.orientation.z = pose[4]
                goal_msg.target_pose.pose.orientation.w = pose[5]
                #rospy.loginfo("Publishing goal c1")
                self.carter2pub.publish(goal_msg)
                rospy.sleep(1)

    #callback for when a robot has reached a destination
    def carter3_ready(self,data):
        #print("carter 1 signal to send goal:", data.data)
        if self.LS_active: #to avoid potential issues with a part getting added when solution module is active
            while(self.LS_active):
                continue
        if self.ZR_active:
            while(self.ZR_active):
                continue
        if (data.data == True):# and (self.prev_ready1 == False or self.carter3count == 1)):
            
            if (self.carter3D or self.carter3_part != None): #check if dictionary is empty
                print("size of carter3D:",len(self.carter3D))
                print("picking up =",self.carter3_pickup)
                prevpos = 'WS' + self.carter3_pos
                #*******to drop off part**********
                if (self.carter3_part != None):
                    if(self.carter3_part.get_dropoff() and self.carter3_pickup): #drop off part
                        print("carter 3 dropping off")
                        rospy.sleep(2.5) #time to drop off a part
                        if self.carter3_part.get_transfer():
                            print("carter3 queueing transfer part:",self.carter3_part.get_ID())
                            self.queue_part(self.carter3_part) #dropping off transfer part
                        else:
                            print("carter3 Adding part:",self.carter3_part.get_ID(),"to WS queue")
                            self.add_to_WSQ(self.carter3_part) #dropping off part at workstation 
                        self.carter3_part = None
                        self.update_part_status(self.carter3D)

                #*******to go to drop off location*******
                if (not self.carter3_pickup): #go to drop off location
                    print("carter 3 going to drop off")
                    self.carter3_pos = self.carter3_part.get_nextws() #record the drop off position as the current position
                    self.carter3_pickup = True #switch sign to get ready to pick-up again
                    self.carter3_part.set_dropoff(True) #signal to dropoff part
                    rospy.sleep(2.5) #time to pick up off a part

                #*******to go to pick up location********
                else: #go to pick-up location
                    print("Carter 3 going to pick up")
                    if not self.carter3D: #check to make sure a part is available
                        return
                    self.carter3D = self.rank_parts(self.carter3D, self.carter3_pos)
                    item = self.carter3D.popitem() # get next part and save as current part
                    self.carter3_part = item[0]
                    self.carter3_pos = self.carter3_part.get_currentws()
                    self.carter3_part.set_dropoff(False) #signal to not dropoff part yet
                    self.carter3_pickup = False # switch sign to get ready to drop-off again
                    print("going to pick up part:", self.carter3_part.get_ID())
                
                #****create a position and send message****
                print("carter 3 going to WS:",'WS' + self.carter3_pos)
                #record the next position in file
                self.record_robot_pos('WS'+self.carter3_pos, 3, self.zone.shortest_dist(prevpos, 'WS' + self.carter3_pos))
                newPosition = self.WS_position('WS' + self.carter3_pos) #create position
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
                    #carter3newgoal = None
                    self.carter3pub.publish(None)
                    return

                #rospy.loginfo("Generated goal pose: {0}".format(pose))
                goal_msg.target_pose.pose.position.x = pose[0]#set the pose in the goal msg
                goal_msg.target_pose.pose.position.y = pose[1]
                goal_msg.target_pose.pose.orientation.x = pose[2]
                goal_msg.target_pose.pose.orientation.y = pose[3]
                goal_msg.target_pose.pose.orientation.z = pose[4]
                goal_msg.target_pose.pose.orientation.w = pose[5]
                #rospy.loginfo("Publishing goal c1")
                #rospy.sleep(1)
                self.carter3pub.publish(goal_msg)
                rospy.sleep(1)

    def gen_intialws_q(self):
        routes = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','processing_routes_test.csv'), sep=',', header=0, names=['part_type','route','qty'], encoding = 'utf-8')
        
        for i in range(0,self.numws): #create queues for every workstation
            self.ws_qs.append(Queue(maxsize=0))

        part_types = routes['part_type']
        qty = routes['qty']
        ID_count = 0
        for i in range(len(part_types)): #create part objects and assign them to thier first WS
            route = routes.at[i,'route'].split(',')
            ws_index = int(route[0])-1
            for j in range(int(qty[i])):
                self.ws_qs[ws_index].put(part(part_types[i], ID_count))
                ID_count += 1
        
        self.num_parts = ID_count
    
    def queue_part(self, part):
        #functions adds part to robot Q assigned to that zone
        currentws = 'WS' + part.get_currentws() #part pick up ws
        nextws = 'WS' + part.get_nextws() #part drop off ws
        trans_ws = self.zone.transfer_ws() #get zone transfer stations
        current_zone = -1
        next_zone = -1
        
        if nextws == "WS-1": # check for when part has reached the end
            print("part finished")
            self.num_fin += 1
            part.add_cycleTime()
            return

        #get the current zone that part is in
        #if part is not a transfer part then the next_zone will be in the same zone as the current_ws
        current_zone = self.find_wszone(currentws)
        next_zone = self.find_wszone(nextws)

        #if part has been dropped off at transfer station then the 
        #next_zone will be in the same zone as the current_ws
        print("part going through zone:", part.get_tsthru())
        if(part.get_transfer()):
            if(part.get_tsthru()): #case for when a part is passing through a zone
                print("recieving thru part:", part.get_ID())
                currentws,nextws = part.at_ts()
                currentws = "WS" + currentws
                nextws = "WS" + nextws
                part.set_transfer(True)
                current_zone = self.find_wszone(currentws)
                next_zone = self.find_wszone(nextws)
                if current_zone == next_zone:
                    part.set_transfer(False)

            else: #case for when a part is a transfer part that has a end traget in next zone
                print("recieving transfer part:",part.get_ID())
                currentws,nextws = part.at_ts()
                currentws = "WS" + currentws
                nextws = "WS" + nextws
                current_zone = self.find_wszone(currentws)
                next_zone = self.find_wszone(nextws)
                part.set_transfer(False)

        #case for when part is to be dropped off at transfer station and is a transfer part
        elif (current_zone != next_zone): 
            print("recvieving part:",part.get_ID(),"that is to be dropped off at transfer station")
            part.set_transfer(True)

        print("part:",part.get_ID(),"current zone:", current_zone)
        print("part:",part.get_ID(),"next zone:", next_zone)

        if (not part.get_transfer()): #case for when the recieving part is not being transfered and being dropped in same zone 
            print("recieving part:",part.get_ID()," that is going to be droped off in same zone as nextws robot")
            #add part into robot queue
            if next_zone == 1:
                print("adding part:",part.get_ID(),"to robot 1")
                self.carter1D[part] = part.score(self.carter1_pos,self.zone) #do not score yet allow the

            elif next_zone == 2:
                print("adding part:",part.get_ID(),"to robot 2")
                self.carter2D[part] = part.score(self.carter2_pos,self.zone)

            elif next_zone == 3:
                print("adding part:",part.get_ID(),"to robot 3")
                self.carter3D[part] = part.score(self.carter3_pos,self.zone)

        else:
            print("processing transfer part:",part.get_ID())
            #find closest transfer station from current point
            #[(1,2),(1,3),(2,3)] -transfer stations
            #need to edit this so that it is not depended on the current 3 zone deisgn
            #possible_ts = []
            if((current_zone == 1 and next_zone == 2) or (current_zone == 2 and next_zone == 1)): possible_ts = trans_ws[0]
            elif((current_zone == 1 and next_zone == 3) or (current_zone == 3 and next_zone == 1)): possible_ts = trans_ws[1] 
            elif((current_zone == 2 and next_zone == 3) or (current_zone == 3 and next_zone == 2)): possible_ts = trans_ws[2] 

            for pairi in range(len(self.tszone)):
                if (current_zone in self.tszone[pairi]) and (next_zone in self.tszone[pairi]):
                    possible_ts = trans_ws[pairi]
                    break

            part.set_tsthru(False)

            print("possible transfer stations for zone:", current_zone,"\n",possible_ts)

            #case for when a zone doesnt have a transfer station 
            #move part though to another zone
            possible_zones = []
            if len(possible_ts) == 0:
                for zonepair in self.tszone:
                    if (next_zone in zonepair) and (current_zone not in zonepair):
                        possible_zones = copy.deepcopy(zonepair)
                        if possible_zones[0] == next_zone:
                            possible_zone = possible_zones[1]
                        else:
                            possible_zone = possible_zones[0]
                        zonec = 0
                        for zonep in self.tszone: 
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
            print("best ts:", best_ts)

            #set part as transfer part
            part.set_transfer(True)
            
            #special cases
            #case for when the transfer station is the next ws, just add part to ws queue
            if(nextws == best_ts):
                print("part:",part.get_ID(),"next ws is at transfer station")
                part.set_transfer(False)
            
            #case for when part that has been processed is at a transfer station
            elif(currentws == best_ts):
                print("part:",part.get_ID(),"current ws is at the transfer station, just have robot in next zone pick up part")
                current_zone = next_zone
                if not part.get_tsthru():
                    best_ts = nextws
                    part.set_transfer(False)
                else:
                    print("next possible_ts:", trans_ws[self.tszone.index(possible_zones)])
                    best_ts = self.find_closestTS(currentws, trans_ws[self.tszone.index(possible_zones)]) #find the next transfer station

            part.going_to_ts(best_ts.replace("WS",""))

            if current_zone == 1:
                print("adding part:", part.get_ID(),"to robot 1")
                self.carter1D[part] = part.score(self.carter1_pos,self.zone)
            elif current_zone == 2:
                print("adding part:", part.get_ID(),"to robot 2")
                self.carter2D[part] = part.score(self.carter2_pos,self.zone)
            elif current_zone == 3:
                print("adding part:", part.get_ID(),"to robot 3")
                self.carter3D[part] = part.score(self.carter3_pos,self.zone)

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
        with open(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data','thoughput.csv'), 'a') as file:
            writerobj = writer(file)
            currenttime = rospy.get_rostime()
            writelist = [((currenttime.secs/60)/60), self.num_fin, self.num_fin/((currenttime.secs/60)/60)]
                
            writerobj.writerow(writelist)
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

                    #record part in load matrix
                    row = newpart.get_ogpickup()
                    col = newpart.get_ogdropoff()
                    if col != "-1": #signal for when part has finished
                        self.recorded_load[int(row)-1,int(col)-1] += 1

            rospy.sleep(.01)            
            
    def add_to_WSQ(self, prepart):
        
        row = prepart.get_ogpickup()
        #print("previous ws: WS",row)
        col = prepart.get_ogdropoff()
        #self.recorded_load[int(row)-1,int(col)-1] += 1

        #start time for log, rolling window
        currentTime = rospy.get_rostime()
        self.rec_pttimes[int(row)-1, int(col)-1].append(currentTime.secs)
        
        prepart.at_ws()

        wsq_index = int(prepart.get_currentws()) - 1
        print("part:",prepart.get_ID(),"added to WS:",wsq_index + 1)
        self.ws_qs[wsq_index].put(prepart)

    def get_WS_queue(self):
        return self.ws_qs
    
    def get_numws(self):
        return self.numws
    
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
            print("*****starting load sharing*******")
            if not self.LS_active:
                self.load_share()
        elif data.data == "a_ZR":
            print("******starting zone repair********")
            recData = copy.deepcopy(self.recorded_load)
            zone_c = copy.deepcopy(self.zone)
            self.zone = self.solution_mod.zone_repair(zone_c, recData)
            print("new zones self.zone:\n", self.zone.phase2_ws())
            #record SVp
            current_time = rospy.get_rostime()
            with open(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data','SVphistory.csv'), 'a') as file:
                writerobj = writer(file)
                writelist = [current_time.secs,self.zone.get_SVp()]
                writerobj.writerow(writelist)
                file.close()

            #adjust current Qs to the new zones
            self.ZR_active = True
            self.update_Dic() #add parts to the appropiate robot

            #self.zone = copy.deepcopy(currentzone)
            self.ZR_active = False
            
    def load_share(self):
        self.LS_active = True
        print("****start load sharing****")
        #dependent on there being 3 robots
        part_list = [[]]*self.num_zones

        part_list[0] = list(self.carter1D.keys()) #create a list of values from each dictionary
        part_list[1] = list(self.carter2D.keys())
        part_list[2] = list(self.carter3D.keys())

        self.carter1D.clear() #clear each dictionary
        self.carter2D.clear()
        self.carter3D.clear()

        part_list = self.solution_mod.load_sharing(part_list, self.zone.phase2_ws()) #start load sharing

        self.add_adjustD(self.carter1D, part_list[0], self.carter1_pos) #add adjusted parts to each robot dictionary
        self.add_adjustD(self.carter2D, part_list[1], self.carter2_pos)
        self.add_adjustD(self.carter3D, part_list[2], self.carter3_pos)

        print("after adjustment")
        print("size of carter1 part Q:", len(self.carter1D))
        print("size of carter2 part Q:", len(self.carter2D))
        print("size of carter3 part Q:", len(self.carter3D))

        self.LS_active = False
        print("****done load sharing****")
             
    def add_adjustD(self, pD, adjustL, robotpos):
        for part in adjustL:
            score = part.score(robotpos, self.zone)
            pD[part] = score

    def update_Dic(self):
        #pool together each part
        pool = []
        while self.carter1D:
            pool.append(list(self.carter1D.keys())[-1])
            self.carter1D.popitem()
        while self.carter2D:
            pool.append(list(self.carter2D.keys())[-1])
            self.carter2D.popitem()
        while self.carter3D:
            pool.append(list(self.carter3D.keys())[-1])
            self.carter3D.popitem()
            
        for part in pool:
            part.set_transfer(False)
            part.set_tsthru(False)
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
        max_period = 200
        time_period = 5 #record throughput every 5 min
        current_period = 0
        while(current_period < max_period):
            currenttime = rospy.get_rostime()
            time = (currenttime.secs - self.startTime.secs)/60
            if(time > time_period):
                self.add_throughput()
                self.startTime = rospy.get_rostime()
                current_period += 1
            
            rospy.sleep(1)
    
    def rank_parts(self, robotd, robotpos):
        #re-score each part in a dictionary and sort
        partsList = []
        partsList = list(robotd.keys())
        robotd.clear()

        for part in partsList: #re-score
            score = part.score(robotpos,self.zone)
            robotd[part] = score

        robot_sorted_list = sorted(robotd.items(), key=lambda x:x[1]) #sorted list
        robotd = dict(robot_sorted_list)

        #printing 
        for key in robotd:
            print("score:",robotd[key],"part ID:",key.get_ID())

        return robotd

    def update_part_status(self, partdic):
        for part in partdic.keys():
            part.write_status()

    def record_robot_pos(self,WS,robot,dist):
        #WS is writen as "WS1" and robot is a number:
        myrow = [['']]*self.num_zones
        with open(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data','visitedWS.csv'), 'r') as file:
            alllines = reader(file)
            i=0
            #for row in alllines:
                #myrow[i] = list(row)
                #i += 1
            myrow = list(alllines)
            file.close()

        if myrow[robot][1] != '':
            calcdist = float(myrow[robot][1]) + dist
        else:
            calcdist = 0
        
        myrow[robot] = [myrow[robot][0],  str(calcdist), myrow[robot][2] + ',' + WS]
        with open(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data','visitedWS.csv'), 'w') as file:
            alllines = writer(file)
            for row in myrow:
                alllines.writerow(row)
            file.close()

        distfile = 'carter'+ str(robot) + 'dist.csv'
        current_time = rospy.get_rostime()
        with open(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data', distfile), 'a') as file:
            writerobj = writer(file)
            writelist = [current_time.secs, calcdist]
            writerobj.writerow(writelist)
            file.close()
        

    def rolling_window(self):
        while(True):
            currentTime = rospy.get_rostime()
            for pt, times in self.rec_pttimes.items(): #retrieves key value pair
                copytimes = copy.deepcopy(times)
                for time in copytimes:
                    if ((currentTime.secs - time) > self.window*60):
                        times.remove(time)
                        self.recorded_load[pt[0],pt[1]] -= 1
                        print("removed 1 from:", pt[0],pt[1])
            rospy.sleep(10)
            #print(self.recorded_load)




def main():
    #sys.path.insert(0,'/home/russell/thesis_ws/src/navigation/zone_assignment')
    rospy.init_node("robot_tasks_py") #create node
    rospy.loginfo("running robot tasks SA")

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
