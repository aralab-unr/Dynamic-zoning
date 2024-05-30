#!/usr/bin/env python
#!/usr/bin/env python
import pandas as pd
import numpy as np
import math
import rospy
import copy
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from itertools import combinations 
from queue import Queue
from zone.part import part
import multiprocessing

class Solution_module():
    
    def __init__(self):
        #self.sys_active = active_check #bool
        #self.action = action
        #self.num_zones = len(self.list_posQ)
        self.num_zones = 0

        self.workstation_points = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/Workstation_points.csv', sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        self.workstation_loc = pd.read_csv(r'/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/Workstation_Loaction.csv', sep=',', header=0, names=['x','y'], encoding = 'utf-8')

    def load_sharing(self, list_partQ, list_posQ, current_ws):
        #look at parts that are waiting for the robot to pickup
            #the robots Q
            #these parts are waiting to be prcoessed
        #starting for the third or second part
            #if the part's next destination is in a zone that is not busy
            #add part to next zone robot Q(only if robot Q in next zone is 0 or 1)

            #if the next zone is busy have robot bypass the transfer station and deliver part to the next WS
        
        print("in load sharing")
        #get all possible combinations
        self.num_zones = len(list_posQ)
        size_of_comb = []
        for x in range(0,self.num_zones): size_of_comb.append(x)
        all_comb = list(combinations(size_of_comb, 2))
        print("all comb\n",all_comb)
        #neighboring_zone example
        #[(0,1),(0,2),(1,2)]

        part_notadded = True
        #tobe_partQ = [[]]*self.num_zones

        #first case when a part needs to droped off in a less busy zone
        for comb in all_comb:
            #which zone is more overloaded
            print("size of comb 0 q:", list_partQ[comb[0]].qsize())
            print("size of comb 1 q:", list_partQ[comb[1]].qsize())

            if list_partQ[comb[0]].qsize() > list_partQ[comb[1]].qsize():
                bigpartQ = list_partQ[comb[0]]
                smallpartQ = list_partQ[comb[1]]
                bigposQ = list_posQ[comb[0]]
                smallposQ = list_posQ[comb[1]]
                bigzone = current_ws[comb[0]]
                smallzone = current_ws[comb[1]]
                bigQindex = comb[0]
                smallQindex = comb[1]

            elif list_partQ[comb[0]].qsize() < list_partQ[comb[1]].qsize():
                bigpartQ = list_partQ[comb[1]]
                smallpartQ = list_partQ[comb[0]]
                bigposQ = list_posQ[comb[1]]
                smallposQ = list_posQ[comb[0]]
                bigzone = current_ws[comb[1]]
                smallzone = current_ws[comb[0]]
                bigQindex = comb[1]
                smallQindex = comb[0]

            else:
                #case for when the Qs are equal also for if both Queus are empty
                continue
            
            #for every part in big Q see if next WS is in small Q's zone
            #only do this once
            print("size of biq part q:", bigpartQ.qsize())
            print("size of small part q:", smallpartQ.qsize())
            BQsize = bigpartQ.qsize()
            SQsize = smallpartQ.qsize()
            
            for x in range(BQsize):
                print("looking at big q")
                part = bigpartQ.get()
                pickuppt = bigposQ.get() #this may not be the currentws if transfer part
                dropoffpt = bigposQ.get() #this position doesn't change
                #Even if the part is transfer part the nextws doesn't change
                nextws = 'WS' + part.get_nextws()
                print("looking at part:",part.get_ID())
                print("current ws:",part.get_currentws())
                print("nextws:",part.get_nextws())

                if(nextws in smallzone and part_notadded):
                    #move part and pos over to smallQ
                    print("putting part",part.get_ID(),"from big to small zone")
                    print("big zone:",bigQindex,"small zone:",smallQindex)
                    #need to get position from WS
                    smallposQ.put(self.WS_position("WS"+part.get_currentws()))
                    smallposQ.put(self.WS_position("WS"+part.get_nextws()))

                    part.set_transfer(False)
                    smallpartQ.put(part)

                    part_notadded = False #only do this once

                else:
                    #loop through big partQ and make sure parts are added back
                    bigpartQ.put(part)
                    bigposQ.put(pickuppt)
                    bigposQ.put(dropoffpt)


            part_notadded = True
            #case for when a part needs to be transfered from small to big
            for x in range(SQsize):
                print("looking at small q")
                part = smallpartQ.get()
                pickuppt = smallposQ.get() #this may not be the currentws if transfer part
                dropoffpt = smallposQ.get() #this position doesn't change
                #Even if the part is transfer part the nextws doesn't change
                nextws = 'WS' + part.get_nextws()

                print("looking at part:",part.get_ID())
                print("current ws:",part.get_currentws())
                print("next ws:",part.get_nextws())

                if(nextws in bigzone and part_notadded):
                    #skip transfer station and drop off directly at WS
                    print("putting part",part.get_ID(),"from small to big zone")
                    print("big zone:",bigQindex,"small zone:",smallQindex)
                    #need to get position from WS
                    smallposQ.put(self.WS_position("WS"+part.get_currentws()))
                    smallposQ.put(self.WS_position("WS"+part.get_nextws()))

                    part.set_transfer(False)
                    smallpartQ.put(part)

                    part_notadded = False #only do this once

                else:
                    #loop through big partQ and make sure parts are added back
                    smallpartQ.put(part)
                    smallposQ.put(pickuppt)
                    smallposQ.put(dropoffpt)

            #save each list of queues
            list_partQ[bigQindex] = bigpartQ
            list_posQ[bigQindex] = bigposQ
            list_partQ[smallQindex] = smallpartQ
            list_posQ[smallQindex] = smallposQ
            print("after adding and subtracting q")
            print("size of biq part q:", list_partQ[bigQindex].qsize())
            print("size of small part q:", list_partQ[smallQindex].qsize())
        #return the list of part then pos Q
        return list_partQ,list_posQ

    def zone_repair(self, currentzones, load_data):
        print("***** in zone repair *******")
        repeat = 3
        to_try = copy.deepcopy(currentzones)
        for x in range(repeat):
            proc = multiprocessing.Process(target=to_try.zone_reparation(load_data))
            proc.start()
            proc.join(timeout=300)
            # If thread is active
            if proc.is_alive():
                print("failed to find alternative zone")
                proc.terminate()
                to_try = copy.deepcopy(currentzones)
                continue
            
            print("found new zones!")
            print("new zones to_try:\n", to_try.phase2_ws())
            return to_try

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

        print("goal (x, y): (",x_pos,",",y_pos,")")
        
        #convert to meters
        return [x_pos, y_pos ,0 ,0 ,1 ,0]

    def action_sol(self,data):
        self.action_sol = data.data
    
    def get_action(self):
        return self.action_sol

#def main():
    #control = Solution_module()
    #rospy.init_node("solution_node") #create node
    #rospy.loginfo("solution node active")

    #start time
    #start_time = rospy.get_rostime()

    #while not rospy.is_shutdown():
        #rospy.sleep(0.1)
    
    
#if __name__ == "__main__":
    #main()
