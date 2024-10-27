#!/usr/bin/env python
import pandas as pd
import rospy
import copy
from itertools import combinations 
from zone_SA.part import part
import threading
import os
import rospkg
import ctypes
import signal
import multiprocessing
import time
import sys

class TimeoutExpired(Exception):
    pass

class Solution_module():
    
    def __init__(self):
        self.num_zones = 0
        print("in solution SA")
        #self.workstation_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','Workstation_points.csv'), sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        self.workstation_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','Workstation_points.csv'), sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        self.workstation_loc = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','Workstation_Loaction.csv'), sep=',', header=0, names=['x','y'], encoding = 'utf-8')
    
    def load_sharing(self, part_list, current_ws):
        #look at parts that are waiting for the robot to pickup
            #the robots Q
            #these parts are waiting to be prcoessed
        #starting for the second part
            #if the part's next destination is in a zone that is not busy
            #add part to next zone robot Q(only if robot Q in next zone is 0 or 1)
            #if the next zone is busy have robot bypass the transfer station and deliver part to the next WS
        
        print("in load sharing")
        #get all possible combinations
        self.num_zones = len(current_ws)
        size_of_comb = []
        for x in range(0,self.num_zones): size_of_comb.append(x)
        all_comb = list(combinations(size_of_comb, 2))
        print("all comb\n",all_comb)
        #neighboring_zone example
        #[(0,1),(0,2),(1,2)]

        #first case when a part needs to droped off in a less busy zone
        for comb in all_comb:
            size_z1 = len(part_list[comb[0]])
            size_z2 = len(part_list[comb[1]])

            #which zone is more overloaded
            print("size of comb 0 q:", size_z1)
            print("size of comb 1 q:", size_z2)

            if size_z1 > size_z2: #if zone1 has a bigger part load than zone 2
                bigQindex = comb[0]
                smallQindex = comb[1]
                bigLsize = size_z1
                smallLsize = size_z2

            elif size_z1 < size_z2:
                bigQindex = comb[1]
                smallQindex = comb[0]
                bigLsize = size_z2
                smallLsize = size_z1

            else:
                #case for when the Qs are equal also for if both Queus are empty
                continue

            bigpartL = part_list[bigQindex]
            smallpartL = part_list[smallQindex]

            bigzone = current_ws[bigQindex]
            smallzone = current_ws[smallQindex]
            
            #for every part in big list see if next WS is in small list's zone
            #only do this once
            print("size of biq part list:", bigLsize)
            print("size of small part list:", smallLsize)
            
            for x in range(bigLsize):
                #print("looking at big q")
                part = bigpartL[x] #pop part off of the list
                del bigpartL[x]
                
                pickuppt = 'WS' + part.get_currentws() #this is where the part currently is
                dropoffpt = 'WS' + part.get_nextws() #this position is the end location
                
                #print("looking at part:",part.get_ID())
                #print("current ws:",pickuppt)
                #print("nextws:",dropoffpt)

                if(dropoffpt in smallzone):
                    #move part over to smallQ
                    print("putting part",part.get_ID(),"from big to small zone")
                    print("big zone:",bigQindex,"small zone:",smallQindex)

                    part.set_transfer(False)
                    part.set_tsthru(False)
                    smallpartL.append(part)

                    break #only do this once

                else:
                    #loop through big partQ and make sure parts are added back
                    bigpartL.append(part)


            #case for when a part needs to be transfered from small to big
            for x in range(smallLsize):
                #print("looking at small q")
                part = smallpartL[x]
                del smallpartL[x] #pop part off of list

                pickuppt = 'WS' + part.get_currentws() #this is where the part currently is
                dropoffpt = 'WS' + part.get_nextws() #this position is the end location

                #print("looking at part:",part.get_ID())
                #print("current ws:",part.get_currentws())
                #print("next ws:",part.get_nextws())

                if(dropoffpt in bigzone):
                    #skip transfer station and drop off directly at WS
                    print("putting part",part.get_ID(),"from small to big zone")
                    print("big zone:",bigQindex,"small zone:",smallQindex)

                    part.set_transfer(False)
                    part.set_tsthru(False)
                    smallpartL.append(part)

                    break #only do this once

                else:
                    #loop through big part list and make sure parts are added back
                    smallpartL.append(part)

            #save each list of queues
            part_list[bigQindex] = bigpartL
            part_list[smallQindex] = smallpartL
            print("after adding and subtracting q")
            print("size of biq part q:", len(part_list[bigQindex]))
            print("size of small part q:", len(part_list[smallQindex]))
        #return the list of part then pos Q
        return part_list

    def zone_repair(self, currentzones, load_data):
        print("***** in zone repair *******")
        repeat = 3
        to_try = copy.deepcopy(currentzones)
        for x in range(repeat):
            q = multiprocessing.Queue()
            proc = multiprocessing.Process(target=worker, args=(q, load_data,))
            proc.start()
            q.put(to_try)
            time.sleep(200)

            to_try = copy.deepcopy(q.get())
            q.close()
            if to_try == None:
                print("failed to find alternative zone")
                to_try = copy.deepcopy(currentzones)
                proc.kill()
                continue
            else:
                print("in loop ws:\n", to_try.phase2_ws())
                proc.kill()
                break

        #print("found new zones!")
        #print("new zones to_try:\n", to_try.phase2_ws())
        if to_try.phase2_ws() == []:
            print("returning to oginial zone design")
            return currentzones
        else:
            #currentzones = copy.deepcopy(to_try)
            return to_try

    def action_sol(self,data):
        self.action_sol = data.data

    def get_action(self):
        return self.action_sol
    
def worker(q, data):
    obj = q.get()
    q.put(None)
    obj.zone_reparation(data)
    noneobj = q.get()
    q.put(obj)
    print("worker all done")
    return