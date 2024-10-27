from .Phase_2 import Phase2
from .Phase_3 import Phase3
import pandas as pd
import numpy as np
import copy
#purpose of this is to provide a link bewtween zone creation and simulation
import multiprocessing as mp
import os
import rospkg
from csv import writer, reader
import signal

class TimeoutExpired(Exception):
    pass

class zone_control:
    def __init__(self):
        self.find_best_zone()

    def find_best_zone(self):

        #run phase 1 and 2
        self.test1 = Phase2()
        #self.test1.start_GA(1)
        self.test1.SA_improvement(1)
        #bestws = [['WS4', 'WS1', 'WS2'], ['WS8', 'WS5', 'WS6', 'WS9', 'WS3'], ['WS11', 'WS15', 'WS14', 'WS18', 'WS17', 'WS16', 'WS12', 'WS7', 'WS13', 'WS10']]
        #bestcs =  [[['i', 'B', 'C', 'D', 'l'], ['j', 'B', 'i']], [['p', 'L', 'm'], ['n', 'N', 'M', 'L', 'm'], ['q', 'P', 'N', 'n'], ['k', 'E', 'I', 'M', 'L', 'p']], [['s', 'Y', 'Z', 'a', 'b', 'c', 'w'], ['s', 'Y', 'Z', 'a', 'b', 'v'], ['s', 'Y', 'Z', 'a', 'b', 'z'], ['v', 'b', 'a', 'y'], ['y', 'a', 'Z', 'x'], ['t', 'Z', 'a', 'y'], ['o', 'O', 'Q', 'X', 'W', 'v'], ['u', 'a', 'b', 'z'], ['r', 'Q', 'X', 'W', 'V', 'U', 't']]]

        #self.test1.set_wscs(bestws,bestcs)
        #run phase 3 with phase 2 map
        self.test2 = Phase3(self.test1.opt_ws,self.test1.opt_cs,self.test1.bestAdj_matrix)
        self.test2.setting_transfer_stations()
        #self.test2.assigning_crit_seg()

        #record zone design
        with open(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data','zonehistory.csv'), 'a') as file:
            writerobj = writer(file)
            writelist = [self.test1.get_SAws(),self.test1.get_SAcs(),self.test2.get_transferstations(),self.test1.get_lowestSVp()]
            writerobj.writerow(writelist)
            file.close()

    def zone_reparation(self, newload):
        #newload is a numpy array
        #self.test1.set_wsloads(newload)

        while(True):
            try:
                newT = Phase2()
                newT.set_wsloads(newload)
                #newT.start_GA(1)
                newT.SA_improvement(1)
                print("found zones")
                self.test1 = copy.deepcopy(newT)
                #run phase 3 with phase 2 map
                self.test2 = Phase3(self.test1.opt_ws,self.test1.opt_cs,self.test1.bestAdj_matrix)
                self.test2.setting_transfer_stations()
                print("all done finding transfer stations")
                ts = self.test2.get_transferstations()
                invalid = 0
                for set in ts:
                    if set == []:
                        invalid += 1
                if invalid >= self.test1.get_numzones()-1:
                    print("a zone is not connected")
                    continue
                break
            except:
                print("error couldn't find zones restarting")

        index=0
        #self.test2.assigning_crit_seg() #something keeps blocking in here
        #print("all done finding new zones")

        with open(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data','zonehistory.csv'), 'a') as file:
            writerobj = writer(file)
            writelist = [self.test1.get_SAws(),self.test2.get_zone(),self.test2.get_transferstations(),self.test1.get_lowestSVp()]
            writerobj.writerow(writelist)
            file.close()
        print("leaving zone redesign")
        return

    def redo_phase3(self):
        #run phase 3 with phase 2 map
        print("redoing phase 3")
        print("phase 2 ws:",self.phase2_ws())
        print("phase 2 cs:",self.phase2_cs())
        self.test2 = Phase3(self.test1.opt_ws,self.test1.opt_cs,self.test1.bestAdj_matrix)
        self.test2.setting_transfer_stations()

    def shortest_dist(self, i, j):
        #get shortest dist
        return self.test1.get_shortest_dist(i,j)

    #getters
    def phase1_ws(self):
        return self.test1.get_initialws()
    def phase1_cs(self):
        return self.test1.get_initialcs()
    def phase2_ws(self):
        return self.test1.get_SAws() #gets list of workstations in a zone
    def phase2_cs(self):
        return self.test1.get_SAcs() 
    def phase3_cs(self):
        return self.test2.get_zone() #returns list of critical segments in a zone
    def transfer_ws(self):
        return self.test2.get_transferstations()
    def get_SVp(self):
        return self.test1.get_lowestSVp()

if __name__ == '__main__':
    test3 = zone_control()
    print("phase1")
    print(test3.phase1_ws())
    print(test3.phase1_cs())
    print("phase2")
    print(test3.phase2_ws())
    print(test3.phase2_cs())
    print("phase3")
    print(test3.transfer_ws())
    #print(test3.phase3_cs())
    '''
    print("shortest dist",test3.shortest_dist("WS1","WS7"))
    
    print("lowest SVp:", test3.get_SVp())

    print("starting partially loaded test")

    new_loads = np.array([
        #1,2,3,4,5,6,7,8,9,10,11
        [0,0,0,0,0,0,5,0,0,0,0], #1
        [0,0,5,0,0,0,0,0,0,0,0], #2
        [0,0,0,0,0,0,0,0,2,0,0], #3
        [0,0,0,0,0,0,0,0,0,0,0], #4
        [0,0,0,3,0,0,4,0,0,0,0], #5
        [0,0,0,0,0,6,0,0,0,0,0], #6
        [0,0,0,0,0,0,0,2,0,0,0], #7
        [0,0,0,0,0,0,2,0,0,0,0], #8
        [0,0,5,0,0,0,0,0,0,4,0], #9
        [0,0,0,0,0,0,0,0,0,0,0], #10
        [0,0,0,0,0,0,0,10,0,0,0], #11
    ])

    new_loads = np.array(
    [[0., 0., 5., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [8., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 5., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 5., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]
    )




    [[0. 7. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
    [0. 0. 3. 1. 0. 0. 0. 0. 0. 0. 0.]
    [0. 0. 0. 0. 0. 0. 2. 0. 0. 0. 0.]
    [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
    [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
    [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
    [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
    [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
    [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
    [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
    [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]]
    '''
    '''
    test3.zone_reparation(new_loads)
    
    test4 = copy.deepcopy(test3)
    for x in range(3):
        ctx = mp.get_context('forkserver')
        proc = ctx.Process(target=test4.zone_reparation(new_loads))#, args = (new_loads,))
        proc.start()
        proc.join(timeout=180)
        # If thread is active
        if proc.is_alive():
            print("failed to find alternative zone")
            proc.terminate()
            test4 = copy.deepcopy(test3)
            continue
        
        print("found new zones!")
        print("new zones to_try:\n", test4.phase2_ws())
        break
    '''
    '''
    print("lowest SVp:", test3.get_SVp())
    print("phase1")
    print(test3.phase1_ws())
    #print(test3.phase1_cs())
    print("phase2")
    print(test3.phase2_ws())
    #print(test3.phase2_cs())
    print("phase3")
    print(test3.transfer_ws())
    #print(test3.phase3_cs())
    '''
    
