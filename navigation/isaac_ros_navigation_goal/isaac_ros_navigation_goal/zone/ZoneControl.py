from .Phase_2 import Phase2
from .Phase_3 import Phase3
import pandas as pd
import numpy as np
import copy
#purpose of this is to provide a link bewtween zone creation and simulation
import multiprocessing as mp

class zone_control:
    def __init__(self):
        self.find_best_zone()


    def find_best_zone(self):

        #run phase 1 and 2
        self.test1 = Phase2()
        self.test1.start_GA(1)
        #self.test1.SA_improvement(1)
        #index=0
        #for x in self.test1.bestAdj_matrix:
            #filepath = '/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/adj'+str(index)+'.csv'
            #pd.DataFrame(x).to_csv(filepath,index=False)
            #index+=1

        #run phase 3 with phase 2 map
        self.test2 = Phase3(self.test1.opt_ws,self.test1.opt_cs,self.test1.bestAdj_matrix)
        self.test2.setting_transfer_stations()
        #self.test2.assigning_crit_seg()

    def zone_reparation(self, newload):
        
        #newload is a numpy array
        #self.test1.set_wsloads(newload)

        #self.test1.start_GA(1)
        try:
            test2 = Phase2()
            test2.set_wsloads(newload)
            test2.SA_improvement(1)
            print("found zones")
            self.test1 = copy.deepcopy(test2)
        except:
            print("error couldn't find zones")

        index=0
        for x in self.test1.bestAdj_matrix:
            filepath = '/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/adj'+str(index)+'.csv'
            pd.DataFrame(x).to_csv(filepath,index=False)
            index+=1

        #run phase 3 with phase 2 map
        self.test2 = Phase3(self.test1.opt_ws,self.test1.opt_cs,self.test1.bestAdj_matrix)
        self.test2.setting_transfer_stations()
        print("all done finding transferstations")
        #self.test2.assigning_crit_seg() #something keeps blocking in here
        #print("all done finding new zones")

        return

    def redo_phase3(self):
        #run phase 3 with phase 2 map
        print("redoing phase 3")
        print("phase 2 ws:",self.phase2_ws())
        print("phase 2 cs:",self.phase2_cs())
        self.test2 = Phase3(self.test1.opt_ws,self.test1.opt_cs,self.test1.bestAdj_matrix)
        self.test2.setting_transfer_stations()


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
    
    '''
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
    
    test3.zone_reparation(new_loads)
    '''
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
    
