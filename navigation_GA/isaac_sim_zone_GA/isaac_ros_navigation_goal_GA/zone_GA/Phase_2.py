import numpy as np
import pandas as pd
import math
import copy
import random
from .Phase_1 import Phase1 #replace the Phase_1 with .Phase_1
from itertools import permutations, combinations
import matplotlib.pyplot as plt
from csv import writer
import time
import os
import rospkg
from matplotlib.animation import FuncAnimation
import concurrent.futures

class Phase2(Phase1):
    def __init__(self):
        self.Ti = 4.5
        self.Tf = 0.35
        #run phase 1
        self.phase1 = Phase1()
        self.phase1.create_map()
        self.phase1.start_phase1()

        self.P_initial_WS = self.phase1.GW #2D list
        self.P_initial_CS = self.phase1.GS #2D list
        self.workstation_points = self.phase1.workstation_points
        self.ws_loads = self.phase1.ws_loads #matix
        self.adj_matrix = self.phase1.adjacency_Mtx #the enter adj matix, only one, contains the distances to each neighboring node
        self.all_adj_matrix = self.phase1.all_adj_matrix #include adj matrix for all zones
        self.workstation_dist_mtx = self.phase1.workstation_dist_mtx #contains the distances between every workstation
        self.all_critical_points = self.phase1.critical_points
        self.num_of_zones = self.phase1.nz

        #for graphing
        self.ws_xy = self.phase1.workstation_loc
        

        self.Vel = self.phase1.V#could improve this by just passing it in
        self.tl = self.phase1.tl
        self.tu = self.phase1.tu
        #140 #550 #750 #1000
        #self.M = 250 #number of reductions 100 is good

        self.lowestSVp = 1
        self.all_path()

        self.all_points = self.all_critical_points.index.to_list()

        self.num_ws = len(self.workstation_points)

        #for phase 3
        df_ws_dist_mtx = pd.DataFrame(self.workstation_dist_mtx)
        df_ws_dist_mtx.to_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_GA'), 'isaac_ros_navigation_goal_GA/zone_GA/data/LE','WS_dist_mtx.csv'), index=False)
        self.adj_dist = self.set_adjdist() #distance to be considered adjacent
        print("adjacency dist:",self.adj_dist)
        #path,dist = self.phase1.shortest_dist('i','p',self.adj_matrix)
        #print(path)
        #print(dist)
        #print("phase 1 initial zone design")
        #print(self.P_initial_WS)
        #print(self.P_initial_CS)

    def start_GA(self,exetime):

        #self.kill_disp_thread = False
        #self.displaycs = copy.deepcopy(self.P_initial_CS)
        #pool = concurrent.futures.ThreadPoolExecutor(1)#add one for data recording
        #pool.submit(self.rundisplay)
        #pool.shutdown(wait=False)

        self.all_path()
        pop_size = 60
        gen_size = 100
        selection_size = 10
        mutation_rate = 0.45
        self.lowestSVp = 1

        ws_neighbors = self.WS_neighbors(self.adj_dist) #2D list

        #generate intial solution model
        #solset = initial_solws and initial_solcs
        initial_solws, initial_solcs = self.gen_solset(pop_size,self.adj_dist,ws_neighbors)      

        prev_bestcs = initial_solcs[0]
        prev_bestws = initial_solws[0]

        self.newgen_ws = [] #used to store the next generation
        self.newgen_cs = []

        SVpPlot = [] #graphing
        topSVps = [[] for k in range(5)]
        
        #Optsol = opt_ws and opt_cs
        self.opt_ws = []
        self.opt_cs = []
        
        #used to run GA for a certian amount of time
        exetime = exetime*60
        starttime = time.perf_counter()
        currenttime = 0

        #debugging purposes
        '''
        testws = [['WS9', 'WS8', 'WS5', 'WS1', 'WS6', 'WS13', 'WS7'], ['WS10','WS2', 'WS3', 'WS4', 'WS12'],['WS17', 'WS16', 'WS14', 'WS18', 'WS11', 'WS15']]
        testcs = [[['q', 'P', 'S', 'R', 'p'], ['p', 'L', 'm'], ['i','A','E','I','M','N','P','q'], ['n', 'N', 'M', 'L', 'p'], ['u', 'V', 'U', 'T', 'S', 'R', 'p'], ['o', 'n', 'N', 'M', 'L', 'm']],[['r', 'Q', 'O', 'J', 'G', 'F', 'j'], ['j', 'F', 'k'], ['r', 'Q', 'O', 'J', 'K', 'l'], ['t', 'U', 'V', 'W', 'X', 'Q', 'r']],[['y', 'a', 'Z', 'x'], ['v', 'b', 'a', 'Z', 'x'], ['v', 'b', 'z'], ['x', 'Z', 'Y', 's'], ['z', 'b', 'c', 'w']]]
        alladj_mtx = []
        for _ in range(self.num_of_zones):
            alladj_mtx.append(self.adj_matrix)
        test_adjmatrix = self.update_adj_matrixs(alladj_mtx, testcs, self.adj_matrix)
        copy_solws, copy_solcs = self.remove_tip_ws(testws, testcs, test_adjmatrix, 'WS10')
        '''
        #while(currenttime < exetime):
        gen =0
        while(gen < gen_size):
            currenttime = time.perf_counter() - starttime
            allSVp = 0

            #calculate fitness for every solution
            ranked_sol = {} #this holds indexs corresponding to a solution both ws and cs
            
            count = 0 #for debugging
            
            #Bestsol = very_bestws and very_bestcs
            very_bestws = []
            very_bestcs = []
            #BestSVp
            very_bestSVp = -1
            
            #Evaluate
            print("evaluate")
            for sol_index in range(len(initial_solws)):
                #print("evaluating solution:",count)
                ranked_sol[sol_index] = self.fitness(initial_solws[sol_index],initial_solcs[sol_index])
                allSVp += 1/ranked_sol[sol_index]
                if ((ranked_sol[sol_index] > very_bestSVp) and (ranked_sol[sol_index] != None)):
                    very_bestws = copy.deepcopy(initial_solws[sol_index])
                    very_bestcs = copy.deepcopy(initial_solcs[sol_index])
                    very_bestSVp = ranked_sol[sol_index]
                elif (ranked_sol[sol_index] == None or ranked_sol[sol_index] == 0):
                    del ranked_sol[sol_index]
                count += 1
            
            #for debugging
            if(self.countnum_ws(very_bestws) > 18): #for testing can delete later
                    print("error")
            
            print("\n*******best solution*******")
            print("\nSVp:", 1/(very_bestSVp))
            print("workstations:\n",very_bestws)
            print("crit segments:\n",very_bestcs,"\n")
            SVpPlot.append(allSVp/pop_size)
            #display zones
            self.displaycs = copy.deepcopy(very_bestcs)
            
            #Compare
            #if very best is better than opt then replace
            if ((1/very_bestSVp) <= self.lowestSVp and (very_bestSVp != 0)):
                if 1/very_bestSVp == self.lowestSVp:
                    numtip1 = 0
                    numtip2 = 0
                    for zone in range(0,self.num_of_zones):
                        numtip1 += len(self.finding_tip_ws(very_bestws[zone],very_bestcs[zone]))
                        numtip2 += len(self.finding_tip_ws(self.opt_ws[zone],self.opt_cs[zone]))
                    if numtip1 > numtip2:
                        self.lowestSVp = 1/very_bestSVp
                        self.opt_ws = copy.deepcopy(very_bestws)
                        self.opt_cs = copy.deepcopy(very_bestcs)
                else:
                    self.lowestSVp = 1/very_bestSVp
                    self.opt_ws = copy.deepcopy(very_bestws)
                    self.opt_cs = copy.deepcopy(very_bestcs)

            #Sort
            #ranked_sol = {1: 0.2, 2: 0.8, 3: 0.05 ...}
            best_ws = []
            best_cs = []
            print("sort")
            ranked_sol = dict(sorted(ranked_sol.items(), key=lambda item: item[1], reverse=True))

            #graphing
            count = 0
            for index in ranked_sol:
                if count > selection_size: 
                    break
                #print(index, "->", ranked_sol[index])
                best_ws.append(copy.deepcopy(initial_solws[index]))
                best_cs.append(copy.deepcopy(initial_solcs[index]))
                if count<5:
                    topSVps[count].append(1/ranked_sol[index])
                count += 1

            #Create a mateing pool
            poolws = []
            poolcs = []
            print("create mating pool")
            for i in range(selection_size):
                zonesws = best_ws[i]
                zonecs = best_cs[i]
                for z in range(len(zonesws)):
                    poolws.append(copy.deepcopy(zonesws[z]))
                    poolcs.append(copy.deepcopy(zonecs[z]))

            #add prevoius best solution to pool
            for z in range(len(zonesws)):
                    poolws.append(copy.deepcopy(prev_bestws[z]))
                    poolcs.append(copy.deepcopy(prev_bestcs[z]))

            #update prev best ws and cs
            prev_bestcs = copy.deepcopy(self.opt_cs)
            prev_bestws = copy.deepcopy(self.opt_ws)

            #Crossover
            print("crossover")
            p = 0
            while p<pop_size:
                solws = []
                solcs = []
                
                #create adj matrix for solution
                alladj_mtx = []
                for _ in range(self.num_of_zones):
                    alladj_mtx.append(self.adj_matrix)

                reset_count = 0
                error = 0
                #add other zone that fit
                while(len(solws) != self.num_of_zones):
                    if reset_count > pop_size: #reset if this combination of zones isnt matching with any other
                        solcs = []
                        solws = []
                        reset_count = 0
                        error += 1
                    
                    if error == 10000:
                        try:
                            raise Exception('no possible zone combinations')
                        except Exception as inst:
                            print(inst.args)
                            raise

                    if len(solws) == 0:
                        zonei = random.randint(0,len(poolws)-1)
                        solws.append(poolws[zonei])
                        solcs.append(poolcs[zonei])
                        wscount = len(solws[0])

                    #ensure that no ws is in the same zone
                    zonei = random.randint(0,len(poolws)-1)
                    zonews = poolws[zonei]
                    zonecs = poolcs[zonei]

                    ws_inprev = False
                    for new_ws in zonews:
                        for prev_zone in solws:
                            if new_ws in prev_zone:
                                ws_inprev = True
                                break
                        if ws_inprev:
                            break
                    if (ws_inprev):
                        reset_count += 1
                        continue
                    else:
                        solws.append(zonews)
                        solcs.append(zonecs)
                        if not self.valid_cs(solcs):
                            solws.remove(zonews)
                            solcs.remove(zonecs)
                            reset_count += 1
                            continue
                        wscount += len(zonews)
                
                if not self.valid_cs(solcs):
                    print("not valid zone")                

                #check for compatibilty
                #at this stage solws contains a set of workstations that could or not be complete
                #modification stage could make incomplete sol complete by adding missing ws
                #if zones are incomplete add missing WS
                if wscount < self.num_ws:
                    alladj_mtx = self.update_adj_matrixs(alladj_mtx, solcs, self.adj_matrix)
                    #find missing ws
                    ws_list = self.workstation_points.loc[:,'workstation'].to_list()
                    missingws = []
                    for ws in ws_list:
                        ws_inzone = False
                        for zone in solws:
                            if ws in zone:
                                ws_inzone = True
                                break
                        if ws_inzone:
                            continue
                        else:
                            missingws.append(ws)
                    #print("missing ws:\n",missingws)
                    
                    #append missing ws to zones
                    while(len(missingws) > 0):
                        ws1 = random.choice(missingws) #chose a random ws 
                        #find neighbors of that ws (a little faster) #might lead to issues if no neighbors can connect
                        wsneigh = ws_neighbors[self.ws_to_index(ws1)]
                        wsneigh = [i for i in wsneigh if i not in missingws] #find nighbors that are not already missing
                        if(len(wsneigh) < 1): #if there are no available neighbors restart
                            continue
                        ws2 = random.choice(wsneigh) #note that ws2 may not be a tip ws

                        #find zone that ws2 is in
                        for z in range(self.num_of_zones): 
                            if ws2 in solws[z]:
                                ws2z = z
                                break
                        
                        crit1 = self.ws_crit_point(ws1)
                        crit2 = self.ws_crit_point(ws2)
                        path, dist = self.phase1.shortest_dist(crit1,crit2,alladj_mtx[ws2z])
                        if path == None or dist > self.adj_dist:
                            continue

                        missingws.remove(ws1)
                        solws[ws2z].append(ws1)
                        solcs[ws2z].append(path)
                        #update adj matrix
                        alladj_mtx = self.update_adj_matrixs(alladj_mtx, solcs, self.adj_matrix)

                if not self.valid_cs(solcs):
                    print("not valid zone")


                #Zones are complete and every WS is assigned to a zone
                #Mutate
                compare = 0
                reset_count = 0
                while compare < mutation_rate: #compare has a %_ chance of mutating
                    if reset_count > 500: #used for when a tip ws cant conect to another tip ws
                        break
                    #move a certain number of ws from a zone to another
                    #find tip workstations
                    all_tip_ws = []
                    for zone in range(0,self.num_of_zones):
                        all_tip_ws.append(self.finding_tip_ws(solws[zone],solcs[zone]))
                    
                    #select a random zone and corresponding random tip ws
                    zonei1 = random.randint(0,self.num_of_zones-1)
                    if len(solws[zonei1]) < 2:
                        #if there is only one ws in that zone restart
                        continue
                    
                    tipws = random.choice(all_tip_ws[zonei1])

                    #find tip ws that are neighbors with other zone
                    #special case for when there is only one ws in zone*****
                    tipws_neigh = ws_neighbors[self.ws_to_index(tipws)]
                    tipws_neigh = [i for i in tipws_neigh if i not in solws[zonei1]] #find nighbors that are not in same zone

                    #special case for when tip ws has no neighbors that are in the same zone
                    if len(tipws_neigh) == 0:
                        continue #find new tipws

                    #disconnect one tipws and add to another zone
                    rand_neigh = random.choice(tipws_neigh)
                    zonei2 = -1
                    for z in range(self.num_of_zones): #find zone that rand_neigh is in
                        if rand_neigh in solws[z]:
                            zonei2 = z
                            break
                    
                    cws = copy.deepcopy(solws)
                    ccs = copy.deepcopy(solcs)
                    cadj = copy.deepcopy(alladj_mtx)
                    if not self.valid_cs(solcs):
                        print("not valid zone")
                    
                    copy_solws, copy_solcs = self.remove_tip_ws(cws, ccs, cadj, tipws)
                    
                    if not self.valid_cs(copy_solcs):
                        print("not valid zone")

                    #update adj matrix
                    alladj_mtx = self.update_adj_matrixs(cadj, copy_solcs, self.adj_matrix)

                    #find path and check
                    crit1 = self.ws_crit_point(tipws)
                    crit2 = self.ws_crit_point(rand_neigh)
                    path, dist = self.phase1.shortest_dist(crit1,crit2,cadj[zonei2])
                    if path == None or dist > self.adj_dist:
                        #restart
                        #solws[zonei1].append(tipws)
                        #solcs[zonei1].append(tipws_cs)
                        reset_count += 1
                    else:
                        solws = copy.deepcopy(copy_solws)
                        solcs = copy.deepcopy(copy_solcs)
                        alladj_mtx = copy.deepcopy(cadj)
                        solws[zonei2].append(tipws)
                        solcs[zonei2].append(path)
                        reset_count += 1

                    if not self.valid_cs(solcs):
                        print("not valid zone")

                    #update adj matrixs
                    alladj_mtx = self.update_adj_matrixs(alladj_mtx, solcs, self.adj_matrix)
                    
                    #see if we can mutate again
                    compare = random.uniform(0,1)

                if reset_count > 500:
                    continue

                if(self.countnum_ws(solws) > 18): #for testing can delete later
                    print("error")
                
                self.newgen_ws.append(copy.deepcopy(solws))
                self.newgen_cs.append(copy.deepcopy(solcs))
                solws.clear()
                solcs.clear()

                p += 1
            
            initial_solws.clear()
            initial_solcs.clear()
            initial_solws = copy.deepcopy(self.newgen_ws)
            initial_solcs = copy.deepcopy(self.newgen_cs)
            self.newgen_ws.clear()
            self.newgen_cs.clear()
            gen += 1
        
        print("\nbest SVp:", self.lowestSVp)
        print("best ws:\n",self.opt_ws)
        print("best cs:\n", self.opt_cs)
        #self.opt_ws = [['WS11', 'WS10', 'WS8'], ['WS3', 'WS1', 'WS2', 'WS5'], ['WS4', 'WS9', 'WS7', 'WS6']]
        #self.opt_cs = [[['a', 'Z', 'Y', 'b'], ['d', 'e', 'f', 'g', 'b']], [['P', 'Q', 'H', 'D', 'E'], ['E', 'D', 'H', 'I'], ['S', 'R', 'Q', 'H', 'I']], [['K', 'M', 'N', 'W', 'c'], ['U', 'V', 'W', 'c'], ['X', 'L', 'M', 'K']]]
        self.bestAdj_matrix = self.update_adj_matrixs(self.all_adj_matrix,self.opt_cs,self.adj_matrix)

        numtip = 0
        for zone in range(0,self.num_of_zones):
            numtip += len(self.finding_tip_ws(very_bestws[zone],very_bestcs[zone]))
        print("number of tip ws: ", numtip)
        self.num_tip = numtip
        
        #plt.subplot(1, 2 , 1)
        #plt.plot(topSVps[0], label="1")
        #plt.plot(topSVps[1], label="2")
        #plt.plot(topSVps[2], label="3")
        #plt.plot(topSVps[3], label="4")
        #plt.plot(topSVps[4], label="5")
        #plt.title('Top 5 SVp')

        #plt.subplot(1, 2, 2)
        #plt.plot(SVpPlot, label="average SVp")
        #plt.title('Average SVp')
        #plt.legend()
        
        #plt.show()

        self.kill_disp_thread = True

    def gen_solset(self,num_pop,neigh_range, ws_neigh):
        #this function generates an inital population set
        #numpop is the amount in intial solution set
        currentmap = copy.deepcopy(self.adj_matrix)
        solution_setws = []
        solution_setcs = []

        for _ in range(num_pop):
            print("solution num",_)
            #all_adjmatrix = copy.deepcopy(self.all_adj_matrix) #contains several adj matrixs
            ws_neighbors = copy.deepcopy(ws_neigh)
            all_ws = self.workstation_points.loc[:,'workstation'].to_list()
            solution_ws = []
            solution_cs = []
            all_adjmatrix = []

            #create inital seed ws
            for z in range(self.num_of_zones):
                seed = random.choice(all_ws)
                all_ws.remove(seed)
                solution_ws.append([seed])
                solution_cs.append([])
                all_adjmatrix.append(currentmap)
                #remove seeds from neighbor list
                for x in range(self.num_ws):
                    if seed in ws_neighbors[x]:
                        ws_neighbors[x].remove(seed)

            while len(all_ws) != 0:
                #select a random zome
                zoneI = random.randint(0,self.num_of_zones-1)
                zone = solution_ws[zoneI]
                #select a random ws in that zone
                ws = random.choice(zone)
                wsi = self.ws_to_index(ws)
                
                #from that ws find neibors and try to connect to a random one
                neigh = ws_neighbors[wsi]
                if neigh == []: #no remaining neighbors
                    continue
                rand_neigh = random.choice(neigh)
                #print("connecting",ws, "to", rand_neigh)

                crit1 = self.ws_crit_point(ws)
                crit2 = self.ws_crit_point(rand_neigh)

                path, dist = self.phase1.shortest_dist(crit1,crit2,all_adjmatrix[zoneI])
                if path == None or dist > neigh_range:
                    if path == None:
                        continue
                    continue
                
                #update adj matrix and cs 
                solution_ws[zoneI].append(rand_neigh)
                solution_cs[zoneI].append(path)
                all_adjmatrix = self.update_adj_matrixs(all_adjmatrix, solution_cs, currentmap)

                #remove ws from lists of neighbors
                for x in range(self.num_ws):
                    if rand_neigh in ws_neighbors[x]:
                        ws_neighbors[x].remove(rand_neigh)

                all_ws.remove(rand_neigh)

                #print(solution_ws)
                #print(solution_cs)

            solution_setws.append(solution_ws)
            solution_setcs.append(solution_cs)

        return solution_setws, solution_setcs
            
    def WS_neighbors(self,adj_dist):
        #adj_dist is the adjacency distance that a WS can have
        #neighbor_mtx = np.zeros([self.num_ws,self.num_ws])
        neighbors = []
        for ws in range(self.num_ws):
            ws_neigh = []
            for neigh in range(self.num_ws):
                dist = self.workstation_dist_mtx[ws,neigh]
                if(dist < adj_dist and dist != 0):
                    ws_str = 'WS' + str(neigh+1)
                    ws_neigh.append(ws_str)
            neighbors.append(ws_neigh)
        
        return neighbors

    def fitness(self, solsetws, solsetcs):
        all_tipws = []

        for zone in range(len(solsetws)):
            all_tipws.append(self.finding_tip_ws(solsetws[zone],solsetcs[zone]))

        SVp = self.calc_SVp(solsetws,all_tipws)
        if ((SVp == None) or (SVp == 0)):
            SVp = 1

        #want the smallest number to have the biggest value
        return 1/SVp

    def countnum_ws(self,zonesws):
        count = 0
        for x in zonesws:
            count += len(x)
        return count

    def SA_improvement(self,exetime):
        print("SA stuff")
        self.all_path()
        self.lowestSVp = 1
        self.opt_ws = []
        self.opt_cs = []

        #c = iteration counter
        c=0
        cmax = 500
        self.M = 500

        #current design
        pc_cs = copy.deepcopy(self.P_initial_CS)
        pc_ws = copy.deepcopy(self.P_initial_WS)
        self.opt_cs = copy.deepcopy(pc_cs)
        self.opt_ws = copy.deepcopy(pc_ws)

        #remeber T is the temperature!
        T = self.Ti

        #to make selection of alpha and beta faster
        comb = permutations([0, 1, 2], 2)
        comb = list(comb)

        Lpz = []
        self.bestLpz = [0,0,0]
        num_zones = self.num_of_zones
        iterator = len(comb) - 1
        possible_comb = copy.deepcopy(comb)
        rand_zones_2 = 0
        
        #for plotting
        SVd_plot = []
        temp_plot =[]

        self.lowestSVp = math.inf

        all_tip_ws = [[] for i in range(num_zones)]
        new_all_tip_ws = [[] for i in range(num_zones)]
        
        exetime = exetime*60
        starttime = time.perf_counter()
        currenttime = 0
        #while(currenttime < exetime):
            #currenttime = time.perf_counter() - starttime
        while(c<cmax):
            neigh_dist = self.adj_dist
            Lpz.clear()
            #all_tip_ws = [[] for i in range(num_zones)]
            #all_tip_ws = []
            #calculate the loads and tip of all zones 
            for zone in range(0,num_zones):
                all_tip_ws[zone].append(self.finding_tip_ws(pc_ws[zone],pc_cs[zone]))

            for zone in range(0,num_zones):
                Lpz.append(self.zone_load(pc_ws[zone], all_tip_ws[zone]))

            #randomly select two zones
            try:
                rand_zones = random.randint(0,len(possible_comb)-1)
            except:
                #neigh_dist += 150
                possible_comb = copy.deepcopy(comb)
                rand_zones = random.randint(0,len(possible_comb)-1)

            alpha, beta = possible_comb[rand_zones]
            #del possible_comb[rand_zones]
            #if rand_zones == 1:
                #print("here")
            
            #alpha 
            alpha_L = Lpz[alpha]
            beta_L = Lpz[beta]

            #rand_zones_2 = rand_zones #need to define this outside of while loop
            wrong_beta = False
            
            iterator = len(possible_comb) - 1
            #find a combination of zones where Lpz alpha > Lpz beta
            while beta_L >= alpha_L: 
                wrong_beta = True
                try:
                    rand_zones_2 = random.randint(0,iterator)
                except:
                    #neigh_dist += 150
                    possible_comb = copy.deepcopy(comb)
                    iterator = len(possible_comb) - 1
                    continue

                alpha, beta = possible_comb[rand_zones_2]

                del possible_comb[rand_zones_2]
                iterator -= 1

                alpha_L = Lpz[alpha]
                beta_L = Lpz[beta]

            #From the tip workstations of alpha, identify those that are adjacent to the tip workstations beta
            #store in WS(alpha,beta)
            #step 2 finding adjacent tip workstations 
            alpha_tip = self.finding_tip_ws(pc_ws[alpha],pc_cs[alpha])
            beta_tip = self.finding_tip_ws(pc_ws[beta],pc_cs[beta])
            
            #adjacent only if ws if <200 = self.adj_dist
            WSa_b = []
            list_mdprime={}

            unionmtx = self.union_matrixs(pc_cs,self.adj_matrix,alpha,beta)
            for wsa in alpha_tip:
                crita = self.ws_crit_point(wsa)
                for wsb in beta_tip:
                    critb = self.ws_crit_point(wsb)
                    path, dist = self.phase1.shortest_dist(crita,critb,unionmtx)
                    if dist<neigh_dist:
                        WSa_b.append(wsa)
                        list_mdprime[wsa] = wsb
            
            if(not WSa_b): #if there is not adjacency 
                print("no adjacency")
                #dont use this combination
                if not wrong_beta:
                    del possible_comb[rand_zones]
                continue
            
            #step 3
            rndint = random.randint(0,len(WSa_b)-1)
            m = WSa_b[rndint]
            crit_tip = self.ws_crit_point(m)
            CCS_alpha = copy.deepcopy(pc_cs[alpha])
            CCS_beta = copy.deepcopy(pc_cs[beta])
            mprime = ''

            mdprime = list_mdprime[m]
            
            #step 4 generate a new zone partition design
            #4.1 disconnect m from CCS_alpha

            #copy of all adj matrix
            temp_all_adj_matrix = copy.deepcopy(self.all_adj_matrix)

            #temp_cs = copy.deepcopy(pc_cs)
            tocontinue = False
            for zone in pc_ws:
                if m in zone:
                    if len(zone) < 3:
                        print("len of zone to remove less than 3 WS")
                        tocontinue = True
                        break
            if tocontinue:
                c += 1
                continue
        
            temp_ws, temp_cs = self.remove_tip_ws(copy.deepcopy(pc_ws),copy.deepcopy(pc_cs),copy.deepcopy(temp_all_adj_matrix),m)
            '''
            for segment in CCS_alpha:
                if crit_tip in segment:
                    if segment.index(crit_tip) == 0:
                        mprime = self.crit_point_ws(segment[len(segment)-1])
                    else:
                        mprime = self.crit_point_ws(segment[0])

                    CCS_alpha.remove(segment)
                    mprime_crit_tip = self.ws_crit_point(mprime)
                    segment.remove(mprime_crit_tip)#alpha still keeps mprime

                    #remove path from adj matrix in alpha
                    temp_all_adj_matrix[alpha] = self.adj_remove_seg(segment,temp_all_adj_matrix[alpha])
                    temp_cs[alpha] = CCS_alpha

                    temp_all_adj_matrix = self.update_adj_matrixs(temp_all_adj_matrix, temp_cs, self.adj_matrix)
            '''
            temp_all_adj_matrix = self.update_adj_matrixs(temp_all_adj_matrix,temp_cs,self.adj_matrix)
            #4.2 connect m and mdprime with shortest fesiable path
            path,dist = self.phase1.shortest_dist(self.ws_crit_point(mdprime), crit_tip, temp_all_adj_matrix[beta])
            #CCS_beta.append(path)

            if path == None or dist > neigh_dist: #check
                #print(temp_cs)
                #print(self.ws_crit_point(mdprime), " to ", crit_tip)
                print("no fesibale path")
                possible_comb = copy.deepcopy(comb)
                c +=1 #this is done for when the the current zone design is stuck and to avoid a infinite loop
                continue
            
            temp_cs[beta].append(path)
            temp_ws[beta].append(m)
            temp_all_adj_matrix = self.update_adj_matrixs(temp_all_adj_matrix,temp_cs,self.adj_matrix)

            #temp_all_adj_matrix[beta] = self.update_adj_add_seg(path,temp_all_adj_matrix[beta]) #add path to beta adj matrix
            #for x in range(0,num_zones): #remove path from others
                #if (x != beta):
                    #temp_all_adj_matrix[x] = self.adj_remove_seg(path,temp_all_adj_matrix[x])

            #for testing 
            #df_adj1 = pd.DataFrame(temp_all_adj_matrix[0])
            #df_adj2 = pd.DataFrame(temp_all_adj_matrix[1])
            #df_adj3 = pd.DataFrame(temp_all_adj_matrix[2])
        
            #df_adj1.to_csv(r'C:/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal_GA/isaac_ros_navigation_goal_GA/zone_GA/data/adj0.csv', header=self.all_points, index=self.all_points)
            #df_adj2.to_csv(r'C:/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal_GA/isaac_ros_navigation_goal_GA/zone_GA/data/adj1.csv', header=self.all_points, index=self.all_points)
            #df_adj3.to_csv(r'C:/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal_GA/isaac_ros_navigation_goal_GA/zone_GA/data/adj2.csv', header=self.all_points, index=self.all_points)

            #integate the new CCS_alpha and Beta into a whole crit seg and ws
            new_cs = copy.deepcopy(temp_cs)
            new_ws = copy.deepcopy(temp_ws)

            #new_cs[alpha] = CCS_alpha
            #new_cs[beta] = CCS_beta

            #new_ws[alpha].remove(m)
            #new_ws[beta].append(m)
            
            #if the new_ws contains less than two workstations not iterate
            if len(new_ws[alpha])<2:
                #print(new_ws)
                #reset possible combinations
                possible_comb = copy.deepcopy(comb)
                c +=1 
                continue

            for zone in range(0,num_zones):
                #Lpz.append(self.zone_load(pc_cs[zone],pc_ws[zone]))
                new_all_tip_ws[zone].append(self.finding_tip_ws(pc_ws[zone],pc_cs[zone]))

            #step 5
            # vd = SVp(c)-SVp0 
            new_SVp = self.calc_SVp(new_ws, new_all_tip_ws)
            prev_SVp = self.calc_SVp(pc_ws, all_tip_ws)

            vd = prev_SVp - new_SVp

            #accept if SVp is greater than zero or new SVp is lower than current SVp
            if (vd >= 0):
                pc_cs.clear()
                pc_ws.clear()
                pc_cs = copy.deepcopy(new_cs)
                pc_ws = copy.deepcopy(new_ws)
            else:
                r = random.random()
                mathy = math.exp(vd/T)
                temp_plot.append(mathy)
                if (r <= mathy):    #use temperature here
                    pc_cs.clear()
                    pc_ws.clear()
                    pc_cs = copy.deepcopy(new_cs)
                    pc_ws = copy.deepcopy(new_ws)
                else:
                    #keep orginal adj matrixs
                    print("*****same*****")
                    print("SVp diff:",vd)
                    print("temperature:",T)
                    possible_comb = copy.deepcopy(comb)
                    continue #return to step 2
            
            #update all adj matixs
            self.all_adj_matrix = copy.deepcopy(temp_all_adj_matrix)

            #reset possible combinations
            possible_comb = copy.deepcopy(comb)
            
            #increment c
            c += 1

            #decrease temperature
            if T > self.Tf: #if it hits the limit
                T = self.Ti * ((self.Tf/self.Ti)**(1/self.M))**c
            else:
                T= self.Tf
            
            
            if(new_SVp < self.lowestSVp):
                self.lowestSVp = new_SVp
                self.opt_cs = copy.deepcopy(pc_cs)
                self.opt_ws = copy.deepcopy(pc_ws)
                self.num_tip = 0
                for zone in range(0,num_zones):
                    self.num_tip += len(self.finding_tip_ws(pc_ws[zone],pc_cs[zone]))
                SVd_plot.append(self.lowestSVp)
                for zone in range(0,num_zones):
                    self.bestLpz[zone] = self.zone_load(pc_ws[zone], new_all_tip_ws[zone])
                self.bestAdj_matrix = copy.deepcopy(self.all_adj_matrix)
            
            #print(pc_cs)
            #print(pc_ws)
            print(c)
            
        #end of while loop
        print("lowest SVp")
        print(self.lowestSVp)
        print("best crit segments")
        print(self.opt_cs)
        print("best WS")
        print(self.opt_ws)
        print(self.bestLpz)
        
        '''
        plt.plot(SVd_plot)
        plt.ylabel('SVd_plot')
        plt.show()
        
        plt.plot(temp_plot)
        plt.ylabel('temp_plot')
        
        plt.show()
        '''
        return
        
    def set_adjdist(self):
        threesum = 0
        for row in self.workstation_dist_mtx:
            rowc = copy.deepcopy(row)
            rowc.sort()
            threesum = sum(rowc[1:17],threesum)
        return threesum/(self.num_ws * (self.num_ws-1))
        
    def add_cs(self, crit_seg, path):
        for point in path:
                if point not in crit_seg:
                    crit_seg.append(point)
        return crit_seg

    def calc_SVp(self ,all_ws, all_tip):
        TZLDp = 0
        Lpz = []
        #first calulate Lpz for every zone
        for zone in range(0,self.num_of_zones):
            Lpz.append(self.zone_load(all_ws[zone], all_tip[zone]))

        for zprime in range(0,self.num_of_zones-1):
            for zdprime in range(zprime+1,self.num_of_zones):
                TZLDp += abs(Lpz[zprime] - Lpz[zdprime])

        SVp = TZLDp/(sum(Lpz) * (self.num_of_zones-1))

        return(SVp)

    def adj_remove_seg(self, path, adj_matrixs):

        all_points = self.all_critical_points.index.to_list()
        for pt in path:
                index_of_pt = all_points.index(pt)
                #have to remove the point from being used 
                adj_matrixs[index_of_pt, :] = 0
                adj_matrixs[ :, index_of_pt] = 0

        return adj_matrixs

    def update_adj_matrixs(self, all_adj_matrixs, crit_segments, map):

        for x in range(len(all_adj_matrixs)):
            #make adj matrix the same as map to start
            all_adj_matrixs[x] = copy.deepcopy(map)
            
            #now remove every point in every other crit segments
            for i in range(len(crit_segments)):
                if i != x:
                    for path in crit_segments[i]:
                            all_adj_matrixs[x] = self.adj_remove_seg(path,all_adj_matrixs[x])
        """
        #just rebuild adjacency matrixes 
        for pt_x in range(0,len(path)-1):
                first_pt = path[pt_x]
                second_pt = path[pt_x+1]
                index_first = all_points.index(first_pt)
                index_second = all_points.index(second_pt)
                #make point available to only the current zone
                #first add segment
                adj_matrixs[index_first, index_second] = copy.deepcopy(self.adj_matrix[index_first, index_second])
                adj_matrixs[index_second, index_first] = copy.deepcopy(self.adj_matrix[index_second, index_first])

                #identify those points that belong to other zones
                #make point available to other points in zone
                comp = adj_matrixs.sum(axis=0)
                for i in range(0, len(comp)):
                    if comp[i] != 0:
                        adj_matrixs[index_first, i] = copy.deepcopy(self.adj_matrix[index_first, i])
                        adj_matrixs[i, index_first] = copy.deepcopy(self.adj_matrix[i, index_first])
                        adj_matrixs[i, index_second] = copy.deepcopy(self.adj_matrix[i, index_second])
                        adj_matrixs[index_second, i] = copy.deepcopy(self.adj_matrix[index_second, i])
        """
        return all_adj_matrixs

    def zone_load(self, ws_in_zone, zone_tip):
        #equations 11-14
        #finding Lpz
        #to find Lpz you need to find fpzij with is the load specifically in the zone
        #print("finding Lpz")
        self.fpzijD = {}

        all_fpzij = self.finding_zone_all_fpzij(ws_in_zone, zone_tip)

        sum_DA_DB = 0

        #finding get gpzij
        for i in ws_in_zone:
            for j in ws_in_zone:
                if (i != j):
                    gpzij = self.finding_gpzij(i, j, ws_in_zone, all_fpzij, zone_tip)
                    
                    index_i = self.ws_to_index(i)
                    index_j = self.ws_to_index(j)
                    dpzij = self.workstation_dist_mtx[index_i,index_j]
                    DApzij = gpzij * dpzij

                    fpzij = self.fpzijD[i + ',' + j]
                    DBpzij = fpzij*dpzij

                    sum_DA_DB += DApzij + DBpzij
        
        Lpz = (sum_DA_DB/self.Vel) + (all_fpzij*(self.tu + self.tl))
        self.fpzijD.clear()
        return Lpz

    def finding_gpzij(self,i,j,ws_in_zone, all_fpzij, zone_tip):
        #finding fpzki and fpzjk
        fpzki = 0
        fpzjk = 0
        for k in ws_in_zone:
            if(k!=i):
                fpzki += self.fpzijD[k + ',' + i]
            if(k!=j):
                fpzjk += self.fpzijD[j + ',' + k]
        return (fpzki*fpzjk)/all_fpzij

    def finding_zone_all_fpzij(self,ws_in_zone, zone_tip):
        all_fpzij = 1
        for i in ws_in_zone:
            for j in ws_in_zone:
                if(i != j):
                    fpzij = self.finding_fpzij(i,j, zone_tip)
                    all_fpzij += fpzij
                    self.fpzijD[i + ',' + j] = fpzij
        return all_fpzij
            
    def finding_tip_ws(self, zone_workstations, zone_crit_segments):
        #crit_segments is a 2D list
        #zone_workstaions is a list
        #all points is a dataframe
        workstations = self.workstation_points.loc[:,'workstation'].to_list()
        crit_workstations = self.workstation_points.loc[:,'critical_points'].to_list()

        tip_ws = []
        adj_crit_points = []

        for node in zone_workstations:
            adj_crit_points.clear()
            
            #node is "WS1" .. 
            ws_crit_point = crit_workstations[workstations.index(node)]
            tipcheck = 0
            #find adj crit points, a tip workstation is one that only has one branch connecting to it
            
            row = self.adj_matrix[self.crit_point_index(ws_crit_point)]
            for x in range(len(row)):
                if row[x]!=0:
                    adj_crit_points.append(self.index_crit_point(x))

            for point in adj_crit_points:
                for segment in zone_crit_segments:
                    if point in segment and ws_crit_point in segment:
                        tipcheck += 1
                        break
            """
            for segment in zone_crit_segments:
                if ws_crit_point in segment:
                    tipcheck +=1
            """

            if(tipcheck == 1):
                tip_ws.append(node)

        #special case for when there is only one WS in zone
        if len(zone_workstations) == 1:
            tip_ws.append(zone_workstations[0])

        return tip_ws

    def remove_tip_ws(self, zonews, zonecs, all_adjMatrix, tipws):
        #best ws:
        #[['WS8', 'WS9', 'WS4', 'WS3'], ['WS5', 'WS2', 'WS1', 'WS7'], ['WS11', 'WS6', 'WS10']]
        #best cs:
        #[[['d', 'V', 'W', 'c'], ['c', 'W', 'N', 'M', 'K'], ['P', 'O', 'J', 'K']], [['S', 'R', 'Q', 'H', 'I'], ['E', 'D', 'H', 'I'], ['U', 'T', 'S']], [['X', 'Y', 'Z', 'a'], ['a', 'Z', 'Y', 'b']]]
        #find zone that zone cs is in
        for z in range(self.num_of_zones):
            if tipws in zonews[z]:
                zonei = z
                break
        #find the crit pint that tip ws is at
        crit_tip = self.ws_crit_point(tipws)

        #find workstations that are connected to tip ws
        connectedws = []
        for segment in zonecs[zonei]:
            if crit_tip in segment:
                criti = segment.index(crit_tip)
                if criti == 0:
                    connectedws.append(self.crit_point_ws(segment[len(segment)-1]))
                else:
                    connectedws.append(self.crit_point_ws(segment[0]))
        
        #special case for when tipws is only connected to one ws
        #remove tipws and cs and update adj matrix
        if len(connectedws) == 1:
            for segment in zonecs[zonei]:
                if crit_tip in segment:
                    zonecs[zonei].remove(segment)
                    zonews[zonei].remove(tipws)

                    #remove path from adj matrix in alpha
                    all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, zonecs, self.adj_matrix)
        
        #case for when there are multiple ws connected to tipws
        if len(connectedws) > 1:
            zonews[zonei].remove(tipws)
            new_zonecs = copy.deepcopy(zonecs)
            #remove all segemnts that are connected to tipws
            for segment in zonecs[zonei]:
                if (crit_tip in segment):
                    new_zonecs[zonei].remove(segment)

            #update adj_matrix
            all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, new_zonecs, self.adj_matrix)

            #rebuld zone
            skipws = []
            for wsi in range(len(connectedws)):
                if wsi == 0:
                    #save the location of the inserted crit segemnts in zonecs
                    inserti = len(new_zonecs[zonei])
                
                ws1 = connectedws[wsi]
                if not ws1 in skipws:
                    #find closest WS in connectedws
                    bestdist = 1000
                    index1 = self.ws_to_index(ws1)
                    for ws in connectedws:
                        if ws != ws1 and ws != tipws:
                            index2 = self.ws_to_index(ws)
                            if self.workstation_dist_mtx[index1][index2] < bestdist:
                                bestdist = self.workstation_dist_mtx[index1][index2]
                                ws2 = ws

                    #find the path connecting two prev connected WS
                    critws1 = self.ws_crit_point(ws1)
                    critws2 = self.ws_crit_point(ws2)
                    path,dist = self.phase1.shortest_dist(critws1, critws2, all_adjMatrix[zonei])
                    #if there is no path connecting the two prev WS then select a random one from connected WS
                    if(path == None):
                        while (True):
                            ws2 = random.choice(connectedws)
                            if ws2 != ws1:
                                critws1 = self.ws_crit_point(ws1)
                                critws2 = self.ws_crit_point(ws2)
                                path,dist = self.phase1.shortest_dist(critws1, critws2, all_adjMatrix[zonei])
                            if path != None:
                                break
                    new_zonecs[zonei].append(path)
                    all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, new_zonecs, self.adj_matrix)
                    skipws.append(ws2)
            
            #special case for when segments are disconnected creating two different zones in the same zone
            if(len(new_zonecs[zonei]) != len(zonews[zonei])-1):
                pool = []
                group = []
                #find groups
                for segment in new_zonecs[zonei]:
                    first = segment[0]
                    last = segment[len(segment)-1]
                    pool.append([first,last])

                skippair = []
                groupi = -1
                for pair in pool:
                    if pair in skippair:
                        continue
                    group.append([])
                    groupi += 1
                    first = pair[0]
                    last = pair[1]
                    group[groupi].append(pair[0])
                    group[groupi].append(pair[1])
                    follow = first
                    skippair.append(pair)
                    added = True
                    while(added):
                        added = False
                        for pair in pool:
                            if pair not in skippair:
                                if follow in pair:
                                    group[groupi].append(pair[0])
                                    group[groupi].append(pair[1])
                                    if pair.index(follow) == 0:
                                        follow = pair[1]
                                    else:
                                        follow = pair[0]
                                    skippair.append(pair)
                                    added = True
                    follow = last
                    added = True
                    while(added):
                        added = False
                        for pair in pool:
                            if pair not in skippair:
                                if follow in pair:
                                    group[groupi].append(pair[0])
                                    group[groupi].append(pair[1])
                                    if pair.index(follow) == 0:
                                        follow = pair[1]
                                    else:
                                        follow = pair[0]
                                    skippair.append(pair)
                                    added = True
                
                #temporary
                for gi1 in range(len(group)):
                    if len(group)-1 < gi1: break
                    g1 = group[gi1]
                    for gi2 in range(len(group)):
                        if len(group)-1 < gi2: break
                        g2 = group[gi2]
                        if (gi1 != gi2):
                            for critws in g1:
                                if critws in g2:
                                    if len(g1)>len(g2):
                                        for x in g2:
                                            group[gi1].append(x)
                                        del group[gi2]
                                    else:
                                        for x in g1:
                                            group[gi2].append(x)
                                        del group[gi1]
                                    break
                #connect to closest ws not in group
                skipg = []
                for gi in range(len(group)):
                    if gi in skipg:
                        continue
                    bestdist = 1000
                    bestcrit1 = ''
                    bestcrit2 = ''
                    for critws1 in group[gi]:
                        #find closest WS connectedws
                        ws1 = self.crit_point_ws(critws1)
                        index1 = self.ws_to_index(ws1)
                        for gi2 in range(len(group)):
                            if gi2 == gi:
                                continue
                            for critws2 in group[gi2]:
                                ws2 = self.crit_point_ws(critws2)
                                index2 = self.ws_to_index(ws2)
                                if (self.workstation_dist_mtx[index1][index2] < bestdist) and (critws1 != critws2):
                                    bestdist = self.workstation_dist_mtx[index1][index2]
                                    bestcrit1 = critws1
                                    bestcrit2 = critws2
                                    skipgi = gi2
                    
                    skipg.append(skipgi)
                    all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, new_zonecs, self.adj_matrix)
                    path,dist = self.phase1.shortest_dist(bestcrit1,bestcrit2,all_adjMatrix[zonei])
                    if(path == None):
                        print("none hit")
                        print(zonews)
                        print(new_zonecs)
                        print(bestcrit1)
                        print(bestcrit2)
                        print(tipws)
                    new_zonecs[zonei].insert(inserti,path)
                    all_adjMatrix = self.update_adj_matrixs(all_adjMatrix, new_zonecs, self.adj_matrix)

            #restructure the zone
            new_zonews = []
            for segment in new_zonecs[zonei]:
                ws1 = self.crit_point_ws(segment[0])
                ws2 = self.crit_point_ws(segment[len(segment)-1])
                if not ws1 in new_zonews:
                    new_zonews.append(ws1)
                if not ws2 in new_zonews:
                    new_zonews.append(ws2)
                
            zonews[zonei] = copy.deepcopy(new_zonews)
            zonecs = copy.deepcopy(new_zonecs)

        return copy.deepcopy(zonews), copy.deepcopy(zonecs)

    def valid_cs(self,zonecs):
        if len(zonecs) == 1:
            return True

        crit_pool = []
        for zone in zonecs:
            zone_pool = []
            for segment in zone:
                for point in segment:
                    zone_pool.append(point)
            crit_pool.append(zone_pool)

        comb = combinations([x for x in range(len(zonecs))], 2)
        comb = list(comb)

        for combo in comb:
            zone1 = crit_pool[combo[0]]
            zone2 = crit_pool[combo[1]]
            for pt in zone1:
                if pt in zone2:
                    return False

        return True

    def finding_fpzij(self,i,j, zone_tip):
        #If i is a tip workstation, then fpzij includes
        #not only fij (which is the flow originating from i and going to j),
        # - but also the flow that originates from workstations in other zones,
        #   passes i, and arrives at j
        # - and the flow that originates from workstations in other zones, passes i, passes j, and arrives at workstations
        #   in other zones.
        # i and j are in the form 'ws1'


        #is i a tip workstation?
        is_tip = False
        #tip_ws = zone_tip

        if(i in zone_tip):
            is_tip = True

        #for x in range(0,self.num_of_zones):
            #x_list = zone_tip[x][0]
            #if (i in x_list):
                #is_tip = True
                #break

        i_index = self.ws_to_index(i)
        j_index = self.ws_to_index(j)

        crit_i = self.ws_crit_point(i)
        crit_j = self.ws_crit_point(j)

        #assumed that i and j are already in the same zone
        fpzij = 0

        if is_tip:
            #case for when i is a tip ws and j is in the zone 
            #need to calculate the flow

            #this could be done better

            for pair,path in self.all_paths_mtx.items():
                #path = self.all_paths_mtx[wsi + ',' + wsj]

                if (path != 0):
                    if (path[0] != crit_j): #for loads going in the opposite direction
                        if (crit_i in path and crit_j in path): #check to see if loads passes through both points
                            if(path.index(crit_i) < path.index(crit_j)): #check to make sure flow is going from i to j
                                wscheck = pair.split(',')
                                fpzij += self.ws_loads[self.ws_to_index(wscheck[0]),self.ws_to_index(wscheck[1])]
        
        else:
            fpzij = self.ws_loads[i_index,j_index]

        return fpzij

    def ws_to_index(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        i_index = ws_list.index(i)
        return i_index
    
    def index_to_ws(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        return ws_list[i]
    
    def ws_crit_point(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        ws_crit_point = self.workstation_points.loc[:,'critical_points'].to_list()
        i_index = ws_list.index(i)
        return ws_crit_point[i_index]
    
    def crit_point_ws(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        ws_crit_point = self.workstation_points.loc[:,'critical_points'].to_list()
        i_index = ws_crit_point.index(i)
        return ws_list[i_index]
    
    def crit_point_index(self, i):
        row_names = self.all_critical_points.index.to_list()
        return row_names.index(i)
    
    def index_crit_point(self, i):
        row_names = self.all_critical_points.index.to_list()
        return row_names[i]
    
    def all_path(self):
        #calculate the path of all workstations that share a load
        #self.all_paths_mtx = np.zeros((len(self.ws_loads[0,:]),len(self.ws_loads[:,0])))
        #all_adj_matrix = [[] for i in range(self.nz)] 2D list
        #this makes a dictionary of paths that share a load between them
        self.all_paths_mtx = {}
        
        for i in range(len(self.workstation_points)):
            i_ws = self.index_to_ws(i)
            for j in range(len(self.workstation_points)):
                j_ws = self.index_to_ws(j)
                if (self.ws_loads[i,j] == 0):
                    continue
                    #self.all_paths_mtx[i_ws +','+ j_ws] = 0
                else:
                    crit_ws_i = self.ws_crit_point(self.index_to_ws(i))
                    crit_ws_j = self.ws_crit_point(self.index_to_ws(j))
                    path,dist = self.phase1.shortest_dist(crit_ws_i, crit_ws_j, self.adj_matrix)
                    #path = np.array(path)
                    self.all_paths_mtx[i_ws +','+ j_ws] = path
        
        #print(self.all_paths_mtx)

    def union_matrixs(self, crit_segments, map, alpha, beta):
        #alpha and beta are indexs
        #only want remove points that are not in alpha or bata
        union_mtx = map.copy()

        for zone in range(0,len(crit_segments)):
            #now remove every point in every crit seg not alpha or beta
            if zone != alpha and zone != beta:
                for path in crit_segments[zone]:
                        union_mtx = self.adj_remove_seg(path,union_mtx)

        return union_mtx
    
    def display(self, i):

        plt.cla()

        #plot map xy for every point and connect to neighbor
        x_points = np.array([self.all_critical_points.get('x')])
        y_points = np.array([self.all_critical_points.get('y')])

        #plot points
        plt.plot(x_points, y_points, 'ok', markersize = 3)

        for row in range(len(self.adj_matrix)):
            for col in range(len(self.adj_matrix)):
                if self.adj_matrix[row,col] != 0:
                    x_line = np.array([])
                    y_line = np.array([])
                    x_line = np.append(x_line,x_points[0][row])
                    y_line = np.append(y_line,y_points[0][row])
                    x_line = np.append(x_line,x_points[0][col])
                    y_line = np.append(y_line,y_points[0][col])
                    plt.plot(x_line,y_line, ls = '-', color = 'k')
        
        #plot workstaions
        station_x = np.array([self.ws_xy.get('x')])
        station_y = np.array([self.ws_xy.get('y')])
        plt.plot(station_x, station_y, 'sb', markersize = 4)

        #plot zones
        zone_color = np.array(['#0000FF', '#FF7F50', '#FF1493'])
        i=0
        for zone in self.displaycs:
            #print("zone to paint",zone)
            zcolor = zone_color[i]
            i += 1
            for segment in zone:
                for criti in range(len(segment)):
                    if criti+1 == len(segment):
                        break
                    crita = segment[criti]
                    critb = segment[criti+1]
                    x_line = np.array([])
                    y_line = np.array([])
                    x_line = np.append(x_line,self.all_critical_points.at[crita,'x'])
                    y_line = np.append(y_line,self.all_critical_points.at[crita,'y'])
                    x_line = np.append(x_line,self.all_critical_points.at[critb,'x'])
                    y_line = np.append(y_line,self.all_critical_points.at[critb,'y'])
                    plt.plot(x_line, y_line, ls = '-', color = zcolor, linewidth = 2.0)

    def rundisplay(self):
        #while(not self.kill_disp_thread):
        self.ani = FuncAnimation(plt.gcf(), self.display, frames = 100, interval = 500)
        plt.show()
        #return

    #getters
    def get_shortest_dist(self,i,j):
        #i and j are in form "WS1"
        i_index = self.ws_to_index(i)
        j_index = self.ws_to_index(j)
        return self.workstation_dist_mtx[i_index,j_index]

    def set_wscs(self,zonews,zonecs):
        #this function is used for when the simulation has stopped unexpectectly and needs to continue where it left off
        self.opt_ws = zonews
        self.opt_cs = zonecs
        self.bestAdj_matrix = self.update_adj_matrixs(self.all_adj_matrix,self.opt_cs,self.adj_matrix)

    def get_numtip(self):
        return self.num_tip
    def get_initialws(self):
        return self.P_initial_WS
    def get_initialcs(self):
        return self.P_initial_CS
    def get_SAws(self):
        return self.opt_ws
    def get_SAcs(self):
        return self.opt_cs
    def get_lowestSVp(self):
        return self.lowestSVp
    def get_numzones(self):
        return self.num_of_zones
    def get_alladjMatix(self):
        return self.bestAdj_matrix
    #setters
    def set_wsloads(self,new_loads):
        self.ws_loads = new_loads #loads has to be a numpy matrix
        self.all_adj_matrix = self.phase1.all_adj_matrix #reset adjacency matrix to og


if __name__ == '__main__':
    print("in main")
    """
    test_zone_ws = [['WS3', 'WS5', 'WS4', 'WS7', 'WS8'], ['WS11', 'WS10', 'WS9', 'WS6'], ['WS1', 'WS2']]
    test_crit_segments = [[['P', 'Q', 'R', 'S'], ['P', 'O', 'J', 'K'], ['S', 'T', 'U'], ['P', 'O', 'T', 'U', 'V', 'd']], [['a', 'Z', 'Y', 'b'], ['a', 'h', 'g', 'f', 'c'], ['b', 'Y', 'X']], [['E', 'D', 'H', 'I']]]
    workstation_points = pd.read_csv(r'Workstation_points.csv', sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
    workstation_loads = pd.read_csv(r'Workstation_Loads.csv', sep=',', header=0, names=['WS1','WS2','WS3','WS4','WS5','WS6','WS7','WS8','WS9','WS10','WS11'], encoding = 'utf-8')
    adjacency_Mtx_pd = pd.read_csv(r'Adjacency_matrix.csv', sep=',', header=0, names=['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f','g','h','i'], encoding = 'utf-8')
    adj_matrix = adjacency_Mtx_pd.to_numpy()
    ws_loads = workstation_loads.to_numpy()
    """


    #(self,zone_work_stations,zone_crit_segments,work_station_loads)
    numtests = 1
    etime = 2
    row = []
    for _ in range(numtests):
        test = Phase2()
        #test.start_GA(etime)
        test.SA_improvement(etime)
        #row.append(test.get_lowestSVp())
        #row.append(test.get_SAws())
        #row.append(test.get_numtip())
        #row.clear()
        #print("next time segment")
