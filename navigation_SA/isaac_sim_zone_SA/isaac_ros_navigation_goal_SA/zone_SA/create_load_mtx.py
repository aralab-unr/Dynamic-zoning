import numpy as np
import pandas as pd
import rospkg
from csv import writer, reader
import os

numws = 18
wsload = np.zeros((numws,numws),dtype=np.float32)
processing_routes = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','processing_routes_train.csv'), sep=',', header=0, names=['part_type','route','qty'], encoding = 'utf-8')
#processing_routes = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','processing_routes_test1.csv'), sep=',', header=0, names=['part_type','route','qty'], encoding = 'utf-8')
qtylist = processing_routes['qty']
routeslist = processing_routes['route']
#for proccessing_route.csv
for i in range(len(qtylist)):
    route = processing_routes.at[i,'route'].split(',')
    for k in range(len(route)-1):
        ws = int(route[k])-1
        nextws = int(route[k+1])-1
        wsload[ws][nextws] += int(qtylist[i])

df = pd.DataFrame(wsload)
names = ['WS'+ str(i+1) for i in range(numws)]
df.columns = names
df.index = names
print(wsload)

df.to_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','Workstation_Loads.csv'))
#df.to_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','WS_rec_load.csv'))
