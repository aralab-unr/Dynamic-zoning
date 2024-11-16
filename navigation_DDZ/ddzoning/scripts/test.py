import numpy as np
import yaml
import os
import copy



zonews=[['WS15', 'WS18'], ['WS5', 'WS3', 'WS7', 'WS6', 'WS9', 'WS2', 'WS10', 'WS1', 'WS4', 'WS14'], ['WS17', 'WS16', 'WS12', 'WS13', 'WS11', 'WS8']]
tipws = 'WS18'
while(True):
    breakloop = False
    for zone in zonews:
        if tipws in zone and len(zone) < 3:
            breakloop = True
    if breakloop:
        print("breaking loop")
        break
print("after loop")
n=0

st = True
dt = st.copy
st = False
print(dt)

print(n/1)

print("is part none", n is None)
if not n:
    print("stuff in n")


t1 = [1,6,3]
t2 = [6,9,6]

if t1 == t2:
    print("same")

t21 = "WS11"
print(t21.replace("WS",""))

starti = 0
for _ in range(len(t1)): 
    for neighi in range(len(t1)-starti):
        neigh = t1[neighi]
        print(neigh)
    starti += 1
    if _ == 1:
        if _ == _:
            print("stoping for loop")
            continue

test_list = [False,False]
if all(test_list) is True:
    print("all true")
else:
    print("some false")

tlist = [3,9]
print(tlist[((tlist.index(3))+1)%2])
tlist.pop()
print(tlist)

point1 = np.array((5,9))
point2 = np.array((5,8.5))

dist = np.linalg.norm(point1-point2)
direction  = point1 - point2

print("test dist:", dist)
print("direction", direction / dist)
miniDick = {2: ['WS1','WS2','WS9'], 6: ['WS8','WS10']}

miniDick[2].remove('WS1')
print("minidik",miniDick)
del miniDick[2]
print("minidik",miniDick)

if 9 in miniDick.keys():
    print("num in miniDick")

minimap1 = {"ID": 2, "Type": "A", "count": 3}
minimap2 = {"ID": 6, "Type": "B", "count": 1, "zone": ['h','j','k']}
minimap3 = {"ID": 3, "Type": "C", "count": 6, "zone": ['z','y','x']}
totmap = {"zone1": minimap1, "zone2": minimap2, "zone3": minimap3}

totmap.popitem()
print(totmap)
#ymap = yaml.safe_load(totmap)

val = copy.deepcopy(minimap3)
for num in val.values():
    print("num in minimap:",num)

with open('test.yaml', 'w') as file:
    yaml.dump(totmap, file)

with open('test.yaml', 'a') as file:
    yaml.dump(miniDick, file)

with open('test.yaml', 'r') as file:
    newmap = yaml.safe_load(file)

totmap["zone1"]["zone"] = ["o","u","a","e","h"]
del totmap["zone2"]["zone"]
#print(totmap)
#print("zone info\n",newmap["zone1"])
#print(newmap["zone1"])

path = os.path.join(os.getcwd(),'test.yaml')

#try:
    #os.remove(path)
#except FileNotFoundError:
    #print("couldnt find file")

testdel = "/robot1/"
res = int(''.join(filter(lambda i: i.isdigit(), testdel)))

testcs = [str(['c','p','l']),str(['l','q','y'])]
testcs = [list(testcs[i]) for i in range(len(testcs))]
testcs1 = [str(['a','p','v']),str(['l','q','y'])]
#print(testcs+'1')

recorded_load = np.zeros((18,18),dtype=np.float32)
print(np.sum(recorded_load))