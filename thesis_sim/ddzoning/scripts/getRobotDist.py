import os

print(os.getcwd())
fileobj = open("terminal","r")
alllineslist = fileobj.readlines()
distance = [0,0,0]
for line in alllineslist:
    if "dist from " in line:
        if "/carter1/" in line:
            filteredline = line.replace("/carter1/ dist from","")
            dist = (float(''.join(filter(lambda i: i.isdigit(), filteredline))))/10
            distance[0] += dist
        elif "/carter2/" in line:
            filteredline = line.replace("/carter2/ dist from","")
            dist = (float(''.join(filter(lambda i: i.isdigit(), filteredline))))/10
            distance[1] += dist
        elif "/carter3/" in line:
            filteredline = line.replace("/carter3/ dist from","")
            dist = (float(''.join(filter(lambda i: i.isdigit(), filteredline))))/10
            distance[2] += dist
        

print(distance)