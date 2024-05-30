import time
from queue import Queue
import random
import numpy as np
import math
import multiprocessing
import signal
import concurrent.futures
from csv import writer, reader

threadnum = 0

class testy:
    def __init__(self):
        self.Qs = []
        self.Qs.append(Queue(maxsize=0))
        self.Qs.append(Queue(maxsize=0))
        self.Qs.append(Queue(maxsize=0))
        self.threadnum = 0

        self.testQueue = Queue()
        self.testQueue.put(3)
        self.testQueue.put(8)
        self.testQueue.put(1)
        self.testQueue.put(5)

        self.piss = True

    def worker(self):
        print("thread created: ", self.threadnum)
        index = self.threadnum
        self.threadnum += 1
        while(self.Qs[index].qsize() != 0):
            time_sleep = self.Qs[index].get()
            print("size of Queue",index,":",self.Qs[index].qsize())
            print("sleeping for:",time_sleep,'\n')
            time.sleep(time_sleep)

    def insert_into_Q(self,index):
        self.Qs[index].put(random.randint(0,10))

    def return_3(self):
        return 2,3,6
    
    def get_Q(self):
        return self.testQueue

    def affect_Q(self,sendQ):
        tempQ = Queue()
        for x in range(sendQ.qsize()):
            if x < 2:
                tempQ.put(sendQ.get())
        return tempQ

    def doing_it(self, ispiss):
        self.piss = ispiss
        for i in range(10000000000):
            print("tick")
        
    def getpiss(self):
        print(self.piss)
    
    def setpiss(self,piss):
        self.piss = piss

def main():

    test_list = [[3],[2],[6]]
    if [] in test_list:
        print("one is empty")
    while(True):
        print("in while loop")
        if [] in test_list:
            print("in while loop")
            continue
        break

    print(1/.0000001)
    myrow = []
    with open('/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/thoughput.csv', 'r') as file:
        alllines = reader(file)
        for row in alllines:
            myrow.append(row)
        file.close()

    myrow[2][2] = "piss"
    with open('/home/russell/thesis_ws/src/navigation/isaac_ros_navigation_goal/isaac_ros_navigation_goal/zone/data/thoughput.csv', 'w') as file:
        alllines = writer(file)
        for row in myrow:
            alllines.writerow(row)
        file.close()



    t = time.localtime()

    test_list =[2,3]
    test_list.append([1,2,3,4])
    test_list.remove(2)
    print(test_list)
    test_list.append([9,7,5,4])
    #test_list.append([])
    #test_list[0].append(list_list)
    print(test_list)
    test_list.clear()
    print(test_list)

    nptest = np.array(["p","i","l"])
    print(nptest)
    nptest = np.insert(nptest,0,str(2))
    nptest = np.insert(nptest,0,str(1))
    print(nptest)
    nptest = np.delete(nptest,0)
    print(nptest)

    print("yeee",8%2)
    
    for x in range(5):
        for y in range(5):
            if y == 2:
                break

    zeromtxx = np.zeros((11,11))
    test_str = 'ws8'
    print(test_str.replace("ws",""))

    print(math.gcd(121,121))

    test_Q = Queue()
    #test_Q.put(5)
    for x in range(test_Q.qsize()):
        print("hit")
    #test_Q.put(5)
    #print(test_Q.empty())
    current_zone = 1
    next_zone = 2
    
    test = testy()
    test.getpiss()
    test2 = testy()
    test2.setpiss(False)

    test = test2
    test.getpiss()


    testQ = Queue(maxsize=0)
    testQ.put(5)
    testQ.put(5)
    #testQ.put(5)

    for x in range(testQ.qsize()-1):
        print("ye")

    '''
    test.getpiss()
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
        try:
            executor.submit(test.doing_it).result(timeout=1)
            print("returned result")
        except concurrent.futures.TimeoutError:
            print("took too long")
            executor.shutdown(wait=False)
    test.getpiss()
    print("all done")
    '''
    '''
    proc = multiprocessing.Process(target=test.doing_it, args = (True,))
    proc.start()
    proc.join(timeout=1)

    # If thread is active
    if proc.is_alive():
        print( "foo is running... let's kill it...")

        # Terminate foo
        proc.terminate()
    '''
    '''
    num1,num2,num3 = test.return_3()
    print(num1,num2,num3)

    newQ = test.get_Q()
    newnewQ = test.affect_Q(newQ)

    mine = test.get_Q()
    for x in range(mine.qsize()):
        print(mine.get())

    print('\n')

    for x in range(newnewQ.qsize()):
        print(newnewQ.get())

    '''
    '''
    test.insert_into_Q(0)
    test.insert_into_Q(1)
    test.insert_into_Q(2)
    test.insert_into_Q(0)
    test.insert_into_Q(1)
    test.insert_into_Q(2)
    test.insert_into_Q(0)
    test.insert_into_Q(1)
    test.insert_into_Q(2)
    test.insert_into_Q(0)
    test.insert_into_Q(1)
    test.insert_into_Q(2)

    pool = concurrent.futures.ThreadPoolExecutor(max_workers=11)
    for x in range(0,3):
        pool.submit(test.worker)
        time.sleep(0.01)

    time.sleep(5)
    print("adding one to each queue")
    test.insert_into_Q(0)
    test.insert_into_Q(1)
    test.insert_into_Q(2)
    time.sleep(10)
    print("adding two to each queue")
    test.insert_into_Q(1)
    test.insert_into_Q(0)
    test.insert_into_Q(2)
    test.insert_into_Q(1)
    test.insert_into_Q(0)
    test.insert_into_Q(2)
    time.sleep(15)
    print("adding three to each queue")
    test.insert_into_Q(1)
    test.insert_into_Q(0)
    test.insert_into_Q(2)
    test.insert_into_Q(1)
    test.insert_into_Q(0)
    test.insert_into_Q(2)
    test.insert_into_Q(1)
    test.insert_into_Q(0)
    test.insert_into_Q(2)

    pool.shutdown(wait=True)
    '''
 
if __name__ == "__main__":
    #while not rospy.is_shutdown():
    main() #runs main()