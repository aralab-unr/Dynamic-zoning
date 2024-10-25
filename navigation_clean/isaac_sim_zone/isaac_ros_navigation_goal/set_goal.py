#!/usr/bin/env python

from __future__ import absolute_import
import rospy
import actionlib #https://wiki.ros.org/actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #This package contains the messages used to communicate with the move_base node.
from obstacle_map import GridMap
from goal_generators import RandomGoalGenerator, GoalReader
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import concurrent.futures
import copy
#from zone.ZoneControl import zone_control


class SetNavigationGoal:
    def __init__(self):
        #self.__goal_generator = self.__create_goal_generator() #calls __create_goal_generator() defines weather goal is random or from goals.txt
        action_server_name = rospy.get_param("action_server_name", "move_base") #gets the value(move_base) from the parameter server(action server) a disctionary is returned which is the namespace
        #The action client and server communicate over a set of topics, described in the actionlib protocol. 
        #The action name(action_server_name) describes the namespace containing these topics, 
        #and the action specification message(MoveBaseAction) describes what messages should be passed along these topics.
        self._action_client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
        self.MAX_ITERATION_COUNT = rospy.get_param("iteration_count", 1)#get max iterations and set the current iteration
        assert self.MAX_ITERATION_COUNT > 0
        self.curr_iteration_count = 1
        self.__initial_goal_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1) #publish the initial goal as the itital Pose "initialpose" is the topic name, "PoseWithCovarianceStamped" is the type of message
        self.__initial_pose = rospy.get_param("initial_pose", None)  #from the prameters server get the initial pose
        self.__is_initial_pose_sent = True if self.__initial_pose is None else False #flag for if the inital pose is none

        self.ready_flag = Bool()
        self.ready_flag.data = False
        self.goal_msg = MoveBaseGoal()
        self.current_msg = MoveBaseGoal()
        self.next_msg = MoveBaseGoal()

        self.flag = rospy.Publisher("ready_flag",Bool,queue_size = 25) #create ros publisher
        rospy.Subscriber("newgoal",MoveBaseGoal, self.retrive_goal)

        self.inital_pose = MoveBaseGoal()
        self.set_initial_goal() #generate inital goal message and save
        self.true_count = 0
        self.false_count = 0
        self.user_navgoal = False

        #self.flag.publish(self.ready_flag) #get ready for initial goal

    def __send_initial_pose(self):
        """
        Publishes the initial pose.
        This function is only called once that too before sending any goal pose
        to the mission server.
        """
        goal = PoseWithCovarianceStamped()
        goal.header.frame_id = rospy.get_param("frame_id", "map")
        goal.header.stamp = rospy.get_rostime()
        goal.pose.pose.position.x = self.__initial_pose[0]
        goal.pose.pose.position.y = self.__initial_pose[1]
        goal.pose.pose.position.z = self.__initial_pose[2]
        goal.pose.pose.orientation.x = self.__initial_pose[3]
        goal.pose.pose.orientation.y = self.__initial_pose[4]
        goal.pose.pose.orientation.z = self.__initial_pose[5]
        goal.pose.pose.orientation.w = self.__initial_pose[6]
        rospy.sleep(1)
        self.__initial_goal_publisher.publish(goal)

    def send_goal(self, goal):
        """
        Sends the goal to the action server.
        """
        self.goal_msg = copy.deepcopy(goal)
        print("*******going to*******")
        print("x:", self.goal_msg.target_pose.pose.position.x)
        print("y:", self.goal_msg.target_pose.pose.position.y)


        if not self.__is_initial_pose_sent:
            #rospy.loginfo("Sending initial pose")
            self.__send_initial_pose()
            self.__is_initial_pose_sent = True

            # Assumption is that initial pose is set after publishing first time in this duration.
            # Can be changed to more sophisticated way. e.g. /particlecloud topic has no msg until
            # the initial pose is set.
            #rospy.sleep(10)
            #rospy.loginfo("Sending first goal")

        self._action_client.wait_for_server() #wait for action server to come online
        #goal_msg = self.__get_goal() #get the list for the generate goal class

        if self.goal_msg is None:
            rospy.signal_shutdown("Goal message not generated.")
            sys.exit(1) #shutdown if no goal was given exit with error
            #rospy.spin()

        #rospy.loginfo("sending goal and monitoring")
        self._action_client.send_goal(self.goal_msg, feedback_cb=self.__goal_response_callback) #send goal to action server and see if the goal has been accepted or rejected, continues sending until the satus has changed
        self._action_client.wait_for_result(rospy.Duration(5000))
        rospy.loginfo("reached goal")

    def __goal_response_callback(self, feedback):
        """
        Callback function to check the response(goal accpted/rejected) from the server.\n
        If the Goal is rejected it stops the execution for now.(We can change to resample the pose if rejected.)
        """

        if self.verify_goal_state():
            #rospy.loginfo("Waiting to reach goal")
            wait = self._action_client.wait_for_result() #this will pause until the action has a result

            if self.verify_goal_state():
                self.__get_result_callback(True) #monitors the state to see if the number of iterations is greater then the max or the goal has been rejected
           
    def verify_goal_state(self):
        print("Action Client State:", self._action_client.get_state(), self._action_client.get_goal_status_text())
        if self._action_client.get_state() not in [0, 1, 3]: #monitor the goal state and see if the action client fails to reach goal
            print("2D goal sent")
            self.false_count = 0 
            self.user_navgoal = True
            #rospy.signal_shutdown("Goal Rejected :(")
            return False
        
        #if self._action_client.get_state() == 2:


        return True

    def set_initial_goal(self):
        self.inital_pose.target_pose.header.frame_id = rospy.get_param("frame_id", "map") #add parameters
        self.inital_pose.target_pose.header.stamp = rospy.get_rostime()
        self.inital_pose.target_pose.pose.position.x = self.__initial_pose[0]#set the pose in the goal msg
        self.inital_pose.target_pose.pose.position.y = self.__initial_pose[1]
        self.inital_pose.target_pose.pose.orientation.x = self.__initial_pose[3]
        self.inital_pose.target_pose.pose.orientation.y = self.__initial_pose[4]
        self.inital_pose.target_pose.pose.orientation.z = self.__initial_pose[5]
        self.inital_pose.target_pose.pose.orientation.w = self.__initial_pose[6]
    
    def __get_goal(self):
        goal_msg = MoveBaseGoal() #create a goal msg
        goal_msg.target_pose.header.frame_id = rospy.get_param("frame_id", "map") #add parameters
        goal_msg.target_pose.header.stamp = rospy.get_rostime()
        pose = self.__goal_generator.generate_goal() #from the GoalReader class generate goal pose is a list of floats

        # couldn't sample a pose which is not close to obstacles. Rare but might happen in dense maps.
        if pose is None:
            rospy.logerr("Could not generate next goal. Returning. Possible reasons for this error could be:")
            rospy.logerr(
                "1. If you are using GoalReader then please make sure iteration count <= no of goals avaiable in file."
            )
            rospy.logerr(
                "2. If RandomGoalGenerator is being used then it was not able to sample a pose which is given distance away from the obstacles."
            )
            return

        rospy.loginfo("Generated goal pose: {0}".format(pose))
        goal_msg.target_pose.pose.position.x = pose[0]#set the pose in the goal msg
        goal_msg.target_pose.pose.position.y = pose[1]
        goal_msg.target_pose.pose.orientation.x = pose[2]
        goal_msg.target_pose.pose.orientation.y = pose[3]
        goal_msg.target_pose.pose.orientation.z = pose[4]
        goal_msg.target_pose.pose.orientation.w = pose[5]
        return self.inital_pose #return the edited goal msg

    def __get_result_callback(self, wait):

        if wait and self.curr_iteration_count < self.MAX_ITERATION_COUNT:
            self.curr_iteration_count += 1
            self.send_goal(self.goal_msg)
        else:
            print("havn't reached goal in time", wait)
            self.go_home()
            #rospy.signal_shutdown("Iteration done or Goal not reached.")

    # in this callback func we can compare/compute/log something while the robot is on its way to goal.
    def __feedback_callback(self, feedback_msg):
        pass

    def __create_goal_generator(self):
        goal_generator_type = rospy.get_param("goal_generator_type", "RandomGoalGenerator") #from the parameter server get the goal generator type 
        goal_generator = None
        if goal_generator_type == "RandomGoalGenerator": #case for if the goal generator is random
            if rospy.get_param("map_yaml_path", None) is None:
                rospy.loginfo("Yaml file path is not given. Returning..")
                sys.exit(1)

            yaml_file_path = rospy.get_param("map_yaml_path", None)
            grid_map = GridMap(yaml_file_path)
            obstacle_search_distance_in_meters = rospy.get_param("obstacle_search_distance_in_meters", 0.2)
            assert obstacle_search_distance_in_meters > 0
            goal_generator = RandomGoalGenerator(grid_map, obstacle_search_distance_in_meters)

        elif goal_generator_type == "GoalReader": #case for if the goal generator is from goals.txt
            if rospy.get_param("goal_text_file_path", None) is None: #check to see if the text file path was given
                rospy.loginfo("Goal text file path is not given. Returning..")
                sys.exit(1)

            file_path = rospy.get_param("goal_text_file_path", None) #get the goal text file path
            goal_generator = GoalReader(file_path) #generate instance of class GoalReader
        else:
            rospy.loginfo("Invalid goal generator specified. Returning...")
            sys.exit(1)
        return goal_generator
    
    def retrive_goal(self, data):
        self.set_ready_flag(False)
        #rospy.loginfo("Recieved goal")
        self.next_msg = data
        #self.flag.publish(self.ready_flag)

    def is_ready(self):
        return self.ready_flag.data
    
    def set_ready_flag(self, bool_status):
        self.ready_flag.data = bool_status
    
    def const_pub(self):
        
        while(True):
            self.flag.publish(self.ready_flag)
            rospy.sleep(5)
            if(self.ready_flag.data == True):
                self.true_count += 1
            else:
                self.true_count = 0

            if self.user_navgoal: #if a user goal was imputed
                self.false_count += 1

    def get_true_count(self):
        return self.true_count

    def get_false_count(self):
        return self.false_count
    
    def reset_false_count(self):
        self.false_count = 0
    
    def save_currentgoal(self):
        self.current_msg = self.next_msg

    def go_home(self):
        self.send_goal(self.inital_pose)
        self.true_count = 0
    
    def go_next(self):
        self.send_goal(self.next_msg)
    
    def go_current(self):
        self._action_client.cancel_goal()
        self.send_goal(self.current_msg)

    def get_usernav(self):
        return self.user_navgoal
    
    def set_usernav(self, data):
        self.user_navgoal = data

def main():
    rospy.init_node("set_goal_py") #create node
    set_goal = SetNavigationGoal() #set creates a class, gets first goal, initalized the action server
    #result = set_goal.send_goal()#send the inital positions
    #flag = rospy.Publisher("ready_flag",Bool,queue_size = 10) #create ros publisher
    #rospy.loginfo("publishing ready flag")
    set_goal.set_ready_flag(True)
    pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    pool.submit(set_goal.const_pub)

    while not rospy.is_shutdown():
        if ((not set_goal.is_ready()) and (not set_goal.get_usernav())):
            #rospy.loginfo("Flag is now set to False")
            set_goal.save_currentgoal() #save the current goal
            set_goal.go_next() #send goal positions
            if not set_goal.get_usernav(): #if the goal wasn't inturputed by user goal
                set_goal.set_ready_flag(True) #get next goal

        #set_goal._pub()
        if(set_goal.get_true_count() > 5):
            #print("going home")
            set_goal.go_home()

        if(set_goal.get_usernav() and set_goal.get_false_count() > 2): 
            print("setting last part goal as next goal")
            set_goal.reset_false_count()
            set_goal.set_usernav(False)
            set_goal.go_current()
            #set_goal.set_ready_flag(True)
        
        if set_goal.get_usernav():
            print("going to user goal time:",set_goal.get_false_count())

            
        rospy.sleep(.01)


    #pool.shutdown(wait=True)

if __name__ == "__main__":
    main()
