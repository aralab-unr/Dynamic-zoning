from __future__ import absolute_import
from .goal_generator import GoalGenerator


class GoalReader(GoalGenerator):
    def __init__(self, file_path):
        self.__file_path = file_path #get the file path to the goals.txt file
        self.__generator = self.__get_goal() #get the first goal as a list 

    def generate_goal(self, max_num_of_trials=1000):
        try:
            return next(self.__generator)
        except StopIteration:
            return

    def __get_goal(self):
        for row in open(self.__file_path, "r"):
            yield list(map(float, row.strip().split(" "))) #yield reads the entire file then return the first line, 
            #it will give the next line once the excution has finished 
            # the map function applies the "float" conversion to each element in line
