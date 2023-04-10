#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

#### NOTE: later on we should try dublin curves, nice package in the git for this...

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):

        self.start_pos = None
        self.goal_pos = None
        self.graph = None
        self.map = None

        self.odom_topic = rospy.get_param("~odom_topic")
        self.odom_topic = '/odom' # FOR TESTING ONLY - on real car, change to listen to localization shit... is this right?
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)



    # def realCoord_to_mapCoord(coord):
    #     u,v = coord



    def map_cb(self, msg):
        # some notes: all map vals are 0, -1, or 100. Why tf we care about -1? set tht shit to 100.
        # at some point will want to dilate map to account for real-world dynamics!
        self.map = msg
        print(self.map)



    def odom_cb(self, data): # sets the curren position of the car
        
        now_odom = np.array([data.twist.twist.linear.x,
                                 data.twist.twist.linear.y,
                                 data.twist.twist.angular.z])
        self.start_pos = now_odom
        print(self.start_pos)

        
            


    def goal_cb(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.orientation.z
        w = data.pose.orientation.w
        theta = 2*np.arctan2(z,w)
        self.goal_pos = (x,y,theta)
        print('set new goal:'+str(self.goal_pos))

    def plan_path(self, start_point, end_point, map):
        # plans path from start to goal
        # start with discretization and A*!
        # chunk up space. Maybe tke 4 corners of a chunk and check if any r obstcle in map and if so mark entire chunk as obstcle? 

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
