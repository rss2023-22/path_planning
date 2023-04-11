#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
import tf
from utils import LineTrajectory
from graph_class import Graph
import heapq

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
        self.map_coords = None
        self.height = None
        self.width = None

        self.odom_topic = rospy.get_param("~odom_topic")
        self.odom_topic = '/odom' # FOR TESTING ONLY - on real car, change to listen to localization shit... is this right?
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        self.rot_matrix = None
        self.rot_alt = None
        self.heuristic = None
        self.map_res = None

    def pixelToMapCoords(self,u,v):
        u *= self.map_res; v *= self.map_res
        x,y,_ = np.matmul(np.array([[u,v,0]]),self.rot_alt)[0]
        return x+self.rot_matrix[0][3], y+self.rot_matrix[1][3]
    
    def eucDist(self,coord1,coord2):
        return np.sqrt((coord1[0]-coord2[0])**2+(coord1[1]+coord1[1])**2)

    def map_cb(self, msg):
        # NOTE: EVERYTHING IN THIS FUNCTION CAN BE IMPROVED FOR SPEED

        self.map_res = msg.info.resolution
        map = np.array(list(msg.data))
        map = np.reshape(map,(msg.info.height, msg.info.width)) # convert from row-major order
        height = np.shape(map)[0]; self.height = height
        width = np.shape(map)[1]; self.width = width
        self.heuristic = np.zeros((height,width))


        # DEFINE ROTATION STUFF
        rot_matrix = tf.transformations.quaternion_matrix([0, 0, msg.info.origin.orientation.z, msg.info.origin.orientation.w])
        rot_matrix[0][3] = msg.info.origin.position.x; rot_matrix[1][3] = msg.info.origin.position.y; rot_matrix[2][3] = msg.info.origin.position.z
        rot_alt = [rot_matrix[0][0:3],rot_matrix[1][0:3],rot_matrix[2][0:3]]
        self.rot_matrix = rot_matrix; self.rot_alt = rot_alt 
 
        # CREATE GRAPH # NOTE: indexes should be flipped, says to do on git? 
        graph = Graph() # each node is (u,v,x,y)! only feasible nodes and edges exist. Edges undirected
        for i in range(height):
            for j in range(width):
                curr = (i,j)
                if map[curr[0]][curr[1]] == 0: # if current node isn't obstacle      
                    neighbors = [(i+1,j),(i-1,j),(i,j+1),(i,j-1),(i+1,j+1),(i-1,j-1),(i+1,j-1),(i-1,j+1)]
                    for node in neighbors:
                        try:
                            val = map[node[0]][node[1]]
                            if val == 0:
                                graph.addEdge((curr[0],curr[1]),(node[0],node[1])) # NOTE: speed of adding edge can be SIGNIFICANTLY improved in Graph via not double-adding
                        except:
                            continue
        self.graph = Graph() 
        print(len(graph.getNodes()))
        self.map = map



    def odom_cb(self, data): # sets the curren position of the car
        now_odom = np.array([data.twist.twist.linear.x,
                                 data.twist.twist.linear.y,
                                 data.twist.twist.angular.z])
        self.start_pos = now_odom



    def goal_cb(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.orientation.z
        w = data.pose.orientation.w
        theta = 2*np.arctan2(z,w)
        self.goal_pos = (x,y,theta)
        print('set new goal:'+str(self.goal_pos))

        # COMPUTE HEURISTIC
        # NOTE: maybe do this live during A*? idk...
        # NOTE: check that heuristic looks correct...
        print('computing new heuristic....')
        mapGoal = (x,y)
        for i in range(self.height):
            for j in range(self.width):
                mapCurr = self.pixelToMapCoords(i,j)
                eucDist = self.eucDist(mapCurr,mapGoal)
                self.heuristic[i][j] = eucDist
        print('done computing new heuristic!')
        print(self.heuristic)
                



    def plan_path(self, start_point, end_point, map):


        # NOTE:
        # should compute heuristic for each node when create map
        # should maybee not store paths explicitly (use parent pointer?)
        #  QUEUE SHOULD USE HEAPQ   -> SEE: https://docs.python.org/3/library/heapq.html#module-heapq

        def AStarWithExpandedList(graph,hConsistent):
            """
            A* with an expanded list, assumes consistent heuristic
            """
            S = graph.getStartNode()
            G = graph.getGoalNode()
            expanded = set()
            Q = [(0,[S])] # (cost-incurred : [partial path])
            while Q:
                # NOTE: again, would be much smarter to have a sorted Q, but for simplicity this way works
                print(Q) # visualize how Q is different than best-first with uniform cost
                N , Q = chooseN(Q,hConsistent) # pick the path with lowest f, NOTE: f is h+g!!! cost_to_come+cost_incurred! STORE H+G and 
                partialPath = N[1]
                costIncurred = N[0]
                head = partialPath[-1]
                if head == G:
                    return costIncurred, partialPath
                elif head in expanded:
                    continue
                else:
                    children = graph.getChildren(head)
                    for child in children:
                        if child not in expanded:
                            extension = partialPath + [child]
                            costToChild = graph.getEdgeWeight(head,child) + costIncurred
                            Q.append((costToChild,extension))
            return None

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
