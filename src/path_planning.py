#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
from scipy import ndimage
import time, os
#import dubins
import tf
from utils import LineTrajectory
from graph_class import Graph
import heapq
#import cv2 
#from skimage import morphology

#### NOTE: later on we should try dublin curves, nice package in the git for this...
# roslaunch racecar_simulator simulate.launch
# roslaunch lab6 plan_trajectory.launch
# sudo apt-get install python-scipy


# TODO: figure out dilation in gradescope? idk
class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):

        self.start_pos = None
        self.goal_pos_irl = None
        self.goal_pos_map = None
        self.graph = None
        self.map = None
        self.map_coords = None
        self.height = None
        self.width = None
        self.start_time = None

        self.odom_topic = rospy.get_param("~odom_topic")
        self.odom_topic = '/odom' # FOR TESTING ONLY - on real car, change to listen to localization shit... is this right?
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb) #TODO: MAKE THIS FUNCTIONAL
        self.intial_pose_sub = rospy.Subscriber('/initialpose',PoseWithCovarianceStamped, self.initial_pose_cb, queue_size=10)

        self.rot_matrix = None
        self.rot_alt = None
        self.rot_back = None
        self.rot_back_alt = None
        self.heuristic = None
        self.map_res = None

    def pixelToMapCoords(self,u,v):
        u *= self.map_res; v *= self.map_res
        x,y,_ = np.matmul(np.array([[u,v,0]]),self.rot_alt)[0]
        return x+self.rot_matrix[0][3], y+self.rot_matrix[1][3]
    
    def mapToPixelCoords(self,x,y):
        u,v,_  = np.matmul(np.array([[x,y,0]]),self.rot_back_alt)[0]
        x = (u+self.rot_back[0][3])/self.map_res; y = (v+self.rot_back[1][3])/self.map_res
        x=np.rint([x])[0]; y=np.rint([y])[0]
        return int(x),int(y)

    def eucDist(self,coord1,coord2): # coord = (x,y)
        return np.sqrt((coord1[0]-coord2[0])**2+(coord1[1]-coord2[1])**2)

    def map_cb(self, msg):
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
        self.rot_matrix = rot_matrix; self.rot_alt = rot_alt; self.rot_back = np.linalg.inv(self.rot_matrix); self.rot_back_alt = np.linalg.inv(self.rot_alt)
        self.map = map
        map[map > 0] = 1
        map[map < 0] = 1
        map = map.astype('uint8')
        kernel = np.ones((12, 12), np.uint8)
        map = ndimage.binary_dilation(map,structure=kernel)
        self.map = map
        print('MAP INITIATED')
        print('-----------------------------------')

    def initial_pose_cb(self,data):
        u,v = self.mapToPixelCoords(data.pose.pose.position.x,data.pose.pose.position.y)
        if  (u,v) != self.start_pos:
            print('new start(init)' + str((u,v)))
            self.start_pos = (u,v)

    def odom_cb(self, data): # sets the curren position of the car... currently green arrow doesn't work?? same fix as in last lab tho
       # print('HERE')
        #now_odom = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z])
        # if self.rot_alt != None:
        #     u,v = self.mapToPixelCoords(data.twist.twist.linear.x,data.twist.twist.linear.y)
        #     if (u,v) != self.start_pos:    
        #         self.start_pos = (u,v)
        #         print('new start(odom)' + str((u,v)))
        pass


    def goal_cb(self, data):
        theta = 2*np.arctan2(data.pose.orientation.z,data.pose.orientation.w)
        self.goal_pos_irl = (data.pose.position.x,data.pose.position.y,theta)
        print('set new goal:'+str(self.goal_pos_irl))
        print('-----------------------------------')
        self.trajectory = LineTrajectory("/planned_trajectory")
        u,v = self.mapToPixelCoords(self.goal_pos_irl[0],self.goal_pos_irl[1])
        self.goal_pos_map = (u,v) # no theta?


        self.plan_path((self.start_pos[0],self.start_pos[1]),self.goal_pos_map,self.map)



    def AStarWithExpandedList(self,map,S,G):
            
            def computeH(u,v): # NOTE: currently this does it in pixel frame
                return self.eucDist((u,v),(self.goal_pos_map[0],self.goal_pos_map[1]))
            
            def getChildren(i,j):
                return [(i+1,j),(i-1,j),(i,j+1),(i,j-1),(i+1,j+1),(i-1,j-1),(i+1,j-1),(i-1,j+1)]

            """
            A* with an expanded list, assumes consistent heuristic to be correct
            """

            parent = {S:None}
            expanded = set()
            Q = [(computeH(S[0],S[1]),(0,S))] # (cost_to_come+cost_incurred,(cost_incurred, head))
            heapq.heapify(Q)
            print('planning path.......')
            while Q:
                _, N = heapq.heappop(Q)
                head = N[1]; costIncurred = N[0]
                if head == G:
                    #get path from parent dictionary:
                    curr = G
                    path = [G]
                    while parent[curr] != None:
                        path.append(parent[curr])
                        curr = parent[curr]
                    path.reverse()
                    print(path)
                    return costIncurred, path
                
                elif head in expanded:
                    continue
                else:
                    expanded.add(head) # NOTE: check!
                    children = getChildren(head[0],head[1])
                    for child in children:
                        if child not in expanded:
                            try:
                                val = map[child[1]][child[0]]
                                if val == 0:
                                    parent[child] = head
                                    costToChild = self.eucDist(head,child) + costIncurred
                                    heapq.heappush(Q,(costToChild+computeH(child[0],child[1]),(costToChild,child)))
                                    #heapq.heappush(Q,(costToChild,(costToChild,child)))
                                else:
                                    expanded.add(child)
                            except: # out of bounds
                                continue             
            return None, None

    def AStarWithExpandedListPartialPaths(self,map,S,G):
            
            def computeH(u,v): # NOTE: currently this does it in pixel frame
                return self.eucDist((u,v),(self.goal_pos_map[0],self.goal_pos_map[1]))
            
            def getChildren(i,j):
                return [(i+1,j),(i-1,j),(i,j+1),(i,j-1),(i+1,j+1),(i-1,j-1),(i+1,j-1),(i-1,j+1)]
                #return [(i+1,j),(i-1,j),(i,j+1),(i,j-1),(i+1,j+1),(i-1,j-1),(i+1,j-1),(i-1,j+1),(i+2,j),(i-2,j),(i,j+2),(i,j-2),(i+2,j+2),(i-2,j-2),(i+2,j-2),(i-2,j+2),(i+2,j-1),(i-2,j+1),(i+2,j+1),(i-2,j-1),(i+1,j+2),(i-1,j-2),(i+1,j-2),(i-1,j+2)]

            """
            A* with an expanded list, assumes consistent heuristic to be correct
            """

            expanded = set()
            Q = [(computeH(S[0],S[1]),(0,[S]))] # (cost_to_come+cost_incurred,(cost_incurred, head))
            heapq.heapify(Q)
            while Q:
                _, N = heapq.heappop(Q)
                partial_path = N[1]; costIncurred = N[0]
                head = partial_path[-1]
                if head == G:
                    return costIncurred, partial_path
                elif head in expanded:
                    continue
                else:
                    expanded.add(head) # NOTE: check!
                    children = getChildren(head[0],head[1])
                    for child in children:
                        if child not in expanded:
                            try:
                                val = map[child[1]][child[0]]
                                if val == 0:
                                    new_partial = partial_path[:]
                                    new_partial.append(child)
                                    costToChild = self.eucDist(head,child) + costIncurred
                                    heapq.heappush(Q,(costToChild+computeH(child[0],child[1]),(costToChild,new_partial)))
                                    #heapq.heappush(Q,(costToChild,(costToChild,child)))
                                else:
                                    expanded.add(child)
                            except: # out of bounds
                                continue             
            return None, None

    def plan_path(self, start_point, end_point, map):
        print('planning path.......')
        self.start_time = rospy.get_time()
        _, partialPath = self.AStarWithExpandedListPartialPaths(map,start_point,end_point)
        print('finished planning path, time = '+str(rospy.get_time()-self.start_time))
        
        if partialPath is not None:
            for node in partialPath:
                point = Point()
                x1,y1 = np.round(self.pixelToMapCoords(node[0],node[1]),decimals =2)
                point.x = x1
                point.y = y1
                point.z = 0.0
                self.trajectory.addPoint(point)

            # publish trajectory
            self.traj_pub.publish(self.trajectory.toPoseArray())

            # visualize trajectory Markers
            self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
