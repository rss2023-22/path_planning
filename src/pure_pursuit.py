#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

eucl_dist = lambda a,b: ((a[0]-b[0])**2+(a[1]-b[1])**2)**0.5

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 0.5 # FILL IN #
        self.speed            = 1 # FILL IN #
        self.wheelbase_length = 1 # FILL IN #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def odometry_callback(self, msg):
        '''
        Given a pose estimate from the localization package,
        figure out where to drive to, and make the appropriate drive command
        '''
        if self.trajectory.empty(): return
        
        pose = (msg.pose.pose.position.x,msg.pose.pose.position.y)
        
        # find target point
        nearest_point_index = None
        nearest_point = None
        nearest_dist = float('inf')
        for i in range(len(self.trajectory.points)-1):
            # Always check endpoints of current segment
            crit_points = [self.trajectory.points[i],self.trajectory.points[i+1]]
            
            # Parameterize the line and determine if an interior point is a critical point
            dx,dy = self.trajectory.points[i+1][0]-self.trajectory.points[i][0],\
                    self.trajectory.points[i+1][1]-self.trajectory.points[i][1]
            x0,y0 = self.trajectory.points[i][0]-pose[0],self.trajectory.points[i][1]-pose[1]
            t = -(x0*dx+y0*dy)/(dx**2+dy**2)
            if t > 0 and t < 1:
                crit_points.append((t*self.trajectory.points[i+1][0]+(1-t)*self.trajectory.points[i][0],
                                    t*self.trajectory.points[i+1][1]+(1-t)*self.trajectory.points[i][1]))
            
            # Test each critical point
            for j in range(len(crit_points)):
                dist = eucl_dist(crit_points[j],pose)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_point = crit_points[j]
                    if j == 0: nearest_point_index = i
                    elif j == 1: nearest_point_index = i+1
                    elif j == 2: nearest_point_index = i+t
        
        # If we're too far from the track, beeline to the nearest point
        if nearest_dist >= self.lookahead:
            target_point = nearest_point
        else:
            # Otherwise, find the first point after the nearest point outside the circle
            for i in range(int(nearest_point_index)+1,len(self.trajectory.points)):
                if eucl_dist(self.trajectory.points[i],pose) > self.lookahead:
                    # Find intersection of the line with the target circle closer to the next point
                    dx,dy = self.trajectory.points[i][0]-self.trajectory.points[i-1][0],\
                            self.trajectory.points[i][1]-self.trajectory.points[i-1][1]
                    x0,y0 = self.trajectory.points[i-1][0]-pose[0],self.trajectory.points[i-1][1]-pose[1]
                    a,b,c = dx**2+dy**2,2*(x0*dx+y0*dy),x0**2+y0**2-self.lookahead**2
                    t = (-b+(b**2-4*a*c)**0.5)/(2*a)
                    assert t >= 0 and t <= 1
                    target_point = (t*self.trajectory.points[i][0]+(1-t)*self.trajectory.points[i-1][0],
                                    t*self.trajectory.points[i][1]+(1-t)*self.trajectory.points[i-1][1])
                    break
            else:
                # Stop the car if the endpoint is close enough
                if eucl_dist(self.trajectory.points[-1],pose) < 0.25:
                    drive_cmd = AckermannDriveStamped()
                    drive_cmd.header.frame_id = 'base_link'
                    drive_cmd.header.stamp = rospy.Time()
                    drive_cmd.drive.speed = 0
                    self.drive_pub.publish(drive_cmd)
                    return
                target_point = self.trajectory.points[-1]

        # find angle in car frame
        dx,dy = target_point[0]-pose[0],target_point[1]-pose[1]
        theta = 2*np.arctan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        dx,dy = np.cos(theta)*dx+np.sin(theta)*dy,-np.sin(theta)*dx+np.cos(theta)*dy
        delta = np.arctan2(dy,dx)
        
        # drive to target
        drive_cmd = AckermannDriveStamped()
        
        drive_cmd.header.frame_id = 'base_link'
        drive_cmd.header.stamp = rospy.Time()
        
        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = delta
        
        self.drive_pub.publish(drive_cmd)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
