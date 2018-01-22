#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from geometry_msgs.msg import Vector3
from std_srvs.srv  import Empty

class ReferencePublisher(object):
    """docstring for ReferencePublisher"""
    def __init__(self):
        
        self.euler_ref_pub = rospy.Publisher('euler_ref', Vector3, queue_size=1)
        self.euler_ref_msg = Vector3()
        self.pitch_ref = [0.0, 0.2, 0.0, 0.0, 0.0, 0.2, 0.0]# 0.0, 0.2, 0.0,-0.2, 0.0]
        self.roll_ref =  [0.0, 0.0, 0.0, -0.2, 0.0,-0.2, 0.0]#, 0.2, 0.0,-0.2, 0.0]
        self.counter = 0

    def run(self):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            gazebo_start = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            gazebo_start()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        while not rospy.is_shutdown():

            rospy.sleep(10.0)
            self.euler_ref_msg.x = self.roll_ref[self.counter]
            self.euler_ref_msg.y = self.pitch_ref[self.counter]
            self.euler_ref_msg.z = 0

            self.euler_ref_pub.publish(self.euler_ref_msg)
            self.counter = (self.counter + 1 ) % len(self.pitch_ref)

            if self.counter == 0:
                return

if __name__ == '__main__':

    rospy.init_node('attitude_ref_pub')
    attitude_ref = ReferencePublisher()
    attitude_ref.run()
