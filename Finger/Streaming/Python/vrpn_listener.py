#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 19 19:53:45 2022

@author: stefan
"""

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/vrpn_client_node/Trackable1/pose", PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
