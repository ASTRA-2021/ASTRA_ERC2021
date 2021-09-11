#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

waypoints = {1:[12.19,8.73,0.11], 2:[25.04,4.36,-0.09], 3:[28.62,-6.17,0.28], 4:[11.63,-16.85,0.98], 5:[7.64,-5.55,-0.08], 6:[27.48,-13.65,0]}

def talker():
    
    markerPub = rospy.Publisher('robotMarker', Marker, queue_size=10)
    
    rospy.init_node('stateNode', anonymous=True)

    rate = rospy.Rate(1)

    # initial starting location I might want to move to the param list
    
    for ID, pos in waypoints.items():

        print(ID, pos)
        robotMarker = Marker()
        robotMarker.header.frame_id = "/base_footprint"
        robotMarker.header.stamp = rospy.get_rostime()
        robotMarker.ns = "robot"
        robotMarker.id = ID
        robotMarker.type = 2 # sphere
        robotMarker.action = 0
        robotMarker.pose.position = Point(*pos)
        robotMarker.pose.orientation.x = 0
        robotMarker.pose.orientation.y = 0
        robotMarker.pose.orientation.z = 0
        robotMarker.pose.orientation.w = 1.0
        robotMarker.scale.x = 1.0
        robotMarker.scale.y = 1.0
        robotMarker.scale.z = 1.0

        robotMarker.color.r = 0.0
        robotMarker.color.g = 1.0
        robotMarker.color.b = 0.0
        robotMarker.color.a = 1.0

        robotMarker.lifetime = rospy.Duration(0)

        # while not rospy.is_shutdown():
        markerPub.publish(robotMarker)
        print("sending marker")
        rate.sleep()


if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass
