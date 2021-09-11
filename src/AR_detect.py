#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import PointStamped, Point, Pose
from visualization_msgs.msg import Marker


landmarks = {1: [7.31, 0, 0.21],
             2: [7.19, 7.55, 0.20],
             3: [18.85, -3.59, 0.36],
             4: [33.77, 6.41, 0.04],
             5: [13.22, -13.61, 0.82],
             6: [21.01, 13.21, 0.13],
             7: [20.96, 3.36, 0.17],
             8: [20.40, -19.41, 0.79],
             9: [14.77, 6.89, 0.44],
             10: [22.46, -10.36, 0.57],
             11: [31.56, -18.81, 0.58],
             12: [29.92, 11.44, 0.05],
             13: [32.79, -6.79, 0.18],
             14: [2.04, -12.02, 0.50],
             15: [7.63, 13.24, -0.01]}
waypoints = {1: [12.19, 8.73, 0.11],
             2: [25.04, 4.36, -0.09],
             3: [28.62, -6.17, 0.28],
             4: [11.63, -16.85, 0.98],
             5: [7.64, -5.55, -0.08],
             6: [27.48, -13.65, 0]}


class DrawMarker():
    def __init__(self):
        self.fiducial_marker_topic = '/ar_pose_marker'
        self.marker_topic = '/robot_marker'
        self.node_name = 'marker_draw_node'
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker,
                                          queue_size=10)
        self.ar_sub = rospy.Subscriber(self.fiducial_marker_topic,
                                       AlvarMarkers, self.artag_callback)

    def transform_pose(self, input_pose, from_frame, to_frame):
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        # pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped = input_pose
        # pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()
        try:
            # ** It is important to wait for the listener to start listening.
            # Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame,
                                                      rospy.Duration(1))
            return output_pose_stamped.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            raise

    def draw_mark(self, marker_pose, id, marker_type="landmark"):
        if marker_type == "landmark":
            color_rgba = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
        else:
            color_rgba = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green

        robotMarker = Marker()
        robotMarker.header.frame_id = "/base_footprint"
        robotMarker.header.stamp = rospy.get_rostime()
        robotMarker.ns = "robot"
        robotMarker.id = id
        robotMarker.type = 2
        robotMarker.action = 0
        robotMarker.pose = marker_pose
        robotMarker.scale.x = 1.0
        robotMarker.scale.y = 1.0
        robotMarker.scale.z = 1.0
        robotMarker.color = color_rgba
        robotMarker.lifetime = rospy.Duration(0)
        self.marker_pub.publish(robotMarker)

    def artag_callback(self, msg):
        # Identify which landmark is associated with the first marker
        try:
            visible_marker_id = msg.markers[0].id
            visible_marker_pose = msg.markers[0].pose

            for id, position in landmarks.items():
                if (id != visible_marker_id):
                    transformed_pose = self.transform_pose(visible_marker_pose,
                                                    ("lm_"+str(visible_marker_id)+"_frame"),
                                                    ("lm_"+str(id)+"_frame"))
                    self.draw_mark(transformed_pose, id, marker_type="landmark")

            for idx, position in waypoints.items():
                transformed_pose = self.transform_pose(visible_marker_pose,
                                                ("lm_"+str(visible_marker_id)+"_frame"),
                                                ("wp_"+str(idx)+"_frame"))
                self.draw_mark(transformed_pose, idx, marker_type="waypoint")
        except IndexError:
            rospy.logerr("No markers found")


if __name__ == '__main__':
    try:
        md = DrawMarker()
        rospy.init_node(md.node_name, anonymous=False)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
