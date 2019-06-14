#! /usr/bin/env python2

from __future__ import division
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from offboard_control_pkg.offboard_control_ros import OffboardControl
from std_msgs.msg import Bool

class MissionControl(OffboardControl):
    
    def __init__(self):
        super(MissionControl,self).__init__()
        self.rate = rospy.Rate(10)
        self.mission_sub = rospy.Subscriber("/mission/start", Bool, self.mission_cb)
        self.mission_started = False
        self.base_wp = Point(0.0,0.0,3.0)
        self.home_pose = Point(0.0,0.0,0.0)
        self.current_wp = self.base_wp
        self.index = 0

        super(MissionControl,self).start_offboard()

    def start_mission(self):
        rospy.loginfo('----Starting mission now ---')


    def get_next_waypoint(self):
        point0 = self.base_wp
        point1 = Point(0.0, 3.0, 3.0)
        point2 = Point(3.0, 3.0, 3.0)
        point3 = Point(3.0, 0.0, 3.0)
        point4 = Point(0.0, 0.0, 3.0)

        waypoints = [point0,point1,point2, point3, point4]
        if not self.index < len(waypoints):
            self.index = 0
        return waypoints[self.index]
        

    def local_position_cb(self,data):
        current_position = Point(data.pose.position.x,data.pose.position.y,data.pose.position.z)
        dist = self.distance_between_two_pose(self.current_wp, current_position)

        if self.state.mode == "OFFBOARD" and dist < 0.5:
            rospy.loginfo('--Vehicle reached waypoint ---')
            rospy.sleep(2.0)
            self.index += 1
            self.current_wp = self.get_next_waypoint()
            if self.current_wp is not None:
                rospy.loginfo('next wp - '+ str(self.current_wp))
                super(MissionControl,self).go_to_position(self.current_wp,0)
            

    def distance_between_two_pose(self,point1, point2):
        return math.pow((point1.x - point2.x),2) + math.pow((point1.y - point2.y),2) + math.pow((point1.z - point2.z),2)


    def mission_cb(self,data):
        if data and not self.mission_started:
            self.start_mission()
            self.mission_started = True




if __name__=='__main__':
    rospy.init_node('mission_control_ros',anonymous=True)
    try:
        mission_control = MissionControl()
        rospy.spin()
    except rospy.ROSInterruptException as exception:
        pass


