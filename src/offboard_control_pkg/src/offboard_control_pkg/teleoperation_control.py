#! /usr/bin/env python2

from __future__ import division
import rospy
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State,Altitude, HomePosition, ExtendedState, PositionTarget
from mavros_msgs.srv import CommandBool,SetMode,CommandTOL, ParamGet, StreamRateRequest,StreamRate
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String,Bool
from geometry_msgs.msg import Vector3, Point
from tf.transformations import euler_from_quaternion
import tty, termios, sys, select


msg = '''t/l takeoff/land'''

class TeleopControl(object):
    
    def __init__(self):
        self.current_key = None
        self.service_timeout = 30
        self.setup_pubsub()
        self.setup_services()

    '''ros subscribers/publisher'''
    def setup_pubsub(self):
        rospy.loginfo("-------Setting pub - sub-----")
        #subscriber
        self.state_sub = rospy.Subscriber('/mavros/state',State,self.state_cb)
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state',ExtendedState,self.extended_state_cb)
        self.setpoint_raw_sub = rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,self.setpoint_raw_cb)
        self.local_position_odom_sub = rospy.Subscriber('/mavros/local_position/odom',Odometry,self.local_position_odom_cb)
        self.local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)

        #publishers
        self.setpoint_raw_pub = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=10)
        self.setmission_pub = rospy.Publisher('/mission/start',Bool,queue_size=10)


     #ros services
    def setup_services(self):
        rospy.loginfo('----Waiting for services to connect----')
        try:
            rospy.wait_for_service('/mavros/param/get', self.service_timeout)
            rospy.wait_for_service('/mavros/set_mode',self.service_timeout)
            rospy.wait_for_service('/mavros/cmd/takeoff',self.service_timeout)
            rospy.wait_for_service('/mavros/cmd/land',self.service_timeout)
            rospy.wait_for_service('/mavros/set_stream_rate',self.service_timeout)
            rospy.loginfo('Services are connected and ready')
        except rospy.ROSException as e:
            rospy.logerr('Failed to initialize service')
        
        #get services
        self.get_param_srv = rospy.ServiceProxy('/mavros/param/get',ParamGet)
        
        #set services
        self.set_arm_srv = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        self.set_takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff',CommandTOL)
        self.set_land_srv = rospy.ServiceProxy('/mavros/cmd/land',CommandTOL)


    def get_key(self,settings):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin],[],[],0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def start_listen(self):
        while True:
            try:
                settings = termios.tcgetattr(sys.stdin)
                key = self.get_key(settings)
                print('Pressed key - '+ str(key))
                if key=="t":
                    self.set_takeoff_srv(altitude = 10.0)
                elif key == "l":
                    self.set_takeoff_srv(altitude = 0.0)
                else:
                    rospy.loginfo("Command not found")
            except rospy.ROSInterruptException as exception:
                rospy.loginfo('error occured while keyboard listen')


    #callback functions
    def altitude_cb(self,data):
        pass     
    
    def state_cb(self,data):
        pass

    def extended_state_cb(self,data):
        pass
    
    def setpoint_raw_cb(self,data):
        pass

    def local_position_cb(self,data):
        pass

    def local_position_odom_cb(self,data):
        pass
        
    def imu_cb(self,data):
        self.imu = data


if __name__=='__main__':
    rospy.init_node('teleop_ros_node',anonymous=True)
    try:
        teleop_control = TeleopControl()
        teleop_control.start_listen()
        rospy.spin()
    except rospy.ROSInterruptException as exception:
        pass