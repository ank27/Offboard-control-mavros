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

class OffboardControl(object):
    
    def __init__(self):
        self.state = State()
        self.altitude = Altitude()
        self.global_position = HomePosition()
        self.local_position = PoseStamped()  #fcu local position
        self.local_position_odom = Odometry()
        self.extended_state = ExtendedState()
        self.mode = ''
        self.rate = rospy.Rate(10)
        self.service_timeout = 30
        self.offboard_request_threshold = rospy.Duration(5.0)
        self.arming_request_threshold = rospy.Duration(5.0)
        self.imu = Imu()
        self.current_yaw = None
        
        self.setup_pubsub()
        self.setup_services()
        self.current_target_pose = PositionTarget()
        self.current_pose = Point()
        self.home_pose = Point(0.0,0.0,0.0)
        self.waypoint_1 = Point(0.0,0.0,3.0)
        self.should_start_mission = False
        
        

    '''ros subscribers/publisher'''
    def setup_pubsub(self):
        rospy.loginfo("-------Setting pub - sub-----")
        #subscriber
        self.altitude_sub = rospy.Subscriber('/mavros/altitude',Altitude, self.altitude_cb)
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
            rospy.wait_for_service('/mavros/cmd/arming',self.service_timeout)
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
        self.set_stream_rate_srv = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
        self.set_arm_srv = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        self.set_takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff',CommandTOL)
        self.set_land_srv = rospy.ServiceProxy('/mavros/cmd/land',CommandTOL)

        '''set mavros stream rate to get mavros messages faster.
        mavros publishes state/setpoint messages at 1 hz by default
        '''
        #self.set_mavros_stream_rate()

    def set_mavros_stream_rate(self):
        stream_rate = StreamRateRequest()
        stream_rate.request.stream_id = 3
        stream_rate.request.message_rate = 10
        stream_rate.request.on_off = 1
        try:
            self.set_stream_rate_srv(stream_rate)
        except rospy.ServiceException as exp:
            rospy.logerr('Stream rate service failed')
            

    def set_target_pose(self,point,yaw, yaw_rate=1):
        target_pose = PositionTarget()
        target_pose.header.stamp = rospy.get_rostime()
        target_pose.coordinate_frame = 9

        target_pose.position = point
        target_pose.yaw = yaw
        target_pose.yaw_rate = yaw_rate


        target_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE


        self.current_pose = point
        return target_pose


    def extract_yaw_from_quaternion(self, q):
        quaternion = (q.x,q.y,q.z,q.w)
        yaw = euler_from_quaternion(quaternion,axes='sxyz')[2]
        return yaw


    def set_mode(self,mode):
        if self.state.mode != mode:
            try:
                mode_change_response = self.set_mode_srv(base_mode = 0, custom_mode = mode)
                last_request_time = rospy.get_rostime()
                if not mode_change_response.mode_sent:
                    rospy.logerr('---Mode change failed---')    
            except rospy.ServiceException as exception:
                rospy.logerr('Failed to change mode')


    def arm(self):
        last_request_time = rospy.get_rostime()
        if not self.state.armed:
            arm_response = self.set_arm_srv(True)
            if arm_response:
                rospy.loginfo('---Vehicle armed --')
            else:
                rospy.loginfo('---Arming failed ---')
            last_request_time = rospy.get_rostime()
        else:
            #vehicle is already armed
            pass



    def disarm(self):
        if self.set_arm_srv(False):
            rospy.loginfo('--Vehicle disarmed---')
        else:
            rospy.loginfo('---Disarming failed')


    def go_to_position(self,point,OFFSET_ENU=True):
        self.current_target_pose = self.set_target_pose(point,0)


    def return_home(self):
        self.current_target_pose = self.set_target_pose(self.home_pose,0)


    # def set_mode(self,mode):
    #     last_request_time = rospy.get_rostime()
    #     while not rospy.is_shutdown():
    #         now = rospy.get_rostime()
    #         if self.state.mode != "OFFBOARD" and (now - last_request_time > self.offboard_request_threshold):
    #             try:
    #                 mode_change_response = self.set_mode_srv(base_mode = 0, custom_mode = mode)
    #                 last_request_time = now
    #                 if not mode_change_response.mode_sent:
    #                     rospy.logerr('---Mode change failed---')    
    #             except rospy.ServiceException as exception:
    #                 rospy.logerr('Failed to change mode')
    #         else:
    #             rospy.loginfo('--in set_mode---')
    #             self.set_arm(True)
    #         self.current_pose.header.stamp = now
    #         self.setpoint_raw_pub.publish(self.current_pose)
    #         self.rate.sleep()


    # def set_arm(self,should_arm):
    #     now = rospy.get_rostime()
    #     if self.state.mode == "OFFBOARD" and not self.state.armed:
    #         try:
    #             arming_response = self.set_arm_srv(should_arm)
    #             if not arming_response.success:
    #                 rospy.logerr('---Arming/Disarming denied ---')
    #             last_request_time = now
    #         except rospy.ServiceException as exception:
    #             rospy.logerr('Failed to Arm/Disarm')
    #         rospy.loginfo('---setting arm ----')
    #         self.rate.sleep()
    

    # def start_offboard_test(self):
    #     #wait to get heartbeat from fcu
    #     while not self.state.connected:
    #         self.rate.sleep()
        
    #     rospy.loginfo('----FCU connected , Got Heartbcallbackeat -----')
    #     self.set_initial_setpoint(self.pose)
    #     self.set_mode("OFFBOARD")
        # self.set_arm(True)

    '''
    This function will start offboard mode
    '''
    def start_offboard(self):
        # wait to get heartbeat from fcu
        while not self.state.connected:
            self.rate.sleep()
        
        rospy.loginfo('--Got heartbeat from FCU----')

        #before going offboard mode, publish setpoint.
        self.current_target_pose = self.set_target_pose(self.waypoint_1,0)
        for i in range(50,0,-1):
            self.setpoint_raw_pub.publish(self.current_target_pose)
            self.arm()
            self.set_mode("OFFBOARD")
            self.rate.sleep()

        while not rospy.is_shutdown() and self.state.mode == "OFFBOARD" and self.state.armed:
            self.setpoint_raw_pub.publish(self.current_target_pose)
            self.rate.sleep()


    #callback functions
    def altitude_cb(self,data):
        pass     
    
    def state_cb(self,data):
        self.state = data
        self.mode = data.mode

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
        self.current_yaw = self.extract_yaw_from_quaternion(self.imu.orientation)



if __name__=='__main__':
    rospy.init_node('offboard_ros_node',anonymous=True)
    try:
        offboard_control = OffboardControl()
        offboard_control.start_offboard()
        rospy.spin()
    except rospy.ROSInterruptException as exception:
        pass