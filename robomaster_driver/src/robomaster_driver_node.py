#!/usr/bin/env python3
import rospy
from robomaster import robot, led
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, JointState
from nav_msgs.msg import Odometry
from threading import Thread
from cv_bridge import CvBridge
import numpy as np
import tf
from pyquaternion import Quaternion


def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


class RobomasterNode:
    def __init__(self):
        # Initialize tf
        self._tf_br = tf.TransformBroadcaster()

        # Initialize robot and set chassis lead
        conn_mode = rospy.get_param("~connection_mode", "WIFI")  # WIFI, USB,AP
        self._robot = robot.Robot()
        try:
            if conn_mode == "WIFI":
                self._robot.initialize(conn_type="sta")
            elif conn_mode == "USB":
                self._robot.initialize(conn_type="rndis")
            elif conn_mode == "AP":
                self._robot.initialize(conn_type="ap")          
            else:
                rospy.logerr("Unknown connection mode {}".format(conn_mode))
                return
                    
        except:
            rospy.logerr("Could not connect to robot, shutting down.")
            return
        rospy.loginfo("Connected to robot.")
        # self._robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
        self._robot.set_robot_mode(mode="free")
        self._robot.gimbal.recenter()

        # Mark ROS mode
        self._robot.led.set_led(comp=led.COMP_ALL, r=255, g=0, b=255, effect=led.EFFECT_ON)

        # Sub to cam
        self._cv_bridge = CvBridge()
        self._img_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=3)
        self._robot.camera.start_video_stream(display=False)
        self._cam_thread = Thread(target=self._cam_loop)
        self._cam_thread.start()

        # Prepare gimbal state publishing
        self._pub_joints = rospy.Publisher("/joint_states", JointState, queue_size=3)
        self._robot.gimbal.sub_angle(freq=50, callback=self._gimbal_angle_cb)
        
        # Prepare odometry publishing
        self._init_orientation = None
        self._pub_odom_tf = rospy.get_param("~publish_odom_tf", True)
        self._odom_include_attitude = rospy.get_param("~odom_include_attitude", True)
        self._pub_odom = rospy.Publisher("/odom", Odometry, queue_size=3)
        # self._robot.chassis.sub_velocity(freq=50, callback=self._vel_cb)
        self._robot.chassis.sub_position(cs=1, freq=50, callback=self._odom_cb)

        # Prepare IMU publishing
        self._last_attitude = None
        self._robot.chassis.sub_attitude(freq=50, callback=self._attitude_cb)
        self._pub_imu = rospy.Publisher("/imu/data", Imu, queue_size=3)
        self._robot.chassis.sub_imu(freq=50, callback=self._imu_cb)

        # Subscribe to twist topic
        self._standing = True
        self._sub = rospy.Subscriber("/cmd_vel", Twist, self._twist_cb, queue_size=3)
        
    # def _vel_cb(self, data):
    #     print(data)

    def _odom_cb(self, data):
        if self._last_attitude is None or self._init_orientation is None:
            return
        x, y, z = data
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = -y
        if self._odom_include_attitude:
            q = self._last_attitude * self._init_orientation.inverse
        else:
            q = Quaternion(axis=(0, 0, 1), radians=self._last_attitude.yaw_pitch_roll[0])
        msg.pose.pose.orientation.x = q.x
        msg.pose.pose.orientation.y = q.y
        msg.pose.pose.orientation.z = q.z
        msg.pose.pose.orientation.w = q.w
        self._pub_odom.publish(msg)

        if self._pub_odom_tf:
            self._tf_br.sendTransform((x, -y, 0),
                     (q.x, q.y, q.z, q.w),
                     msg.header.stamp,
                     "base_link",
                     "odom")
    
    def _imu_cb(self, data):
        ax, ay, az, wx, wy, wz = data
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"
        msg.linear_acceleration.x = ax*9.861980434
        msg.linear_acceleration.y = -ay*9.861980434
        msg.linear_acceleration.z = -az*9.861980434
        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz
        q = self._last_attitude * self._init_orientation.inverse
        msg.orientation.x = q.x
        msg.orientation.y = q.y
        msg.orientation.z = q.z
        msg.orientation.w = q.w
        self._pub_imu.publish(msg)
    
    def _gimbal_angle_cb(self, data):
        pitch, yaw, _, _ = data
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["gimbal_yaw_joint", "gimbal_pitch_joint"]
        msg.position = [-yaw/180.0*np.pi, -pitch/180.0*np.pi]
        self._pub_joints.publish(msg)

    def _attitude_cb(self, data):
        yaw, pitch, roll = data
        self._last_attitude = Quaternion(axis=(0, 0, 1), degrees=-yaw)*Quaternion(axis=(0, 1, 0), degrees=-pitch)*Quaternion(axis=(1, 0, 0), degrees=roll)

        if self._init_orientation is None:
            self._init_orientation = self._last_attitude

    def _twist_cb(self, msg):
        self._robot.gimbal.drive_speed(pitch_speed=msg.angular.x,yaw_speed=msg.angular.y)

        if msg.linear.x != 0 or msg.linear.y != 0 or msg.angular.z != 0:
            self._standing = False
        
        if msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
            self._robot.chassis.drive_speed(x=0, y=0, z=0, timeout=1)
            self._standing = True

        if not self._standing:
            self._robot.chassis.drive_speed(x=msg.linear.x, y=-msg.linear.y, z=-msg.angular.z/np.pi*180.0, timeout=1)
            # self._robot.chassis.drive_speed(x=msg.linear.x, y=-msg.linear.y, z=0, timeout=1)
            self._robot.gimbal.drive_speed(pitch_speed=msg.angular.x,yaw_speed=msg.angular.y)

            # self._robot.chassis.drive_speed(x=0, y=0, z=-msg.linear.x*10/np.pi*180.0, timeout=1)
            # self._robot.gimbal.drive_speed(pitch_speed=msg.angular.x,yaw_speed=msg.angular.y)


    def _cam_loop(self):
        while not rospy.is_shutdown():
            try:
                tsn = rospy.Time.now()
                img = self._robot.camera.read_cv2_image(timeout=0.1, strategy='newest')
                tsn2 = rospy.Time.now()
                msg = self._cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
                msg.header.stamp = tsn + (tsn2-tsn)/2.0
                msg.header.frame_id = "camera_link_optical_frame"
                self._img_pub.publish(msg)
            except:
                pass
    
    def _controller_envent(self,msg):
        if msg.linear.x ==1 :
            self._robot.blaster.firle(fire_type="ir", times=1)
        if msg.angular.x ==1 :
            self._robot.chassis.stick_overlay(0)
        if msg.angular.y ==1 :
            self._robot.chassis.stick_overlay(1)
        if msg.angular.z ==1 :
            self._robot.chassis.stick_overlay(2)
        

    def shutdown(self):
        self._robot.camera.stop_video_stream()
        self._robot.close()


if __name__ == "__main__":
    rospy.init_node("robomaster_driver")
    node = RobomasterNode()

    rospy.spin()
    node.shutdown()