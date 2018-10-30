#! /usr/bin/env python

import math
import sys
import time

import rospy
import serial
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class BaseControl(object):
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = long(rospy.get_param("~baud", "115200"))
        
        self.baseId = rospy.get_param("~base_id", "base_footprint")
        self.odomId = rospy.get_param("~odom_id", "odom")
        self.odomTopic = rospy.get_param("~odom_topic", "odom")
        self.odom_freq = float(rospy.get_param("~odom_freq", "20"))
        self.cmdTopic = rospy.get_param("~cmd_topic", "cmd_vel")
        self.cmd_freq = float(rospy.get_param("~cmd_freq", "10"))
        self.imuTopic = rospy.get_param("~imu_topic", "imu")

        self.wheelSep = float(rospy.get_param("~wheel_separation", "0.47"))
        self.wheelRad = float(rospy.get_param("~wheel_radius", "0.08"))

        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=10)
        except Exception as e:
            rospy.logerr("Cannot connect to port: " + self.port + ".")
            rospy.logerr(e)
            # sys.exit(0)
        rospy.loginfo("Communication success!")

        self.sub_cmd = rospy.Subscriber(
            self.cmdTopic, Twist, self.cmdvelCB, queue_size=10)
        self.sub_imu = rospy.Subscriber(
            self.imuTopic, Imu, self.imuCB, queue_size=10)
        self.pub_odom = rospy.Publisher(self.odomTopic, Odometry, queue_size=10)

        self.timer_cmd = rospy.Timer(rospy.Duration(
            1.0/self.cmd_freq), self.timerCmdCB)
        # self.pub_cmd = rospy.Publisher()
        self.timer_odom = rospy.Timer(rospy.Duration(
            1.0/self.odom_freq), self.timerOdomCB)

        self.trans_x = 0.0
        self.rotat_z = 0.0
        self.vel_wheel_r = 0.0
        self.vel_wheel_l = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_th = 0.0
        self.current_time = rospy.Time.now()
        self.previous_time = rospy.Time.now()

    def cmdvelCB(self, msg):
        self.trans_x = msg.linear.x
        self.rotat_z = msg.angular.z

    def imuCB(self, msg):
        quat = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.pose_th = euler[2]
        # rospy.loginfo("{}, {}, {}".format(euler[0],euler[1],euler[2]))


    def timerCmdCB(self, event):
        def constrain(value, value_min, value_max):
            return max(min(value_max, value), value_min)
        # self.sendWL = self.constrain(
        #     (self.trans_x - self.wheelSep/2.0*self.rotat_z)/self.wheelRad, -self.MAX_W, self.MAX_W)
        # self.sendWR = self.constrain(
        #     (self.trans_x + self.wheelSep/2.0*self.rotat_z)/self.wheelRad, -self.MAX_W, self.MAX_W)
        self.sendWL = -(self.trans_x - self.wheelSep /
                       2.0*self.rotat_z)/self.wheelRad
        self.sendWR = (self.trans_x + self.wheelSep /
                       2.0*self.rotat_z)/self.wheelRad

        speedL = constrain(int(self.sendWL * 10) & 0xFF, 0, 255)
        speedR = constrain(int(self.sendWR * 10) & 0xFF, 0, 255)
        # rospy.loginfo("#{},{},  {}, {}".format(speedR, speedL,self.vel_wheel_r,self.vel_wheel_l))
        
        command = [255,speedR,speedL,254]
        self.serial.write(command)

    def timerOdomCB(self, event):
        speedL = -self.vel_wheel_l * self.wheelRad
        speedR = self.vel_wheel_r * self.wheelRad
        speedTh = (speedR + speedL)/self.wheelSep
        speedX = (speedR + speedL)/2.0

        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.previous_time).to_sec()
        self.previous_time = self.current_time
        self.pose_x += (speedX * math.cos(self.pose_th) * dt)
        self.pose_y += (speedX * math.sin(self.pose_th) * dt)
        # self.pose_th += (speedTh * dt)
        pose_quat = tf.transformations.quaternion_from_euler(
            0, 0, self.pose_th)
        
        msg = Odometry()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.odomId
        msg.child_frame_id = self.baseId
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.orientation.x = pose_quat[0]
        msg.pose.pose.orientation.y = pose_quat[1]
        msg.pose.pose.orientation.z = pose_quat[2]
        msg.pose.pose.orientation.w = pose_quat[3]
        msg.twist.twist.linear.x = speedX
        msg.twist.twist.angular.z = speedTh
       
        for i in range(36):
            msg.twist.covariance[i] = 0
        msg.twist.covariance[0] = 1.0  # speedX Cov
        msg.twist.covariance[35] = 1.0  # speedTh Cov
        msg.pose.covariance = msg.twist.covariance
        self.pub_odom.publish(msg)


def main():
    try:
        rospy.init_node("base_serial_control")
        rospy.loginfo("Base Control ...")
        bc = BaseControl()

        while not rospy.is_shutdown():
            if bc.serial.readable():
                if bc.serial.read() == b'\xff':
                    byte = ord(bc.serial.read())
                    bc.vel_wheel_r = (256-byte) * (-1/10.0) if byte > 127 else byte/10.0
                    byte = ord(bc.serial.read())
                    bc.vel_wheel_l = (256-byte) * (-1/10.0) if byte > 127 else byte/10.0
                    bc.serial.read()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
