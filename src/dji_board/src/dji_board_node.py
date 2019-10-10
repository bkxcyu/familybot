#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
import serial
import string
import math
import sys
import json
import time
from geometry_msgs.msg import Twist

rospy.init_node("dji_board_node")
def callback(Twist):
    rospy.loginfo("Sending cmd:[%.2f,%.2f,%.2f]", Twist.linear.x,Twist.linear.y,Twist.angular.z)
    x=38.61*Twist.linear.x+1.27
    y=38.61*Twist.linear.y+1.27
    z=38.61*Twist.angular.z+1.27
    send_command(-int(y),int(x),int(z))


pub = rospy.Publisher('familybot_velocities', Twist, queue_size=1)
sub = rospy.Subscriber("cmd_vel", Twist, callback)
sub1= rospy.Subscriber("teleop_cmd_vel", Twist, callback)

port='/dev/dji_board'
rospy.loginfo("Opening %s...", port)
#控制电机速度
def send_command(vx,vy,vz):
    data = { "speed":[vx,vy,vz] }
    data_string = json.dumps(data)
    ser.write('*'+ data_string +';')

try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=5)
except serial.serialutil.SerialException:
    rospy.logerr("Dji not found at port "+port + ". Did you specify the correct port in the launch file?")
    sys.exit(0)

#ser.write('t\n')


while not rospy.is_shutdown():
    line = ser.readline()
    #rospy.loginfo("I have got chassis velicities: " + line)
    #解析字符串为python 对象
    try:
        de_json = json.loads(line)
        linear_x = de_json[1][0]
        linear_y = de_json[1][1]
        angular_z = de_json[1][2]
        # rospy.loginfo("get_feedback:[" + str(linear_x) + "," + str(linear_y)+ "," + str(angular_z) + "]\n")
        twist = Twist()
        twist.linear.x = float(linear_x); twist.linear.y = float(linear_y); twist.linear.z = float(angular_z)
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
    except:
        None
    #rospy.loginfo(de_json)
    #控制电机速度
    # speed_x = 0
    # speed_y = 30
    # speed_z = 0
    # send_command(speed_x,speed_y,speed_z)
#关闭节点时停止电机
#time.sleep(1)
send_command(0,0,0)
ser.close

#关闭节点时先按control-s再按control-c
