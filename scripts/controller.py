#!/usr/bin/env python

import roslib; roslib.load_manifest("chairbot_neato_node")
import rospy, time

from math import sin, cos, fabs
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
from chairbot_neato_driver.chairbot_neato_driver import Botvac
#from neato_controller.msg import NeatoCommand

import socket
import re
global chairbot_number
hostname = socket.gethostname()
chair_id = re.search(r"\d+(\.\d+)?", hostname)
chairbot_number = chair_id.group(0)

class NeatoNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('controller'+chairbot_number, anonymous=True)
        self._port = rospy.get_param('~neato_port', "/dev/neato_port")
        rospy.loginfo("Using port: %s"%(self._port))
        self.ramp_rate=rospy.get_param("~ramp_rate",0.3)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self._robot = Botvac(self._port)
        #self.pub_motor = rospy.Publisher('roboBaseSub', NeatoCommand, queue_size=10)
        self.rate = rospy.get_param("~rate",20)
        self.w = rospy.get_param("~base_width", 0.49)
        self.ramping_enable = rospy.get_param("~ramping_enable", False)
        self.velfactor=rospy.get_param("~mps_to_rpm", 1)
        self.prev_left_vel=0
        self.prev_right_vel=0
        rospy.Subscriber('/neato01/cmd_vel_mux/input/navi', Twist, self.twistCallback)

        #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(20)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
        ###### main loop  ######
        while not rospy.is_shutdown():
            while not rospy.is_shutdown(): # and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
    
    #############################################################
    ########################################################################### 
    def ramp_velocity(self,target_vel,previous_vel,ramp_rate):
        if (target_vel>=previous_vel):
            sign=1
        else:
            sign=-1
        step_size=ramp_rate/self.rate
        delta=fabs(target_vel-previous_vel)
        if delta>=step_size:
            command_vel=previous_vel+sign*step_size
        else:
            command_vel=target_vel
        return command_vel
                
    #############################################################
    def spinOnce(self):
    #############################################################

        self.left = 1.0 * self.dx - self.dr * self.w / 2
        self.right = 1.0 * self.dx + self.dr * self.w / 2
        ###RAMPING SECTION#########
        if self.ramping_enable:
            self.prev_left_vel=self.ramp_velocity(self.left,self.prev_left_vel,self.ramp_rate)
            self.prev_right_vel=self.ramp_velocity(self.right,self.prev_right_vel,self.ramp_rate)
            self.cmdVel[0] =int(self.prev_left_vel*self.velfactor)	
            self.cmdVel[1] =int(self.prev_right_vel*self.velfactor)
        ###########################
        
        self.cmdVel[0] =int(self.left*self.velfactor)	
        self.cmdVel[1] =int(self.right*self.velfactor)
        #############################################################
       
        #self.pub_motor.publish(self.cmdVel)
        self._robot.setMotors(self.cmdVel[0], self.cmdVel[1], 150)
        self.ticks_since_target += 1

    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

if __name__ == "__main__":    
    robot = NeatoNode()
    robot.spin()
