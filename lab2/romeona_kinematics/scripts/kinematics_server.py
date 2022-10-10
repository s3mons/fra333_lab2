#!/usr/bin/python3

# import all other neccesary libraries here
from tokenize import String
from turtle import forward
import numpy as np
import math
from argparse import Namespace
from platform import node
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dummy_kinematics_interfaces.srv import GetPosition
from dummy_kinematics_interfaces.srv import SolveIK
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from romeona_kinematics.forward_kinematics import forward_kinematics
from romeona_kinematics.inverse_kinematics import inverse_kinematics

class Kinematics_Server(Node):

    def __init__(self):
        super().__init__('kinematics_server')
        # get the rate from argument or default
        self.rate = 10.0
        
        # Create Service 
        self.joint_states = self.create_service(GetPosition,'set_joint',self.set_joint_callback)
        self.joint_states_ik = self.create_service(SolveIK,'solve_ik',self.set_joint_callback_ik)

        # Create Publisher
        self.command_publisher = self.create_publisher(JointState,'/joint_states',10)
        
        # Publish
        timer_period = 1/self.rate
        self.timer = self.create_timer(timer_period,self.timer_callback)

        # additional attributes
        self.joint_state_position = JointState()
        self.joint_state_position.name = ['joint_1' ,'joint_2' ,'joint_3']
        self.joint_state_position.position = [0. , 0. , 0.]


    def set_joint_callback(self,request:GetPosition.Request,response:GetPosition.Response):

        self.joint_state_position.position[0] = request.q.position[0]
        self.joint_state_position.position[1] = request.q.position[1]
        self.joint_state_position.position[2] = request.q.position[2]

        fk = forward_kinematics(self.joint_state_position.position)
        position = Point()
        position.x = fk[0][3] 
        position.y = fk[1][3] 
        position.z = fk[2][3] 
        response.p = position

        return response
    
    # Inverse Kinematics
    def set_joint_callback_ik(self,request,response:SolveIK.Response):

        self.joint_state_position.position[0] = request.point.x
        self.joint_state_position.position[1] = request.point.y
        self.joint_state_position.position[2] = request.point.z
        gamma = request.gamma

        ik,flag = inverse_kinematics(self.joint_state_position.position,gamma)
        self.joint_state_position.position = [ik[0],ik[1],ik[2]]
        # self.joint_state_position.position[1] = ik[1]
        # self.joint_state_position.position[2] = ik[2]

        response.q = self.joint_state_position

        response.flag.data = flag

        return response 

    def timer_callback(self):
        
        now = self.get_clock().now()
        self.joint_state_position.header.stamp = now.to_msg()

        self.command_publisher.publish(self.joint_state_position)
        # print(self.rate)

    

def main(args=None):
    # remove pass and add codes here
    rclpy.init(args=args)
    joint_state = Kinematics_Server()
    print("Start")
    rclpy.spin(joint_state)
    joint_state.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
