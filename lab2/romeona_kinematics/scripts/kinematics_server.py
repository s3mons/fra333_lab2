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
from romeona_kinematics_interfaces.srv import GetPosition
from romeona_kinematics_interfaces.srv import SolveIK
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from romeona_kinematics.forward_kinematics import forward_kinematics
from romeona_kinematics.inverse_kinematics import inverse_kinematics

class Kinematics_Server(Node):

    def __init__(self):
        super().__init__('kinematics_server')
        # get the rate from argument or default
        self.rate = 10.0
        
        # Create Service for Forward Kinematic
        self.joint_states = self.create_service(GetPosition,'set_joint',self.set_joint_callback)

        # Create Service for Inverse Kinematic
        self.joint_states_ik = self.create_service(SolveIK,'solve_ik',self.set_joint_callback_ik)

        # Create Publisher
        self.command_publisher = self.create_publisher(JointState,'/joint_states',10)
        
        # Publish
        timer_period = 1/self.rate
        self.timer = self.create_timer(timer_period,self.timer_callback)

        # additional attributes
        self.joint_state_position = JointState() # สร้างตัวแปรในรูปของ JointState() ที่เป็น sensor_msgs
        self.joint_state_position.name = ['joint_1' ,'joint_2' ,'joint_3'] # ตั้งชื่อให้เหมือนในไฟล์ xacro ทำให้ name เหมือนกันทำให้ควบคุมกันได้
        self.joint_state_position.position = [0. , 0. , 0.] # ตั้งค่ามุมในหน่วย rad เป็น default ที่ 0 rad ทั้ง 3 joints


    def set_joint_callback(self,request:GetPosition.Request,response:GetPosition.Response):

        # นำค่าที่รับ request ซึ่งเป็นค่า rad มาเก็บเข้าตัวแปร joint_state_position ในหัวข้อ position
        self.joint_state_position.position[0] = request.q.position[0]
        self.joint_state_position.position[1] = request.q.position[1]
        self.joint_state_position.position[2] = request.q.position[2]

        # นำค่าที่รับเข้ามาไปทำการ Forward Kinematic เพื่อหาตำแหน่งปลายเทียบกับฐานของหุ่นยนต์
        fk = forward_kinematics(self.joint_state_position.position)

        # สร้างตัวแปรที่เป็น Point() ที่เป็น geometry_msg ซึ่งจะประกอบไปด้วยค่า x y z และนำค่า x y z ที่ได้จากการคำนวณ FK มาเก็บไว้ในตัวแปรนี้เพื่อที่จะนำไปแสดง response  
        position = Point()
        position.x = fk[0][3] 
        position.y = fk[1][3] 
        position.z = fk[2][3] 
        response.p = position

        return response
    
    # Inverse Kinematics
    def set_joint_callback_ik(self,request:SolveIK.Request,response:SolveIK.Response):

        # นำค่าที่รับ request ซึ่งเป็นค่า x y z มาเก็บเข้าตัวแปร joint_state_position ในหัวข้อ position
        # และเพิ่มค่า gamma เข้ามาซึ่งเป็นค่าที่จะกำหนด pose ของหุ่นยนต์
        self.joint_state_position.position[0] = request.point.x
        self.joint_state_position.position[1] = request.point.y
        self.joint_state_position.position[2] = request.point.z
        gamma = request.gamma

        # นำค่าที่เก็บไว้ไปทำการคำนวณ Inverse kinematic เพื่อหาค่ามุมที่แขนหุ่นยนต์ต้องทำซึ่งจะได้ค่า q_1 q_2 q_3 แลพ flag ที่จะบอกว่าหุ่นยนต์สามารถทำงานใน task space ได้หรือไม่
        # และนำค่า q_1 q_2 q_3 ที่ได้จากการคำนวณมาเก็บเข้าตัวแปร joint_state_position ในหัวข้อ position เพื่อนำไป Publish ต่อไป
        ik,flag = inverse_kinematics(self.joint_state_position.position,gamma)
        self.joint_state_position.position = [ik[0],ik[1],ik[2]]

        # ค่าที่ได้จากการคำนวณและ flag จะเป็นค่าที่จะทำการ response ด้วย
        response.q = self.joint_state_position
        response.flag.data = flag

        return response 

    def timer_callback(self):
        
        # ใน JointState() จำเป็นที่จะต้องมีการกำหนดเวลา 
        now = self.get_clock().now()
        self.joint_state_position.header.stamp = now.to_msg()

        # Publish ค่ามุมในหน่วย rad ไปที่ node robotstate_publisher เพื่อที่จะได้แสดงการจำลองผ่านจาก rviz
        self.command_publisher.publish(self.joint_state_position)


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
