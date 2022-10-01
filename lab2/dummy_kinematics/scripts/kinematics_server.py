#!/usr/bin/python3

# import all other neccesary libraries here
from tokenize import String
import numpy as np
from argparse import Namespace
from platform import node
import sys

import rclpy
from rclpy.node import Node
from dummy_kinematics_interfaces.srv import SetJoint
# from std_msgs.msg import Float64


class Kinematics_Server(Node):

    def __init__(self):
        # get the rate from argument or default
        self.rate = 10.0

        # add codes here

        # Node
        super().__init__('kinematics_server')

        # Create Service 
        self.joint_state = self.create_service(SetJoint,'set_joint',self.set_joint_callback)

        # Create Publisher
        # self.command_publisher = self.create_publisher(Float64,'noise',10)

        # Publish
        timer_period = 1/self.rate
        self.timer = self.create_timer(timer_period,self.timer_callback)


        # additional attributes
        self.mean = 0.0
        self.variance = 1.0
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()} with the default parameter. mean: {self.mean}, variance: {self.variance}')
    
    def set_joint_callback(self,request:SetJoint.Request,response:SetJoint.Response):

        self.mean = request.setjoints
        self.variance = request.variance.data**(1/2)
        # self.get_logger().info('Incoming request\nmean: %d variance: %d' % (request.mean, request.variance))
  
        return response
    
    def timer_callback(self):
        # remove pass and add codes here
        msg = Float64()
        msg.data = np.random.normal(self.mean, self.variance)
        self.command_publisher.publish(msg)
        print(self.rate)
        

def main(args=None):
    # remove pass and add codes here
    rclpy.init(args=args)
    noise_gen = NoiseGenerator()
    print("Start")
    rclpy.spin(noise_gen)
    noise_gen.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
