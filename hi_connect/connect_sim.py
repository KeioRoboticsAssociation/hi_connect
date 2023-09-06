#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import List
from catch2023_interfaces.msg import CreateMessage
from std_msgs.msg import Bool
import serial
import math
import serial.tools.list_ports
import time


class Connect(Node):
    def __init__(self):
        super().__init__('connect')
        self.subscription = self.create_subscription(CreateMessage, 'degpos_data', self.deg_callback, 100)
        self.publisher = self.create_publisher(CreateMessage, 'real_pos', 100)
        self.get_logger().info("SIMULATION MODE")
        self.get_logger().info("s")
        self.get_logger().info("homing")
        
        time.sleep(2)
        
        # self.port = serial.tools.list_ports.comports()[0].device
        # print(self.port)
        # self.uart = serial.Serial(self.port, 115200)
        self.deg = [0, 0]
        self.currentDeg = [0, 0]
        self.stepper = 0
        self.hand = 0
        self.armtheta = 0
        self.catch = False
        self.tmr = self.create_timer(0.05, self.callback)
        self.cnt=0
        # self.uart.write("s\n".encode('ascii'))

    def send(self):
        message = str(int(self.stepper))+' '+str(int(self.catch))+' '+str(self.hand) + \
            ' '+str(self.armtheta)+' '+str(self.deg[0])+' '+str(self.deg[1])+'\n'
        # self.uart.write(message.encode('ascii'))

    def deg_callback(self, deg_msg):
        self.deg[0] = math.floor(deg_msg.theta*100)/100
        self.deg[1] = math.floor(deg_msg.r*100)/100
        # self.stepper = deg_msg.stepper
        if(self.cnt>10):
            self.stepper=deg_msg.stepper
            self.cnt=0
        self.hand = 1 if (deg_msg.hand == 45 or deg_msg.hand == -45) else 0
        self.armtheta = int(deg_msg.armtheta)
        self.catch = deg_msg.judge
        self.cnt+=1

    def catch_callback(self, catch_msg):
        self.catch = catch_msg.data

    def receive(self):
        # line = self.uart.readline()
        # self.readdata = line.decode('utf-8')
        if self.currentDeg[0] < self.deg[0]:
            self.currentDeg[0] += abs(self.deg[0]-self.currentDeg[0])/10 if abs(self.deg[0]-self.currentDeg[0]) < 3 else 3
        elif self.currentDeg[0] > self.deg[0]:
            self.currentDeg[0] -= abs(self.deg[0]-self.currentDeg[0])/10 if abs(self.deg[0]-self.currentDeg[0]) < 3 else 3
            
        if self.currentDeg[1] < self.deg[1]:
            self.currentDeg[1] += abs(self.deg[1]-self.currentDeg[1])/10 if abs(self.deg[1]-self.currentDeg[1]) < 3 else 3
        elif self.currentDeg[1] > self.deg[1]:
            self.currentDeg[1] -= abs(self.deg[1]-self.currentDeg[1])/10 if abs(self.deg[1]-self.currentDeg[1]) < 3 else 3
        
        self.currentDeg[0]=math.floor(self.currentDeg[0]*10)/10
        self.currentDeg[1]=math.floor(self.currentDeg[1]*10)/10
        self.readdata = str(self.currentDeg[0])+', '+str(self.currentDeg[1])+', '+str(int(self.catch))+', '+str(self.hand)+', ' + \
            str(self.armtheta)+', '+str(self.deg[0])+', '+str(self.deg[1])+', xxxx, 1, 1, 1, 1'+'\n'
        real_pos = CreateMessage()
        try:
            real_pos.theta = self.currentDeg[0]
            real_pos.r = self.currentDeg[1]
            real_pos.stepper = self.stepper
        except:
            pass
        self.publisher.publish(real_pos)

    def callback(self):
        self.send()
        self.receive()
        self.get_logger().info(self.readdata[:-1])

    def __del__(self):
        pass
        # self.uart.write("r\n".encode('ascii'))
        # self.uart.close()


def main():
    rclpy.init()
    con = Connect()
    rclpy.spin(con)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
