#!/usr/bin/env python3
from cmath import pi
from email.header import Header
from shutil import move
from time import struct_time
import rclpy
from rclpy.node import Node
from typing import List
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Header
# from cobs import cobs
import serial
import struct
import math
import serial.tools.list_ports


class Connect(Node):
    def __init__(self):
        super().__init__('connect')
        self.subscription = self.create_subscription(Float32MultiArray, 'degpos_data', self.deg_callback, 10)
        self.port = serial.tools.list_ports.comports()[0].device
        print(self.port)
        self.uart = serial.Serial(self.port, 115200)
        self.deg = [0, 0]
        self.tmr = self.create_timer(0.1, self.callback)

    def send(self):
        message = str(self.deg[0])+','+str(self.deg[1])+'\n'
        self.uart.write(message.encode('ascii'))

    def deg_callback(self, deg_msg):
        self.deg[0] = deg_msg.data[0]
        self.deg[1] = deg_msg.data[1]

    def receive(self):
        line = self.uart.readline()
        self.readdata = line.decode('ascii')

    def callback(self):
        self.send()
        self.receive()
        print(self.readdata)


def main():
    rclpy.init()
    con = Connect()
    rclpy.spin(con)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
