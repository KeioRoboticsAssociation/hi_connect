#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import List
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import serial
import math
import serial.tools.list_ports


class Connect(Node):
    def __init__(self):
        super().__init__('connect_seiton')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 100)
        # self.port = serial.tools.list_ports.comports()[1].device
        self.port = '/dev/seiton'
        print(self.port)
        self.uart = serial.Serial(self.port, 115200)
        self.tmr = self.create_timer(0.001, self.callback)
        # self.uart.write("s\n".encode('ascii'))
        self.pos = 0
        self.prev = 0

    def joy_callback(self, joy_msg):
        if joy_msg.axes[6] == -1:
            self.pos = 0
        elif joy_msg.axes[6] == 1:
            self.pos = 2
        elif joy_msg.axes[7] == 1:
            self.pos = 1
        elif joy_msg.axes[7] == -1:
            self.pos = 3

    def send(self):
        if self.pos != self.prev:
            message = str(self.pos)+'\n'
            self.get_logger().info('Sent: '+str(self.pos))
            self.uart.write(message.encode('ascii'))
        self.prev = self.pos

    def receive(self):
        line = self.uart.readline()
        self.readdata = line.decode('utf-8')
        self.get_logger().info(self.readdata[:-1])

    def callback(self):
        self.send()
        self.receive()
        # self.get_logger().info(str(self.pos))
        pass

    def __del__(self):
        # self.uart.write("r\n".encode('ascii'))
        # self.uart.close()
        pass


def main():
    rclpy.init()
    con = Connect()
    rclpy.spin(con)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
