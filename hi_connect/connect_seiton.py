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
        self.publisher = self.create_publisher(Bool, 'emergency', 100)
        # self.port = serial.tools.list_ports.comports()[1].device
        self.port = '/dev/seiton'
        # self.port = '/dev/ttyACM7'
        print(self.port)
        self.uart = serial.Serial(self.port, 115200,write_timeout=2)
        self.tmr = self.create_timer(0.01, self.callback)
        self.uart.write("s\n".encode('ascii'))
        self.pos = 0
        self.prev = 0
        self.flag = False
        self.g_flag = False

    def joy_callback(self, joy_msg):
        self.get_logger().info('callback')
        # if joy_msg.axes[6] == -1: # right
        #     self.pos = 0
        # elif joy_msg.axes[6] == 1: # left
        #     self.pos = 2
        # elif joy_msg.axes[7] == 1: # up
        #     self.pos = 1
        # elif joy_msg.axes[7] == -1: # down
        #     self.pos = 3
        if joy_msg.axes[6] == -1: # right
            self.pos = 3
        elif joy_msg.axes[6] == 1: # left
            self.pos = 1
        elif joy_msg.axes[7] == 1: # up
            self.pos = 0
        elif joy_msg.axes[7] == -1: # down
            self.pos = 2
        elif joy_msg.axes[2] < 0.5:
            self.pos = 4
            self.flag = True
        elif joy_msg.axes[2] == 1 and self.flag:
            self.pos = 7
            self.flag =False
        elif joy_msg.axes[5] < 0.5:
            self.pos = 5
            self.g_flag = True
        elif joy_msg.axes[5] == 1 and self.g_flag:
            self.pos = 6
            self.g_flag = False
        if joy_msg.buttons[8]:
            msg = Bool()
            msg.data = True
            self.publisher.publish(msg)


    def send(self):
        if self.pos != self.prev:
            message = str(self.pos)+'\n'
            self.get_logger().info('Sent: '+str(self.pos))
            self.uart.write(message.encode('ascii'))
            self.get_logger().info('fin')
            self.flag = False
        self.prev = self.pos

    def receive(self):
        line = self.uart.readline()
        self.readdata = line.decode('utf-8')
        self.get_logger().info(self.readdata[:-1])

    def callback(self):
        self.send()
        # self.receive()
        # self.get_logger().info(str(self.pos))
        pass

    def __del__(self):
        self.uart.write("r\n".encode('ascii'))
        self.uart.close()
        pass


def main():
    rclpy.init()
    con = Connect()
    rclpy.spin(con)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
