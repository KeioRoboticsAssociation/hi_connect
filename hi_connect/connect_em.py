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
        super().__init__('connect_em')
        
        self.port = serial.tools.list_ports.comports()[0].device
        self.publisher = self.create_publisher(Bool,'emergency', 100)
        print(self.port)
        self.uart = serial.Serial(self.port, 115200)
        self.tmr = self.create_timer(0.001, self.callback)
        # self.uart.write("s\n".encode('ascii'))
        self.pos = 0
        self.prev = 0


    def receive(self):
        line = self.uart.readline()
        self.readdata = line.decode('utf-8')
        self.get_logger().info(self.readdata[:-1])
        if 'Received 2' in self.readdata:
            msg = Bool()
            msg.data = True
            self.publisher.publish(msg)

    def callback(self):
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
