#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import List
from catch2023_interfaces.msg import CreateMessage
import serial
import math
import serial.tools.list_ports


class Connect(Node):
    def __init__(self):
        super().__init__('connect')
        self.subscription = self.create_subscription(CreateMessage, 'degpos_data', self.deg_callback, 10)
        self.port = serial.tools.list_ports.comports()[0].device
        print(self.port)
        self.uart = serial.Serial(self.port, 115200)
        self.deg = [0, 0]
        self.tmr = self.create_timer(0.00001, self.callback)

    def send(self):
        message = str(self.deg[0])+','+str(self.deg[1])+str(self.stepper)+'\n'
        self.uart.write(message.encode('ascii'))

    def deg_callback(self, deg_msg):
        self.deg[0] = math.floor(deg_msg.data.r*100)/100
        self.deg[1] = math.floor(deg_msg.data.theta*100)/100
        self.stepper = deg_msg.data.stepper

    def receive(self):
        line = self.uart.readline()
        self.readdata = line.decode('ascii')

    def callback(self):
        self.send()
        self.receive()
        print(self.readdata,end='')


def main():
    rclpy.init()
    con = Connect()
    rclpy.spin(con)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
