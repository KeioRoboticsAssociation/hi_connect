#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import List
from catch2023_interfaces.msg import CreateMessage
from std_msgs.msg import Bool
import serial
import math
import serial.tools.list_ports


class Connect(Node):
    def __init__(self):
        super().__init__('connect')
        self.subscription = self.create_subscription(CreateMessage, 'degpos_data', self.deg_callback, 100)
        self.publisher = self.create_publisher(CreateMessage, 'real_pos', 100)
        self.emg = self.create_subscription(Bool, 'emergency', self.emg_callback, 100)
        self.port = serial.tools.list_ports.comports()[0].device
        print(self.port)
        self.uart = serial.Serial(self.port, 115200)
        self.deg = [0, 0]
        self.stepper = 0
        self.hand = 0
        self.armtheta = 0
        self.catch = False
        self.tmr = self.create_timer(0.001, self.callback)
        self.uart.write("s\n".encode('ascii'))

    def send(self):
        message = str(int(self.stepper))+' '+str(int(self.catch))+' '+str(self.hand)+' '+str(self.armtheta)+' '+str(self.deg[0])+' '+str(self.deg[1])+'\n'
        self.uart.write(message.encode('ascii'))

    def deg_callback(self, deg_msg):
        self.deg[0] = math.floor(deg_msg.theta*100)/100
        self.deg[1] = math.floor(deg_msg.r*100)/100
        self.stepper = deg_msg.stepper
        self.hand = 1 if (deg_msg.hand == 45 or deg_msg.hand == -45) else 0
        self.armtheta = int(deg_msg.armtheta)
        self.catch = deg_msg.judge

    def emg_callback(self, emg_msg):
        if emg_msg.data:
            self.get_logger().info('RECEIVED EMERGENCY')
            self.__del__()
            exit()
        return

    def receive(self):
        line = self.uart.readline()
        self.readdata = line.decode('utf-8')
        real_pos = CreateMessage()
        try:
            real_pos.theta = float(self.readdata.split(',')[0])
            real_pos.r = float(self.readdata.split(',')[1])
            real_pos.stepper = int(self.readdata.split(',')[2])
        except:
            pass
        self.publisher.publish(real_pos)

    def callback(self):
        self.send()
        self.receive()
        self.get_logger().info(self.readdata[:-1])

    def __del__(self):
        self.uart.write("r\n".encode('ascii'))
        self.uart.close()


def main():
    rclpy.init()
    con = Connect()
    rclpy.spin(con)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
