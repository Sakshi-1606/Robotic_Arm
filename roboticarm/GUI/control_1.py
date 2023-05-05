#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import tkinter as tk
from tkinter import ttk

class Controller:
    def __init__(self):
        self.publisher = rospy.Publisher('/joint1_position_controller', Float64, queue_size=10)
        self.root = tk.Tk()
        self.create_gui()
        
    def create_gui(self):
        self.root.title("ROS Controller")
        self.scale = ttk.Scale(self.root, from_=0.0, to=100.0, orient=tk.HORIZONTAL, command=self.send_command)
        self.scale.pack(expand=True, fill=tk.BOTH, padx=10, pady=10)
        
    def send_command(self, value):
        self.publisher.publish(float(value))
        
    def run(self):
        self.root.mainloop()
        
if __name__ == '__main__':
    rospy.init_node('controller_node')
    controller = Controller()
    controller.run()
