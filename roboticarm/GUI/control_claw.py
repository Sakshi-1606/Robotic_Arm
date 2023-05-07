#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

import tkinter as tk


class SliderController:
    def __init__(self, master, name, topic, min_value, max_value, color):
        # Create a publisher to publish the slider value to the joint controller
        self.pub = rospy.Publisher(topic, Float64, queue_size=10)

        # Create a horizontal slider
        self.slider = tk.Scale(master, from_=min_value, to=max_value, orient=tk.HORIZONTAL, length=300, width=20, sliderlength=10, bg=color, command=self.slider_callback)
        self.slider.pack(pady=5)

        # Create a label with the controller name
        self.label = tk.Label(master, text=name)
        self.label.pack()

    def slider_callback(self, value):
        # Convert the slider value to radians
        angle = float(value) * 3.14159 / 180.0

        # Publish the angle to the joint controller
        self.pub.publish(Float64(angle))

class RoboticArmController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robotic_arm_controller')

        # Create a Tkinter GUI window
        self.root = tk.Tk()
        self.root.title("Robotic Arm Controller")

        # Set the padding around window
        self.root.geometry("+100+100")

        # Create individual sliders for each joint
        controller1 = SliderController(self.root, "Controller 1", "/roboticarm/joint1_position_controller/command", -180, 180, "red")

    def run(self):
        # Start the Tkinter main loop
        self.root.mainloop()


if __name__ == '__main__':
    controller = RoboticArmController()
    controller.run()
