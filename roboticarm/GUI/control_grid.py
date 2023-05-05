#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

import tkinter as tk

class SliderController:
    def __init__(self, master, name, topic):
        # Create a publisher to publish the slider value to the joint controller
        self.pub = rospy.Publisher(topic, Float64, queue_size=10)

        # Create a horizontal slider
        self.slider = tk.Scale(master, from_=0, to=360, orient=tk.HORIZONTAL, length=200, width=20, sliderlength=10, bg="black", command=self.slider_callback)
        self.slider.grid(row=0, column=0, columnspan=1, pady=5)

        # Create a label with the controller name
        self.label = tk.Label(master, text=name)
        self.label.grid(row=1, column=0, sticky="E")

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

        # Set the padding between rows and columns
        pad = 100

        # Create a 3x3 grid of slider controllers
        for row in range(3):
            for col in range(3):
                # Calculate the controller name and topic based on the row and column
                name = f"Controller{row*3+col+1}"
                topic = f"/roboticarm/joint{row*3+col+1}_position_controller/command"
                controller = SliderController(self.root, name, topic)

                # Add the controller to the grid with padding
                controller.label.grid(row=row+1, column=col*2, padx=pad, pady=pad, sticky="E")
                controller.slider.grid(row=row+1, column=col*2+1, padx=pad, pady=pad, sticky="W")

        # Set the padding around window
        self.root.geometry(f"+{pad}+{pad}")

    def run(self):
        # Start the Tkinter main loop
        self.root.mainloop()

if __name__ == '__main__':
    controller = RoboticArmController()
    controller.run()
