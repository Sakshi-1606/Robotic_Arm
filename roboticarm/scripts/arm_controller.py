#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

import tkinter as tk


class SliderController:
    def __init__(self, master, name, topic, min_value, max_value, color):
        # Create a publisher to publish the slider value to the joint controller
        self.pub = rospy.Publisher(topic, Float64, queue_size=10)

        # Create a label with the controller name
        self.label = tk.Label(master, text=name)
        self.label.pack()

        # Create a horizontal slider
        self.slider = tk.Scale(master, from_=min_value, to=max_value, orient=tk.HORIZONTAL, length=300, width=20, sliderlength=10, bg=color, fg="snow", command=self.slider_callback)
        self.slider.pack()

        # Create a small space between the slider and next label
        self.space = tk.Label(master, text=" ")
        self.space.pack()

    def slider_callback(self, value):
        # Convert the slider value to radians
        angle = float(value) * 3.14159 / 180.0

        # Publish the angle to the joint controller
        self.pub.publish(Float64(angle))


class ClawController:
    def __init__(self, master, name, open_value, close_value):
        # Create a publisher to publish the claw value to the joint controller
        self.pub1 = rospy.Publisher('/joint_controller/joint6_position_controller/command', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/joint_controller/joint7_position_controller/command', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('/joint_controller/joint8_position_controller/command', Float64, queue_size=10)
        self.pub4 = rospy.Publisher('/joint_controller/joint9_position_controller/command', Float64, queue_size=10)

        # Create a label with the controller name
        self.label = tk.Label(master, text=name)
        self.label.pack()

        # Create an open and close button
        self.open_button = tk.Button(master, text="Open", command=lambda: self.button_callback(open_value), bg="green", fg="white")
        self.open_button.pack(side="left", fill="x", expand=True)

        self.close_button = tk.Button(master, text="Close", command=lambda: self.button_callback(close_value), bg="red", fg="white")
        self.close_button.pack(side="left", fill="x", expand=True)

    def button_callback(self, value):
        # Publish the claw value to the joint controller
        self.pub1.publish(Float64(-1*value))
        self.pub2.publish(Float64(value))
        self.pub3.publish(Float64(-1*value))
        self.pub4.publish(Float64(value))


class MasterController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('arm_controller')

        # Create a Tkinter GUI window
        self.root = tk.Tk()
        self.root.title("Robotic Arm Controller")

        # Set the padding around window
        self.root.geometry("+50+50")

        # Create individual sliders for each joint
        controller1 = SliderController(self.root, "Controller 1", "/joint_controller/joint1_position_controller/command", -180, 180, "blue4")
        controller2 = SliderController(self.root, "Controller 2", "/joint_controller/joint2_position_controller/command", -90, 90, "blue4")
        controller3 = SliderController(self.root, "Controller 3", "/joint_controller/joint3_position_controller/command", -97.4, 97.4, "blue4")
        controller4 = SliderController(self.root, "Controller 4", "/joint_controller/joint4_position_controller/command", -97.4, 97.4, "blue4")
        controller5 = SliderController(self.root, "Controller 5", "/joint_controller/joint5_position_controller/command", -180, 180, "blue4")

        # Create a claw controller
        claw_controller = ClawController(self.root, "Claw Controller", 0.0, -0.785)

    def run(self):
        # Start the Tkinter main loop
        self.root.mainloop()


if __name__ == '__main__':
    try:
        controller = MasterController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
