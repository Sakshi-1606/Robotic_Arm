import tkinter as tk
from tkinter import *
import rospy
from std_msgs.msg import Float64

class control_1(tk.Tk):

    def __init__(self):
        super().__init__()
        self.sub = rospy.Publisher("/joint1_position_controller", Float64, queue_size=10)
        self.controller_1_data = tk.IntVar()

        # configure the root window
        self.title('Controller 1 Data')
        self.resizable(0, 0)
        self.geometry('250x80')
        self['bg'] = 'black'
        self.style = ttk.Style(self)
        self.style.configure('TLabel', background='black', foreground='red')
        self.label = ttk.Label(self, text=self.get_controller_data(), font=('Digital-7', 20))
        self.label.pack(expand=True)
        self.label.after(1000, self.update)     # schedule an update every 1 second
        w = Scale(master, from_=0, to=42)
        w.pack()
        w = Scale(master, from_=0, to=200, orient=HORIZONTAL)
        w.pack()

    def callback_controller_1(self, data):   
        self.controller_1_data = data.data

    def get_controller_data(self):
        return self.controller_1_data

    def update(self):
        """ update the label every 1 second """
        self.label.configure(text=self.get_controller_data())
        self.label.after(1000, self.update)     # schedule another timer
 
if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    sensor = control_1()
    sensor.mainloop()