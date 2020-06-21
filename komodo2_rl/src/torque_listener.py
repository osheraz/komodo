#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
import numpy as np
from scipy.signal import savgol_filter
import os
from datetime import datetime
from scipy.ndimage.filters import gaussian_filter1d


def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

class TorqueListener:
    def __init__(self):
        self.current_path = os.getcwd()
        self.sub = rospy.Subscriber("/bucket_torque_sensor", WrenchStamped, self.callback)
        self.arr = []

    def callback(self,msg):
        torque = msg.wrench.torque.y
        self.arr.append(abs(torque))

    def force_plot(self):
        self.sub.unregister()
        import matplotlib.pyplot as plt
        date_time = str(datetime.now().strftime('%d_%m_%Y_%H_%M'))
        # np.save(self.current_path + '/data/sim/torque_simulation_' + date_time, self.arr)
        plt.subplot(311)
        w = savgol_filter(self.arr, 5, 2)
        plt.plot(w, 'b-', lw=2)
        plt.subplot(312)
        plt.plot(smooth(self.arr, 10), 'r-', lw=2)
        plt.subplot(313)
        plt.plot(smooth(self.arr, 20), 'g-', lw=2)
        plt.xlabel('time (s)')
        plt.title('Torque over time - Simulation')
        plt.ylabel('Torque (Nm)')
        plt.show()


if __name__ == '__main__':
    TorqueListener()
