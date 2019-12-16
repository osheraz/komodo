#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
import numpy as np
from scipy.signal import savgol_filter

from scipy.ndimage.filters import gaussian_filter1d


def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

class TorqueListener:
    def __init__(self):
        # start the node
        #rospy.init_node("listener")
        # subscribe to the '/sr_tactile/touch/ff' topic
        self.sub = rospy.Subscriber("/bucket_torque_sensor", WrenchStamped, self.callback)
        self.arr = []
        # keep python from exiting until this node is stopped
        #rospy.spin()

    def callback(self,msg):
        # log the message data to the terminal
        torque = msg.wrench.torque.y
        #rospy.loginfo(torque)
        self.arr.append(torque)

    def force_plot(self):
        self.sub.unregister()
        import matplotlib.pyplot as plt
        np.save('torque_simulation', self.arr)
        plt.subplot(311)
        w = savgol_filter(self.arr, 501, 2)
        plt.plot(w, 'b-', lw=2)
        plt.subplot(312)
        plt.plot(smooth(self.arr, 3), 'r-', lw=2)
        plt.subplot(313)
        plt.plot(smooth(self.arr, 10), 'g-', lw=2)
        plt.xlabel('time (s)')
        plt.title('Torque over time - Simulation')
        plt.ylabel('Torque (Nm)')
        plt.show()


if __name__ == '__main__':
    force_listener()
