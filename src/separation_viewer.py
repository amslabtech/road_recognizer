#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from road_recognizer.msg import OtsuBinary
# import time


def viewer(otsu_msg):
    separation_list = np.empty((0, 1), float)
    separation_threshold = np.empty((0, 1), float)

    for r_g in range(otsu_msg.range_resolution):
        separation_threshold = np.append(separation_threshold, np.array([[otsu_msg.otsubinary_separation_threshold]]))
    
        
    for r_g in range(otsu_msg.range_resolution):
        separation_list = np.append(separation_list, np.array([[otsu_msg.analysis[r_g].separation]]), axis=0)
    
    plt.clf()
    plt.cla()
    
    plot_separation = plt.plot(np.arange(len(otsu_msg.analysis)), separation_list, label="separation", color="b")
    plot_separation = plt.plot(np.arange(len(otsu_msg.analysis)), separation_threshold, label="threshold of separation", color="r")
    plt.title("Analisy (Separation))")
    plt.xlabel("Range[m]")
    plt.ylabel("Separation")
    plt.ylim(0, 1.1)

    plt.pause(0.01)

def callback(msg):
    # print("callback")
    # start = time.time()
    viewer(msg)
    # elapsed_time = time.time() - start
    # print ("1 roop elapsed_time:{0}".format(elapsed_time) + "[sec]")


def listen():
    print("listen")
    rospy.Subscriber("/intensity_partition/otsu_binary_info", OtsuBinary, callback)
    print("subscribing")
    rospy.spin()


def main():
    rospy.init_node('intensity_histogram_viewer', anonymous=True)
    listen()


if __name__ == '__main__':
    main()
