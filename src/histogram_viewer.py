#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from road_recognizer.msg import OtsuBinary


def viewer(otsu_msg):
    intensity_threshold_list = np.empty((0, 1), float)
    for r_g in range(otsu_msg.range_resolution):
        intensity_threshold_list = np.append(intensity_threshold_list, np.array([[otsu_msg.intensity[r_g].threshold]]), axis=0)
    
    plt.clf()
    plt.cla()
    plt.subplots_adjust(wspace=0.5, hspace=0.5)

    for r_g in range(otsu_msg.range_resolution):
        plt.subplot(4, 5, r_g+1)
        distribution_intensity_list = np.empty((0, 1), float)
        RANGE = r_g
        for idx_intensity in range(len(otsu_msg.analysis[RANGE].distribution)):
            distribution_intensity_list = np.append(distribution_intensity_list, np.array([[otsu_msg.analysis[RANGE].distribution[idx_intensity].intensity]]), axis=0)
        plot_distribution = plt.plot(np.arange(len(otsu_msg.analysis[RANGE].distribution)), distribution_intensity_list)
        # plot_distribution = plt.bar(np.arange(len(otsu_msg.analysis[RANGE].distribution)), distribution_intensity_list)
        # plot_distribution = plt.bar(otsu_msg.intensity[r_g].threshold, 1, color='r')

        title = "Range=" + str(r_g) + "~" + str(r_g+1) + "[m]"

        # plt.title("Analysis (Distribution of Intensity)")
        plt.title(title)
        plt.xlabel("Intensity")
        plt.ylabel("Amount")
        plt.xlim(0, 255)
        # plt.ylim(0, 200)
        plt.legend()

    plt.pause(0.01)

def callback(msg):
    viewer(msg)


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
