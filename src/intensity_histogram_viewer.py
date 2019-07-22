#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from road_recognizer.msg import OtsuBinary



def viewer(otsu_msg):
    intensity_threshold_list = np.empty((0, 1), float)
    intensity_min_list = np.empty((0, 1), float)
    intensity_max_list = np.empty((0, 1), float)
    separation_list = np.empty((0, 1), float)
    skewness_list = np.empty((0, 1), float)
    #distribution_list = []

    for r_g in range(otsu_msg.range_resolution):
        intensity_threshold_list = np.append(intensity_threshold_list, np.array([[otsu_msg.intensity[r_g].threshold]]), axis=0)
        intensity_min_list = np.append(intensity_min_list, np.array([[otsu_msg.intensity[r_g].min]]), axis=0)
        intensity_max_list = np.append(intensity_max_list, np.array([[otsu_msg.intensity[r_g].max]]), axis=0)
        separation_list = np.append(separation_list, np.array([[otsu_msg.analysis[r_g].separation]]), axis=0)
        skewness_list = np.append(skewness_list, np.array([[otsu_msg.analysis[r_g].skewness]]), axis=0)

        

    plt.clf()
    plt.cla()
    plt.subplots_adjust(wspace=0.5, hspace=0.5)

    plt.subplot(2, 2, 1)
    plot_intensity = plt.plot(np.arange(len(otsu_msg.intensity)), intensity_threshold_list, label="threshold", color="g")
    plot_intensity = plt.plot(np.arange(len(otsu_msg.intensity)), intensity_min_list, label="min", color="b")
    plot_intensity = plt.plot(np.arange(len(otsu_msg.intensity)), intensity_max_list, label="max", color="r")
    plt.title("Intensity")
    plt.xlabel("Range[m]")
    plt.ylabel("Intensity")
    plt.legend()
        
    plt.subplot(2, 2, 2)
    plot_separation = plt.plot(np.arange(len(otsu_msg.analysis)), separation_list, label="separation", color="b")
    plt.title("Analisy (Separation))")
    plt.xlabel("Range[m]")
    plt.ylabel("Separation")
    plt.ylim(0, 1000)

    plt.subplot(2, 2, 4)
    plot_skewness = plt.plot(np.arange(len(otsu_msg.analysis)), skewness_list, label="skewness", color="b")
    plt.title("Analysis (Skewness)")
    plt.xlabel("Range[m]")
    plt.ylabel("Skewness")

    plt.subplot(2, 2, 3)
    # for r_g in range(otsu_msg.range_resolution):
    #     len_distribution = len(otsu_msg.analysis[r_g].distribution)
    #     distribution_intensity_list = np.empty((0, 1), float)
    #     for idx_intensity in range(len_distribution):
    #         distribution_intensity_list = np.append(distribution_intensity_list, np.array([[otsu_msg.analysis[r_g].distribution[idx_intensity].intensity]]), axis=0)
    #     plot_distribution = plt.plot(np.arange(len_distribution), distribution_intensity_list)

    distribution_intensity_list = np.empty((0, 1), float)
    RANGE = 5
    for idx_intensity in range(len(otsu_msg.analysis[RANGE].distribution)):
        distribution_intensity_list = np.append(distribution_intensity_list, np.array([[otsu_msg.analysis[RANGE].distribution[idx_intensity].intensity]]), axis=0)
    plot_distribution = plt.plot(np.arange(len(otsu_msg.analysis[RANGE].distribution)), distribution_intensity_list)


    # plt.title("Analysis (Distribution of Intensity)")
    plt.title("Distribution of intensity (@Range=5[m])")
    plt.xlabel("Intensity")
    plt.ylabel("Amount")
    plt.xlim(-10, 255)

    plt.pause(0.01)
    

def callback(msg):
    viewer(msg)


def listen():
    print("listen")
    rospy.Subscriber("/intensity_partition/otsu_binary_info", OtsuBinary, callback)
    print("subscribed")
    rospy.spin()


def main():
    rospy.init_node('intensity_histogram_viewer', anonymous=True)
    listen()


if __name__ == '__main__':
    main()
