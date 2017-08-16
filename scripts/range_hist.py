#!/usr/bin/env python

import sys
import numpy as np
import scipy.stats as st
import rospy
import rosbag
from pprint import pprint
import matplotlib.pyplot as plt

def plot_temporal(data, key):
    plt.plot(np.arange(len(data)), data, label='Ground Truth = ' + str(key) + ' m')

def parse_bag_header(s):
    n = s.split('/')[1].split('m')[0].split('_')

    d = float(n[0]) # dollars
    c = 0           # cents
    if len(n) == 2:
        c = float(n[1])
    
    # ground truth from header
    gt = d + (c / 100.)
    
    return gt;

def read_uwb_data(bag):

    topic_list = ['/pulson/node_103/tdma_node/ranges']
    ranges = []

    for topic, msg, t in bag.read_messages(topics=topic_list):
        if topic == topic_list[0]:
            d = msg.precision_range
            ranges.append(d / 1000.)

    return np.array(ranges)

def open_bags(argv):

    data = {}
    gtruth = []

    for arg in argv:

        # deal with header
        gt = parse_bag_header(arg)
        gtruth.append(gt)

        # open and read
        bag = rosbag.Bag(arg, 'r')
        ranges = read_uwb_data(bag)
        bag.close

        # save
        key = int(gt)
        data[key] = ranges[np.nonzero(ranges)]
    
    return gtruth, data;


if __name__ == '__main__':

    if len(sys.argv) < 2:
        print 'Usage: '
        sys.exit()

    gtruth, data = open_bags(sys.argv[1:])

    # plot histogram
    hdata = np.array([])
    for range in gtruth:
        key = int(range)
        hdata = np.concatenate((hdata, data[key]))

    plt.figure()
    mn = 0
    mx = 20
    step = .01 # meters
    res = (mx - mn) / step
    for range in gtruth:
        key = int(range)
        counts, bins = np.histogram(data[key], res, (mn, mx))
        bins = bins[:-1] + (bins[1] - bins[0]) / 2
        p = counts / float(counts.sum())
        plt.bar(bins, 
                p, 
                (mx - mn) / res, 
                color=np.random.rand(3, 1),
                label='Ground Truth = ' + str(key) + ' m')

    plt.legend()
    plt.xlim([0, 20])
    plt.title('Histogram of Distance Measurements')
    plt.xlabel('Distance (m), Bin Width = ' + str(step) + ' m')
    plt.ylabel('Probability')

    # some temporal data
    plt.figure()
    for range in gtruth:
        key = int(range)
        plot_temporal(data[key], range)

    plt.legend()
    plt.xlim([0, 200])
    plt.ylim([0, 20])
    plt.title('Distance Measurements over Time')
    plt.xlabel('Time (steps)')
    plt.ylabel('Distance')
    plt.show()

