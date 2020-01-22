#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sensor_msgs
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import pcl
import pandas as pd
import geopandas
import matplotlib.pyplot as plt
import time
import numpy as np


def callback(ros_cloud):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    y = np.random.random([10,1])

    plt.plot(y)
    plt.draw()
    plt.pause(0.0001)
    plt.clf()
    
def geo_plotting():
    rospy.init_node('geo_plotting', anonymous=True)
    rospy.Subscriber('/georeferenced_topic',PointCloud2, callback)
    rospy.spin()


if __name__ == '__main__':
    geo_plotting()
