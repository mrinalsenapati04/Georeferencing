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

world = geopandas.read_file(geopandas.datasets.get_path('naturalearth_lowres'))
ax = world[world.continent == 'Asia'].plot(color='white', edgecolor='black')

def callback(ros_cloud):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    points_list = []
    print('points are coming')
    i=0
    lat_list=[]
    lon_list=[]
    height_list=[]
    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2]])
        print('The object ',i,' location is ',data[0],' ',data[1],' ',data[2])
        i=i+1
        lat_list.append(data[0])
        lon_list.append(data[1])
        height_list.append(data[2])

    #world = geopandas.read_file(geopandas.datasets.get_path('naturalearth_lowres'))
    df = pd.DataFrame({'Latitude': lat_list,'Longitude': lon_list})
    gdf = geopandas.GeoDataFrame(df, geometry=geopandas.points_from_xy(df.Longitude, df.Latitude))
    #ax = world[world.continent == 'Asia'].plot(color='white', edgecolor='black')
    gdf.plot(ax=ax, color='red')
    #plt.show()
    #plt.show(block=False)
    #time.sleep(0.1)
    #plt.close('all')
    plt.draw()
    plt.pause(0.1)
    plt.clf()




def geo_plotting():
    rospy.init_node('geo_plotting', anonymous=True)
    rospy.Subscriber('/georeferenced_topic',PointCloud2, callback)
    rospy.spin()


if __name__ == '__main__':
    geo_plotting()
