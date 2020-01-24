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
plt.ion()
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
        #print('The object ',i,' location is ',data[0],' ',data[1],' ',data[2])
        i=i+1
        lat_list.append(data[0])
        lon_list.append(data[1])
        height_list.append(data[2])

    #world = geopandas.read_file(geopandas.datasets.get_path('naturalearth_lowres'))
    print(lat_list)
    print(lon_list)
    df = pd.DataFrame({'Latitude': lat_list,'Longitude': lon_list})
    gdf = geopandas.GeoDataFrame(df, geometry=geopandas.points_from_xy(df.Longitude, df.Latitude))
    gdf.plot(ax=ax, color='red',edgecolor='black')
    plt.draw()
    plt.pause(1)
    gdf.plot(ax=ax, color='white', edgecolor='white')
    #plt.clf()




def geo_plotting():
    rospy.init_node('geo_plotting', anonymous=True)
    rospy.Subscriber('/georeferenced_topic',PointCloud2, callback)
    rospy.spin()


if __name__ == '__main__':
    geo_plotting()
    
    
    
#    for i in range(20):
#   
#    #df = pd.DataFrame({'Latitude': [17.78,17.12],'Longitude': [78.66,78]})
#    lat_list=[17.5943603515625, 17.59447479248047, 17.594579696655273, 17.594528198242188, 17.59454917907715, 17.594446182250977, 17.594491958618164, 17.594446182250977, 17.594594955444336, 17.5944766998291, 17.59458351135254]
#    lon_list=[78.12322998046875, 78.12317657470703, 78.12321472167969, 78.12313079833984, 78.12326049804688, 78.12316131591797, 78.12326049804688, 78.1231689453125, 78.12317657470703, 78.12318420410156, 78.1231460571289]
#    #df = pd.DataFrame({'Latitude': [lat1,lat2],'Longitude': [lon1,lon2]})
#    df = pd.DataFrame({'Latitude': lat_list,'Longitude': lon_list})
#    gdf = geopandas.GeoDataFrame(df, geometry=geopandas.points_from_xy(df.Longitude, df.Latitude))
#    
#    gdf.plot(ax=ax, color='red', edgecolor='black')
#    lat1=lat1+1
#    lat2=lat2+1
#    lon1=lon1+1
#    lon2=lon2+1
#    plt.draw()
#    plt.pause(1)
#    gdf.plot(ax=ax, color='white', edgecolor='white')
