import os
import numpy as np
import matplotlib.pyplot as plt
import geopandas
import pandas as pd

#world = geopandas.read_file(geopandas.datasets.get_path('naturalearth_lowres'))

#cities = geopandas.read_file(geopandas.datasets.get_path('naturalearth_cities'))




#    lon_list.append(78.66)
#    lon_list.append(78)
world = geopandas.read_file(geopandas.datasets.get_path('naturalearth_lowres'))
ax = world[world.continent == 'Asia'].plot(color='white', edgecolor='black')
lat1=17.78
lat2=17.12
lon1=78.66
lon2=78
plt.ion()
for i in range(20):
   
    #df = pd.DataFrame({'Latitude': [17.78,17.12],'Longitude': [78.66,78]})
    lat_list=[17.5943603515625, 17.59447479248047, 17.594579696655273, 17.594528198242188, 17.59454917907715, 17.594446182250977, 17.594491958618164, 17.594446182250977, 17.594594955444336, 17.5944766998291, 17.59458351135254]
    lon_list=[78.12322998046875, 78.12317657470703, 78.12321472167969, 78.12313079833984, 78.12326049804688, 78.12316131591797, 78.12326049804688, 78.1231689453125, 78.12317657470703, 78.12318420410156, 78.1231460571289]
    #df = pd.DataFrame({'Latitude': [lat1,lat2],'Longitude': [lon1,lon2]})
    df = pd.DataFrame({'Latitude': lat_list,'Longitude': lon_list})
    gdf = geopandas.GeoDataFrame(df, geometry=geopandas.points_from_xy(df.Longitude, df.Latitude))
    
    gdf.plot(ax=ax, color='red', edgecolor='black')
    lat1=lat1+1
    lat2=lat2+1
    lon1=lon1+1
    lon2=lon2+1
    plt.draw()
    plt.pause(1)
    gdf.plot(ax=ax, color='white', edgecolor='white')
    #plt.clf()
    