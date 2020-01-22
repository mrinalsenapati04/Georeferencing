import pandas as pd
import geopandas
import matplotlib.pyplot as plt
#df = pd.DataFrame(
#    {'Latitude': [1.58, 18.78, 19.45, 20.60, 21.48],
#     'Longitude': [70.66, -77.91, 78.66, -79.08, 80.86]})
df = pd.DataFrame(
    {'Latitude': [17.78,17.12],
     'Longitude': [78.66,78]})
gdf = geopandas.GeoDataFrame(
    df, geometry=geopandas.points_from_xy(df.Longitude, df.Latitude))
print(gdf.head())
world = geopandas.read_file(geopandas.datasets.get_path('naturalearth_lowres'))
#print(world.continent)
# We restrict to South America.
ax = world[world.continent == 'Asia'].plot(
    color='white', edgecolor='black')

# We can now plot our ``GeoDataFrame``.
gdf.plot(ax=ax, color='red')

plt.show()
plt.pause(1)
plt.close()