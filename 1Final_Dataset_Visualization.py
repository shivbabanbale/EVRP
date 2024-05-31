import pandas as pd
import matplotlib.pyplot as plt

# Load datasets
districts_df = pd.read_csv("11districts.csv")
charging_stations_df = pd.read_csv("22charging_stations.csv")
road_network_df = pd.read_csv("33newroads.csv")

# Create a dictionary mapping District IDs to District Names
district_names = dict(zip(districts_df['DistrictID'], districts_df['DistrictName']))

# Plot district points with names
for idx, district in districts_df.iterrows():
    plt.scatter(district['Longitude'], district['Latitude'], color='blue')
    plt.text(district['Longitude'], district['Latitude'], district_names[district['DistrictID']], fontsize=8, ha='right')

# Plot charging stations
plt.scatter(charging_stations_df['Longitude'], charging_stations_df['Latitude'], color='red', label='Charging Stations', marker='x')

# Plot road network
for idx, road in road_network_df.iterrows():
    start_point_id = road['StartPointID']
    end_point_id = road['EndPointID']
    start_point = districts_df[districts_df['DistrictID'] == start_point_id]
    end_point = districts_df[districts_df['DistrictID'] == end_point_id]
    if len(start_point) == 0 or len(end_point) == 0:
        print(f"Error: Road segment {idx} has invalid start or end point IDs.")
        continue
    start_point = start_point.iloc[0]
    end_point = end_point.iloc[0]
    plt.plot([start_point['Longitude'], end_point['Longitude']], [start_point['Latitude'], end_point['Latitude']], color='green')

plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('District Points, Charging Stations, and Road Network')
plt.legend()
plt.grid(True)
plt.show()
