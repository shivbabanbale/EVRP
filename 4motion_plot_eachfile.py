import networkx as nx
import folium
import time

from folium.plugins import AntPath


# Load datasets
def load_district_points(filename):
    district_points = {}
    with open(filename, 'r') as file:
        next(file)  # Skip header
        for line in file:
            district_id, district_name, latitude, longitude, charging_station_id = line.strip().split(',')
            district_points[int(district_id)] = {
                'DistrictName': district_name,
                'Latitude': float(latitude),
                'Longitude': float(longitude),
                'ChargingStationID': charging_station_id
            }
    return district_points


def load_charging_stations(filename):
    charging_stations = {}
    with open(filename, 'r') as file:
        next(file)  # Skip header
        for line in file:
            charging_station_id, latitude, longitude, charger_type, power, cost_per_kwh, operating_hours = line.strip().split(
                ',')
            charging_stations[charging_station_id] = {
                'Latitude': float(latitude),
                'Longitude': float(longitude),
                'ChargerType': charger_type,
                'Power': int(power),
                'CostPerKWh': float(cost_per_kwh),
                'OperatingHours': int(operating_hours)
            }
    return charging_stations


def load_road_network(filename):
    road_network = nx.Graph()
    with open(filename, 'r') as file:
        next(file)  # Skip header
        for line in file:
            data = line.strip().split(',')
            if len(data) < 3:
                continue  # Skip line if it doesn't have enough data
            road_id, start_point_id, end_point_id, distance, average_speed_limit, road_type = data
            if start_point_id and end_point_id:
                road_network.add_edge(int(start_point_id), int(end_point_id), weight=float(distance))
    return road_network


district_points = load_district_points('11districts.csv')
charging_stations = load_charging_stations('22charging_stations.csv')
road_network = load_road_network('33newroads.csv')

# Define EV specifications
vehicle_battery_capacity = 60  # kWh
vehicle_consumption_rate = 0.2  # kWh/km
charging_time = 1  # hours per kWh
min_charge_threshold = 10  # minimum battery level to avoid running out of charge


# Function to compute shortest route for both distance and time
def find_shortest_routes(start_node, visiting_locations):
    # Create a new graph for the current problem
    G = road_network.copy()

    # Add charging stations as intermediate nodes
    # (code for adding charging stations as intermediate nodes)

    # Compute shortest path for distance
    shortest_distance_route = nx.shortest_path(G, source=start_node, target=visiting_locations[0], weight='weight')
    for i in range(1, len(visiting_locations)):
        shortest_distance_route += nx.shortest_path(G, source=visiting_locations[i - 1], target=visiting_locations[i], weight='weight')
    shortest_distance_route += nx.shortest_path(G, source=visiting_locations[-1], target=start_node, weight='weight')

    # Compute shortest path for time
    shortest_time_route = nx.shortest_path(G, source=start_node, target=visiting_locations[0], weight='time')
    for i in range(1, len(visiting_locations)):
        shortest_time_route += nx.shortest_path(G, source=visiting_locations[i - 1], target=visiting_locations[i], weight='time')
    shortest_time_route += nx.shortest_path(G, source=visiting_locations[-1], target=start_node, weight='time')

    return shortest_distance_route, shortest_time_route


# Helper function to calculate distance between two points
def distance(point1, point2):
    return ((point1['Latitude'] - point2['Latitude']) ** 2 + (point1['Longitude'] - point2['Longitude']) ** 2) ** 0.5



# Helper function to visualize the routes
def visualize_routes_in_motion(distance_route, time_route):
    map = folium.Map(location=[district_points[distance_route[0]]['Latitude'], district_points[distance_route[0]]['Longitude']], zoom_start=8)

    # Plot charging stations
    for node, info in district_points.items():
        if info['ChargingStationID']:
            charging_station_id = info['ChargingStationID']
            charging_station = charging_stations[charging_station_id]
            folium.Marker(location=[charging_station['Latitude'], charging_station['Longitude']],
                          popup=f"Charging Station {charging_station_id}",
                          icon=folium.Icon(color='green')).add_to(map)

    # Plot starting point
    folium.Marker(location=[district_points[distance_route[0]]['Latitude'], district_points[distance_route[0]]['Longitude']],
                  popup="Starting Point",
                  icon=folium.Icon(color='blue')).add_to(map)

    # Animate both routes using AntPath with different colors
    distance_locations = [[district_points[node]['Latitude'], district_points[node]['Longitude']] for node in distance_route]
    time_locations = [[district_points[node]['Latitude'], district_points[node]['Longitude']] for node in time_route]

    distance_ant_path = AntPath(distance_locations, color='blue', weight=2.5, delay=800, dash_array=[10, 20], pulse_color='red')
    time_ant_path = AntPath(time_locations, color='green', weight=2.5, delay=800, dash_array=[10, 20], pulse_color='yellow')

    distance_ant_path.add_to(map)
    time_ant_path.add_to(map)

    # Add vehicle markers for both routes
    vehicle_icon = folium.features.CustomIcon(icon_image='car.png', icon_size=(30, 30))
    distance_vehicle_marker = folium.Marker(location=distance_locations[0], icon=vehicle_icon)
    time_vehicle_marker = folium.Marker(location=time_locations[0], icon=vehicle_icon)
    distance_vehicle_marker.add_to(map)
    time_vehicle_marker.add_to(map)

    # Update vehicle marker positions for a 10-second preview
    for i in range(1, len(distance_locations)):
        distance_vehicle_marker.location = distance_locations[i]
        time_vehicle_marker.location = time_locations[i]
        map.save(f'routes_comparison_preview_{i}.html')
        time.sleep(1)  # Adjust the delay if needed


# Input from user
start_node_name = input("Enter the starting node: ")
start_node = None
for node_id, point in district_points.items():
    if point['DistrictName'] == start_node_name:
        start_node = node_id
        break

if start_node is None:
    print("Error: Starting node not found.")
    exit()

visiting_locations_names = input("Enter visiting locations separated by spaces: ").split()
visiting_locations = []
for location_name in visiting_locations_names:
    for node_id, point in district_points.items():
        if point['DistrictName'] == location_name:
            visiting_locations.append(node_id)
            break

if len(visiting_locations) != len(visiting_locations_names):
    print("Error: One or more visiting locations not found.")
    exit()

shortest_distance_route, shortest_time_route = find_shortest_routes(start_node, visiting_locations)
print("Shortest Distance Route:", shortest_distance_route)
print("Shortest Time Route:", shortest_time_route)

# Visualize the route on map
# visualize_route(shortest_route)
# Visualize the route in motion
visualize_routes_in_motion(shortest_distance_route, shortest_time_route)