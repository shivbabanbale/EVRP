import networkx as nx
import folium
import time

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



district_points = load_district_points('districts.csv')
charging_stations = load_charging_stations('charging_stations.csv')
road_network = load_road_network('roads.csv')

# Define EV specifications
vehicle_battery_capacity = 60  # kWh
vehicle_consumption_rate = 0.2  # kWh/km
charging_time = 1  # hours per kWh
min_charge_threshold = 10  # minimum battery level to avoid running out of charge


# Function to compute shortest route
def find_shortest_route(start_node, visiting_locations):
    # Create a new graph for the current problem
    G = road_network.copy()

    # Add charging stations as intermediate nodes
    for node, info in district_points.items():
        if info['ChargingStationID']:
            charging_station_id = info['ChargingStationID']
            charging_station = charging_stations[charging_station_id]
            G.add_node(charging_station_id, Latitude=charging_station['Latitude'],
                       Longitude=charging_station['Longitude'])

            # Add edges between districts and charging stations
            G.add_edge(node, charging_station_id, weight=distance(district_points[node], charging_station))
            G.add_edge(charging_station_id, node, weight=distance(district_points[node], charging_station))

    # Compute shortest path
    shortest_path = []
    for i in range(len(visiting_locations)):
        if i == 0:
            shortest_path.extend(nx.shortest_path(G, source=start_node, target=visiting_locations[i]))
        else:
            shortest_path.extend(nx.shortest_path(G, source=visiting_locations[i - 1], target=visiting_locations[i]))

    # Add return to start node
    shortest_path.extend(nx.shortest_path(G, source=visiting_locations[-1], target=start_node))

    return shortest_path


# Helper function to calculate distance between two points
def distance(point1, point2):
    return ((point1['Latitude'] - point2['Latitude']) ** 2 + (point1['Longitude'] - point2['Longitude']) ** 2) ** 0.5

import networkx as nx
import folium


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



district_points = load_district_points('districts.csv')
charging_stations = load_charging_stations('charging_stations.csv')
road_network = load_road_network('roads.csv')

# Define EV specifications
vehicle_battery_capacity = 60  # kWh
vehicle_consumption_rate = 0.2  # kWh/km
charging_time = 1  # hours per kWh
min_charge_threshold = 10  # minimum battery level to avoid running out of charge


# Function to compute shortest route
def find_shortest_route(start_node, visiting_locations):
    # Create a new graph for the current problem
    G = road_network.copy()

    # Add charging stations as intermediate nodes
    for node, info in district_points.items():
        if info['ChargingStationID']:
            charging_station_id = info['ChargingStationID']
            charging_station = charging_stations[charging_station_id]
            G.add_node(charging_station_id, Latitude=charging_station['Latitude'],
                       Longitude=charging_station['Longitude'])

            # Add edges between districts and charging stations
            G.add_edge(node, charging_station_id, weight=distance(district_points[node], charging_station))
            G.add_edge(charging_station_id, node, weight=distance(district_points[node], charging_station))

    # Compute shortest path
    shortest_path = []
    for i in range(len(visiting_locations)):
        if i == 0:
            shortest_path.extend(nx.shortest_path(G, source=start_node, target=visiting_locations[i]))
        else:
            shortest_path.extend(nx.shortest_path(G, source=visiting_locations[i - 1], target=visiting_locations[i]))

    # Add return to start node
    shortest_path.extend(nx.shortest_path(G, source=visiting_locations[-1], target=start_node))

    return shortest_path


# Helper function to calculate distance between two points
def distance(point1, point2):
    return ((point1['Latitude'] - point2['Latitude']) ** 2 + (point1['Longitude'] - point2['Longitude']) ** 2) ** 0.5

# Function to visualize the route on map with moving vehicle
def visualize_route(route):
    # Create a map centered around the starting point
    start_node = route[0]
    map = folium.Map(location=[district_points[start_node]['Latitude'], district_points[start_node]['Longitude']],
                     zoom_start=6)

    # Plot route with simplified polyline
    if len(route) > 100:
        simplified_route = [route[i] for i in range(0, len(route), len(route) // 100)]  # Downsample route
    else:
        simplified_route = route
    points = [(district_points[node]['Latitude'], district_points[node]['Longitude']) for node in simplified_route]
    folium.PolyLine(points, color="blue", weight=2.5, opacity=1).add_to(map)

    # Plot charging stations (optional, you can remove this part if it's not necessary)
    # for node, info in district_points.items():
    #     if info['ChargingStationID']:
    #         charging_station_id = info['ChargingStationID']
    #         charging_station = charging_stations[charging_station_id]
    #         folium.Marker(location=[charging_station['Latitude'], charging_station['Longitude']],
    #                       popup=f"Charging Station {charging_station_id}",
    #                       icon=folium.Icon(color='green')).add_to(map)

    # Save the map as an HTML file
    map.save('route_map.html')
    print("Map saved as route_map.html")

# Use the function with the provided route


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

# Compute shortest route
shortest_route = find_shortest_route(start_node, visiting_locations)
print("Shortest Route:", shortest_route)

# Visualize the route on map
visualize_route(shortest_route)