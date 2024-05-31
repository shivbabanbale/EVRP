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


district_points = load_district_points('districts.csv')
charging_stations = load_charging_stations('charging_stations.csv')
road_network = load_road_network('roads.csv')

# Define EV specifications
vehicle_battery_capacity = 60  # kWh
vehicle_consumption_rate = 0.2  # kWh/km
charging_time = 1  # hours per kWh
min_charge_threshold = 10  # minimum battery level to avoid running out of charge


# Function to compute shortest route for both distance and time
import heapq

def find_shortest_routes(start_node, visiting_locations):
    # Create a new graph for the current problem
    G = road_network.copy()

    # Priority queue to store paths to be explored
    priority_queue = []

    # Initialize the priority queue with the start node and an empty path
    heapq.heappush(priority_queue, (0, [start_node]))

    # Initialize the best shortest route and its weight
    best_shortest_route = None
    best_shortest_weight = float('inf')

    # Explore all possible paths using DFS
    while priority_queue:
        current_weight, current_path = heapq.heappop(priority_queue)

        # If the current path reaches the goal node, update the best shortest route
        if current_path[-1] == visiting_locations[-1] and current_weight < best_shortest_weight:
            best_shortest_route = current_path
            best_shortest_weight = current_weight

        # Explore neighbors of the last node in the current path
        for neighbor, edge_data in G[current_path[-1]].items():
            weight = edge_data['weight']
            if neighbor not in current_path:  # Avoid cycles
                # Compute the weight of the new path
                new_weight = current_weight + weight
                # Push the new path to the priority queue
                heapq.heappush(priority_queue, (new_weight, current_path + [neighbor]))

    # Construct shortest distance and time routes
    shortest_distance_route = [visiting_locations[0]]
    shortest_time_route = [visiting_locations[0]]
    for i in range(1, len(visiting_locations)):
        shortest_distance_route.extend(nx.shortest_path(G, source=visiting_locations[i - 1], target=visiting_locations[i], weight='weight'))
        shortest_time_route.extend(nx.shortest_path(G, source=visiting_locations[i - 1], target=visiting_locations[i], weight='time'))
    shortest_distance_route.append(start_node)
    shortest_time_route.append(start_node)

    return shortest_distance_route, shortest_time_route, best_shortest_route

# Helper function to calculate distance between two points
def distance(point1, point2):
    return ((point1['Latitude'] - point2['Latitude']) ** 2 + (point1['Longitude'] - point2['Longitude']) ** 2) ** 0.5

# Helper function to visualize the routes
import folium
from folium.plugins import AntPath

def visualize_routes_in_motion(shortest_distance_route, shortest_time_route, best_shortest_route):
    map = folium.Map(location=[district_points[shortest_distance_route[0]]['Latitude'], district_points[shortest_distance_route[0]]['Longitude']], zoom_start=8)

    # Plot charging stations
    for node, info in district_points.items():
        if info['ChargingStationID']:
            charging_station_id = info['ChargingStationID']
            charging_station = charging_stations[charging_station_id]
            folium.Marker(location=[charging_station['Latitude'], charging_station['Longitude']],
                          popup=f"Charging Station {charging_station_id}",
                          icon=folium.Icon(color='green')).add_to(map)

    # Plot starting point
    folium.Marker(location=[district_points[shortest_distance_route[0]]['Latitude'], district_points[shortest_distance_route[0]]['Longitude']],
                  popup="Starting Point",
                  icon=folium.Icon(color='blue')).add_to(map)

    # Plot shortest distance route
    shortest_distance_locations = [[district_points[node]['Latitude'], district_points[node]['Longitude']] for node in shortest_distance_route]
    shortest_distance_ant_path = AntPath(shortest_distance_locations, color='blue', weight=2.5, delay=800, dash_array=[10, 20], pulse_color='red')
    shortest_distance_ant_path.add_to(map)

    # Plot shortest time route
    shortest_time_locations = [[district_points[node]['Latitude'], district_points[node]['Longitude']] for node in shortest_time_route]
    shortest_time_ant_path = AntPath(shortest_time_locations, color='green', weight=2.5, delay=800, dash_array=[10, 20], pulse_color='yellow')
    shortest_time_ant_path.add_to(map)

    # Plot best shortest route
    best_shortest_locations = [[district_points[node]['Latitude'], district_points[node]['Longitude']] for node in best_shortest_route]
    best_shortest_ant_path = AntPath(best_shortest_locations, color='red', weight=2.5, delay=800, dash_array=[10, 20], pulse_color='orange')
    best_shortest_ant_path.add_to(map)

    map.save('routes_comparison.html')


# Visualize the routes in motion
# Input starting point from user
start_node_name = input("Enter the starting node: ")
start_node = None
for node_id, point in district_points.items():
    if point['DistrictName'] == start_node_name:
        start_node = node_id
        break

if start_node is None:
    print("Error: Starting node not found.")
    exit()

# Input visiting locations from user
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

# Call find_shortest_routes with the correct unpacking
# shortest_distance_route, shortest_time_route, best_shortest_route = find_shortest_routes(start_node, visiting_locations)
shortest_distance_route, shortest_time_route, best_shortest_route = find_shortest_routes(start_node, visiting_locations)

print("Shortest Distance Route:", shortest_distance_route)
print("Shortest Time Route:", shortest_time_route)
print("Best Shortest Route:", best_shortest_route)

# Visualize the route on map
# visualize_route(shortest_route)
# Visualize the route in motion
visualize_routes_in_motion(shortest_distance_route, shortest_time_route , best_shortest_route)