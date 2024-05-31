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
    lat1, lon1 = point1
    lat2, lon2 = point2
    return ((lat1 - lat2) ** 2 + (lon1 - lon2) ** 2) ** 0.5

# Helper function to visualize the routes
# Helper function to calculate time required for charging at a charging station
def calculate_charging_time(distance_to_travel, remaining_battery):
    # Calculate energy required to cover the remaining distance
    energy_required = (distance_to_travel / vehicle_consumption_rate) - remaining_battery
    # Calculate time required for charging
    charging_time_needed = energy_required * charging_time
    return charging_time_needed

# Helper function to find the nearest charging station
# Helper function to find the nearest charging station
# Helper function to find the nearest charging station
def find_nearest_charging_station(location):
    min_distance = float('inf')
    nearest_charging_station = None
    for charging_station_id, charging_station in charging_stations.items():
        distance_to_charging_station = distance(location, (charging_station['Latitude'], charging_station['Longitude']))
        if distance_to_charging_station < min_distance:
            min_distance = distance_to_charging_station
            nearest_charging_station = charging_station_id
    return nearest_charging_station



# Helper function to visualize the routes with charging stations
def visualize_routes_in_motion(distance_route, time_route, selected_charging_station=None):
    map = folium.Map(location=[district_points[distance_route[0]]['Latitude'], district_points[distance_route[0]]['Longitude']], zoom_start=8)

    # Plot charging stations
    for node, info in district_points.items():
        if info['ChargingStationID']:
            charging_station_id = info['ChargingStationID']
            charging_station = charging_stations[charging_station_id]
            if selected_charging_station and charging_station_id == selected_charging_station:
                # Mark selected charging station with a different color
                folium.Marker(location=[charging_station['Latitude'], charging_station['Longitude']],
                              popup=f"Charging Station {charging_station_id}",
                              icon=folium.Icon(color='red')).add_to(map)
            else:
                folium.Marker(location=[charging_station['Latitude'], charging_station['Longitude']],
                              popup=f"Charging Station {charging_station_id}",
                              icon=folium.Icon(color='green')).add_to(map)

    # Plot starting point
    folium.Marker(location=[district_points[distance_route[0]]['Latitude'], district_points[distance_route[0]]['Longitude']],
                  popup="Starting Point",
                  icon=folium.Icon(color='blue')).add_to(map)

    # Ensure both routes have the same number of locations
    min_len = min(len(distance_route), len(time_route))
    distance_locations = [[district_points[node]['Latitude'], district_points[node]['Longitude']] for node in distance_route[:min_len]]
    time_locations = [[district_points[node]['Latitude'], district_points[node]['Longitude']] for node in time_route[:min_len]]

    # Animate the going route using AntPath with a blue blinking effect and index nodes
    distance_ant_path = AntPath(distance_locations, color='blue', weight=2.5, delay=800, dash_array=[10, 20], pulse_color='red')
    distance_ant_path.add_to(map)

    # Animate the returning route using AntPath with a green blinking effect and index nodes
    time_ant_path = AntPath(time_locations, color='green', weight=2.5, delay=800, dash_array=[10, 20], pulse_color='yellow')
    time_ant_path.add_to(map)

    # Add vehicle markers for both going and returning routes
    vehicle_icon = folium.features.CustomIcon(icon_image='car.png', icon_size=(30, 30))
    distance_vehicle_marker = folium.Marker(location=distance_locations[0], icon=vehicle_icon, popup="Starting Point")
    time_vehicle_marker = folium.Marker(location=time_locations[0], icon=vehicle_icon, popup="Starting Point")
    distance_vehicle_marker.add_to(map)
    time_vehicle_marker.add_to(map)

    # Add indexing above the nodes
    for i, location in enumerate(distance_locations):
        folium.Marker(location=location, icon=folium.DivIcon(html=f"<div style='color: blue;'>{i + 1}</div>")).add_to(map)
    for i, location in enumerate(time_locations):
        folium.Marker(location=location, icon=folium.DivIcon(html=f"<div style='color: green;'>{i + 1}</div>")).add_to(map)

    # Update vehicle marker positions and consider charging if necessary
    remaining_battery = vehicle_battery_capacity
    for i in range(1, min_len):
        distance_vehicle_marker.location = distance_locations[i]
        time_vehicle_marker.location = time_locations[i]

        # Check if charging is required
        distance_to_travel = distance(distance_locations[i-1], distance_locations[i])  # Fixed the distance function call
        if remaining_battery < distance_to_travel:
            # Charging required
            nearest_charging_station_id = find_nearest_charging_station(distance_locations[i])
            charging_station = charging_stations[nearest_charging_station_id]

            # Calculate required charge to cover the remaining distance
            required_charge = vehicle_battery_capacity - remaining_battery
            # Calculate charging time needed to charge required amount of battery
            charging_time_needed = calculate_charging_time(distance_to_travel, remaining_battery)

            # Print charging details
            print(f"Charging required at Station {nearest_charging_station_id} to charge {required_charge:.2f} kWh for {charging_time_needed:.2f} hours")
            # Visualize charging station with different color
            folium.Marker(location=[charging_station['Latitude'], charging_station['Longitude']],
                          popup=f"Charging Station {nearest_charging_station_id}\nCharge: {required_charge:.2f} kWh\nCharging Time: {charging_time_needed:.2f} hours",
                          icon=folium.Icon(color='orange')).add_to(map)

            # Update remaining battery after charging
            remaining_battery = vehicle_battery_capacity
            # Update remaining battery considering time spent for charging
            remaining_battery -= (charging_time_needed / charging_time) * vehicle_consumption_rate

        # Update remaining battery
        remaining_battery -= distance_to_travel / vehicle_consumption_rate

    map.save('routes_comparison.html')




# Rest of the code remains the same


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
# Calculate total distance of the route
total_distance = 0
for i in range(len(shortest_distance_route) - 1):
    edge_data = road_network.get_edge_data(shortest_distance_route[i], shortest_distance_route[i + 1])
    if edge_data is not None and 'weight' in edge_data:
        total_distance += edge_data['weight']
# Calculate total distance of the route within battery range
total_distance_within_battery_range = 0
remaining_battery = vehicle_battery_capacity
for i in range(len(shortest_distance_route) - 1):
    edge_data = road_network.get_edge_data(shortest_distance_route[i], shortest_distance_route[i + 1])
    if edge_data is not None and 'weight' in edge_data:
        edge_distance = edge_data['weight']  # Rename variable to avoid conflict
        if remaining_battery >= edge_distance * vehicle_consumption_rate:
            total_distance_within_battery_range += edge_distance
            remaining_battery -= edge_distance * vehicle_consumption_rate
        else:
            break

# Print total distance within battery range

print("Shortest Distance Route:", shortest_distance_route)
print("Shortest Time Route:", shortest_time_route)
print(f"Total distance to cover: {total_distance:.2f} km")
print(f"Vehicle battery range: {vehicle_battery_capacity} kWh")
print(f"Total distance traveled within battery range: {total_distance_within_battery_range:.2f} km")
# Visualize the route on map
# visualize_route(shortest_route)
# Visualize the route in motion
visualize_routes_in_motion(shortest_distance_route, shortest_time_route)