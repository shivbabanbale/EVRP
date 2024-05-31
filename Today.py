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


def distance(point1, point2):
    lat1, lon1 = point1
    lat2, lon2 = point2
    return ((lat1 - lat2) ** 2 + (lon1 - lon2) ** 2) ** 0.5


def calculate_charging_time(distance_to_travel, remaining_battery):
    energy_required = (distance_to_travel / vehicle_consumption_rate) - remaining_battery
    charging_time_needed = energy_required * charging_time
    return charging_time_needed


def find_nearest_charging_station(location):
    min_distance = float('inf')
    nearest_charging_station = None
    for charging_station_id, charging_station in charging_stations.items():
        distance_to_charging_station = distance(location, (charging_station['Latitude'], charging_station['Longitude']))
        if distance_to_charging_station < min_distance:
            min_distance = distance_to_charging_station
            nearest_charging_station = charging_station_id
    return nearest_charging_station


def visualize_routes_in_motion(distance_route, time_route, charging_station_id_to_charge=None):
    map = folium.Map(
        location=[district_points[distance_route[0]]['Latitude'], district_points[distance_route[0]]['Longitude']],
        zoom_start=8)

    # Plot charging stations
    for node, info in district_points.items():
        if info['ChargingStationID']:
            charging_station_id = info['ChargingStationID']
            charging_station = charging_stations[charging_station_id]
            if charging_station_id == charging_station_id_to_charge:
                # Mark the selected charging station for charging with a different color
                folium.Marker(location=[charging_station['Latitude'], charging_station['Longitude']],
                              popup=f"Charging Station {charging_station_id}",
                              icon=folium.Icon(color='orange')).add_to(map)
            else:
                folium.Marker(location=[charging_station['Latitude'], charging_station['Longitude']],
                              popup=f"Charging Station {charging_station_id}",
                              icon=folium.Icon(color='green')).add_to(map)

    # Rest of the function remains the same...

    # Update vehicle marker positions and consider charging if necessary
    remaining_battery = vehicle_battery_capacity
    for i in range(1, min_len):
        distance_vehicle_marker.location = distance_locations[i]
        time_vehicle_marker.location = time_locations[i]

        # Check if charging is required
        distance_to_travel = distance(distance_locations[i - 1], distance_locations[i])
        if remaining_battery < distance_to_travel:
            # Charging required
            nearest_charging_station_id = find_nearest_charging_station(distance_locations[i])
            charging_station = charging_stations[nearest_charging_station_id]
            charging_time_needed = calculate_charging_time(distance_to_travel, remaining_battery)
            print(f"Charging required at Station {nearest_charging_station_id} for {charging_time_needed:.2f} hours")

            # Update charging station ID to charge
            charging_station_id_to_charge = nearest_charging_station_id

            # Visualize charging station with different color
            folium.Marker(location=[charging_station['Latitude'], charging_station['Longitude']],
                          popup=f"Charging Station {nearest_charging_station_id}\nCharging Time: {charging_time_needed:.2f} hours",
                          icon=folium.Icon(color='orange')).add_to(map)

            # Update remaining battery after charging
            remaining_battery = vehicle_battery_capacity

        # Update remaining battery
        remaining_battery -= distance_to_travel / vehicle_consumption_rate

    map.save('routes_comparison.html')

    # Return the updated charging station ID to charge
    return charging_station_id_to_charge


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

# Visualize the route in motion
charging_station_id_to_charge = None
for i in range(1, min_len):
    charging_station_id_to_charge = visualize_routes_in_motion(shortest_distance_route, shortest_time_route,
                                                               charging_station_id_to_charge)
