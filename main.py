import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import math

# Load datasets
districts_df = pd.read_csv('districts.csv')  # Adjust file path
stations_df = pd.read_csv('charging_stations.csv')  # Adjust file path
roads_df = pd.read_csv('roads.csv')  # Adjust file path

# Create a graph
G = nx.DiGraph()

# Add nodes for districts
for index, row in districts_df.iterrows():
    G.add_node(row['DistrictID'], pos=(row['Longitude'], row['Latitude']), type='district', name=row['DistrictName'])

# Add nodes for charging stations
for index, row in stations_df.iterrows():
    G.add_node(row['ChargingStationID'], pos=(row['Longitude'], row['Latitude']), type='station',
               power=row['Power'], cost_per_kwh=row['CostPerKWh'], name=row['ChargingStationID'])

# Add edges for roads
for index, row in roads_df.iterrows():
    G.add_edge(row['StartPointID'], row['EndPointID'], distance=row['Distance'])

# Basic visualization to check the graph
pos = nx.get_node_attributes(G, 'pos')
nx.draw(G, pos, with_labels=True, node_size=700, node_color="lightblue")
plt.show()

# Shortest path, minimum time, and cost calculation function with charging constraints
def calculate_shortest_path_with_charging(start_node, end_node, vehicle_battery_capacity, vehicle_consumption_rate,
                                          min_charge_threshold, intermediate_nodes=None):
    try:
        # Map location names to node IDs
        start_node = districts_df.loc[districts_df['DistrictName'] == start_node, 'DistrictID'].iloc[0]
        end_node = districts_df.loc[districts_df['DistrictName'] == end_node, 'DistrictID'].iloc[0]
        if intermediate_nodes:
            intermediate_nodes = [districts_df.loc[districts_df['DistrictName'] == node, 'DistrictID'].iloc[0]
                                  for node in intermediate_nodes]

        # Initialize battery level
        battery_level = vehicle_battery_capacity

        # Initialize variables
        total_distance = 0
        total_cost = 0

        current_node = start_node

        # Initialize lists to store paths and distances for each segment
        paths = []
        distances = []

        # Iterate over each pair of consecutive locations (including the start and end points)
        for i in range(len(intermediate_nodes)):
            destination_node = intermediate_nodes[i]

            # Find the shortest path to the destination
            path_to_destination = nx.shortest_path(G, source=current_node, target=destination_node, weight='distance')
            paths.append(path_to_destination)

            # Iterate through the path to calculate distance and check for charging stations
            distance_to_destination = 0
            for j in range(len(path_to_destination) - 1):
                source = path_to_destination[j]
                target = path_to_destination[j + 1]

                # Calculate distance to the next node
                distance_to_next_node = G.edges[(source, target)]['distance']
                distance_to_destination += distance_to_next_node

                # Check if the node is a charging station
                if G.nodes[target]['type'] == 'station':
                    # Calculate energy required to reach the charging station
                    energy_required = distance_to_next_node * vehicle_consumption_rate

                    # If battery level is below threshold, charge at the station
                    if battery_level <= min_charge_threshold:
                        charging_time = math.ceil((vehicle_battery_capacity - battery_level) /
                                                  (G.nodes[target]['power'] / 60))  # Charging in minutes
                        total_cost += G.nodes[target]['cost_per_kwh'] * (
                                vehicle_battery_capacity - battery_level)
                        battery_level = vehicle_battery_capacity

            distances.append(distance_to_destination)
            total_distance += distance_to_destination
            current_node = destination_node

        # Find the shortest path back to the starting node
        path_to_start_node = nx.shortest_path(G, source=current_node, target=start_node, weight='distance')
        paths.append(path_to_start_node)

        # Iterate through the path to calculate distance and check for charging stations
        distance_to_start_node = 0
        for j in range(len(path_to_start_node) - 1):
            source = path_to_start_node[j]
            target = path_to_start_node[j + 1]

            # Calculate distance to the next node
            distance_to_next_node = G.edges[(source, target)]['distance']
            distance_to_start_node += distance_to_next_node

            # Check if the node is a charging station
            if G.nodes[target]['type'] == 'station':
                # Calculate energy required to reach the charging station
                energy_required = distance_to_next_node * vehicle_consumption_rate

                # If battery level is below threshold, charge at the station
                if battery_level <= min_charge_threshold:
                    charging_time = math.ceil((vehicle_battery_capacity - battery_level) /
                                              (G.nodes[target]['power'] / 60))  # Charging in minutes
                    total_cost += G.nodes[target]['cost_per_kwh'] * (
                                vehicle_battery_capacity - battery_level)
                    battery_level = vehicle_battery_capacity

        distances.append(distance_to_start_node)
        total_distance += distance_to_start_node

        # Print the paths
        for i in range(len(paths)):
            print(f"Segment {i + 1} path: {paths[i]}")
            print(f"Segment {i + 1} distance: {distances[i]} km")

        # Calculate total time (assuming uniform speed)
        total_time = total_distance / 60  # Assuming average speed of 60 km/h
        print("Total time:", total_time)

        print("Total cost:", total_cost)

        # Visualization
        plt.figure(figsize=(10, 6))
        nx.draw(G, pos, with_labels=True, node_size=500, node_color="lightblue", edgelist=[], width=2)

        # Annotate nodes with location names
        node_labels = {node: G.nodes[node]['name'] for node in G.nodes}
        nx.draw_networkx_labels(G, pos, labels=node_labels, font_size=10, font_color='black')

        # Draw edges for each path segment
        for path in paths:
            path_edges = list(zip(path, path[1:]))
            nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="red", width=2)

        plt.show()


    except nx.NetworkXNoPath:
        print("No path found")


# Example usage
start_location = 'Mumbai'  # Adjust to your starting point
end_location = 'Pune'  # Adjust to your destination
intermediate_locations = ['Nashik', 'Shirdi', 'Lonavala']  # Adjust to intermediate locations to visit
vehicle_battery_capacity = 100  # Adjust to your vehicle's battery capacity in kWh
vehicle_consumption_rate = 0.2  # Adjust to your vehicle's energy consumption rate in kWh/km
min_charge_threshold = 20  # Adjust to your vehicle's minimum charge threshold in kWh

calculate_shortest_path_with_charging(start_location, end_location, vehicle_battery_capacity, vehicle_consumption_rate,
                                      min_charge_threshold, intermediate_locations)
