import matplotlib.pyplot as plt
import networkx as nx
import csv

# Load datasets from CSV files
districts = []
charging_stations = []
road_network = []

# Load District Points Dataset
with open('11districts.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # Skip header
    for row in reader:
        districts.append((int(row[0]), row[1], float(row[2]), float(row[3])))
# Load EV Charging Stations Dataset
with open('22charging_stations.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # Skip header
    for row in reader:
        print("Row:", row)  # Print the entire row to see its contents

        # Ensure that the third and fourth columns are float values
        try:
            latitude = float(row[2])
            longitude = float(row[3])
        except ValueError:
            print("Error: Latitude and longitude should be numerical values.")
            print("Skipping row:", row)
            continue

        # Append the charging station data
        charging_stations.append((row[0], row[1], latitude, longitude))

# Load Road Network Dataset
with open('33newroads.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # Skip header
    for row in reader:
        road_network.append((int(row[0]), int(row[1]), int(row[2])))

# Create a graph
G = nx.Graph()

# Add districts as nodes
for district in districts:
    G.add_node(district[0], pos=(district[2], district[3]), label=district[1])

# Add charging stations as nodes
for station in charging_stations:
    G.add_node(station[0], pos=(station[2], station[3]), label=station[1])

# Add road connections as edges
for road in road_network:
    G.add_edge(road[1], road[2])

# Get positions for plotting
pos = nx.get_node_attributes(G, 'pos')

# Draw nodes
nx.draw_networkx_nodes(G, pos, nodelist=[node[0] for node in districts], node_color='b', node_size=300)
nx.draw_networkx_nodes(G, pos, nodelist=[node[0] for node in charging_stations], node_color='g', node_size=300)

# Draw edges
nx.draw_networkx_edges(G, pos, edgelist=road_network, width=2, alpha=0.5, edge_color='k')

# Draw labels
nx.draw_networkx_labels(G, pos, labels={node[0]: node[1]['label'] for node in G.nodes(data=True)}, font_size=10)

# Set plot limits
plt.xlim(min(pos[node][0] for node in G.nodes()) - 0.5, max(pos[node][0] for node in G.nodes()) + 0.5)
plt.ylim(min(pos[node][1] for node in G.nodes()) - 0.5, max(pos[node][1] for node in G.nodes()) + 0.5)

# Add legend
plt.legend(['Districts', 'Charging Stations'], loc='upper right')

# Display plot
plt.title('Road Network Visualization with Districts and Charging Stations')
plt.axis('off')
plt.show()
