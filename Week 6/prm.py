#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import heapq
import rospy

# ROS node initialization
rospy.init_node('prm_node', anonymous=True)

# Parameters
num_nodes = 100
k_nearest = 5
start_pos = np.array([0.0, 0.0])
goal_pos = np.array([20.0, 20.0])
x_max, y_max = 20, 20

# Generate random nodes
np.random.seed(42)
nodes = np.random.rand(num_nodes, 2) * [x_max, y_max]
nodes = np.vstack((start_pos, nodes, goal_pos))

# Build KDTree for nearest neighbor search
tree = KDTree(nodes)

# Find edges using k-nearest neighbors
edges = []
for i in range(len(nodes)):
    distances, neighbors = tree.query(nodes[i], k=k_nearest + 1)
    for j in range(1, k_nearest + 1):
        neighbor = neighbors[j]
        if neighbor != i:
            edges.append((i, neighbor))

# Dijkstra's algorithm for shortest path
def dijkstra(nodes, edges, start, goal):
    graph = {i: [] for i in range(len(nodes))}
    for (src, dst) in edges:
        dist = np.linalg.norm(nodes[src] - nodes[dst])
        graph[src].append((dist, dst))
        graph[dst].append((dist, src))

    # Priority queue for Dijkstra
    queue = [(0, start)]
    distances = {node: float("inf") for node in range(len(nodes))}
    distances[start] = 0
    previous_nodes = {node: None for node in range(len(nodes))}
    
    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_node == goal:
            break
        
        for neighbor_distance, neighbor in graph[current_node]:
            distance = current_distance + neighbor_distance
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))
    
    # Reconstruct path
    path = []
    node = goal
    while node is not None:
        path.insert(0, node)
        node = previous_nodes[node]
    return path

# Get the shortest path from start to goal
start_index, goal_index = 0, len(nodes) - 1
path = dijkstra(nodes, edges, start_index, goal_index)

# Visualization with Matplotlib
plt.figure()
plt.title("Probabilistic Roadmap (PRM) with Shortest Path")

# Draw edges
for (i, j) in edges:
    plt.plot([nodes[i][0], nodes[j][0]], [nodes[i][1], nodes[j][1]], "r-", linewidth=0.5)

# Draw the shortest path
if path:
    plt.plot(nodes[path][:, 0], nodes[path][:, 1], "y-", linewidth=2, label="Shortest Path")

# Draw nodes
plt.scatter(nodes[:, 0], nodes[:, 1], c="blue")
plt.scatter(*start_pos, c="green", s=100, label="Start")
plt.scatter(*goal_pos, c="red", s=100, label="Goal")

# Labels and Legend
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.xlim(0, x_max)
plt.ylim(0, y_max)

# Show the plot
plt.show(block=True)  # Ensure the plot blocks execution so it remains open