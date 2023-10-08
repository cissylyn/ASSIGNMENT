# number 2
# MUSIIMENTA CISSYLYN 21/U/6402
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'C': 3, 'S': 1, 'A': 2},
    'C': {'A': 2, 'D': 4, 'B': 3, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'D': 1, 'C': 4}
}
heuristic_values = {'S': 7, 'B': 7, 'A': 5, 'C': 4, 'D': 1, 'G': 0}

def print_graph_and_heuristic(graph, heuristic_values):
    print("Graph:")
    print("Nodes:", list(graph.keys()))
    print("Edges with weights:")
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors.items():
            print(f"({node} - {neighbor}): {weight}")

    print("\nHeuristic Values:")
    for node, heuristic in heuristic_values.items():
        print(f"{node}: {heuristic}")

# Print the graph and heuristic values
print_graph_and_heuristic(graph, heuristic_values)
