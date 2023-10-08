from collections import deque
import heapq

# MUSIIMENTA CISSYLYN 22/U/6402
# number 6
custom_weighted_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'B': 1, 'A': 2, 'C': 3},
    'C': {'A': 2, 'B': 3, 'D': 4, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'C': 4, 'D': 1}
}

custom_heuristics = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

# Breadth-First Search (BFS)
def bfs_search(graph, start, goal):
    visited = set()
    queue = deque([(start, [start])])
    expanded_nodes = []

    while queue:
        node, path = queue.popleft()
        expanded_nodes.append(node)
        if node == goal:
            return path, expanded_nodes, []  
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

    return None, expanded_nodes, []  

# Depth-First Search (DFS)
def dfs_search(graph, start, goal):
    visited = set()
    stack = [(start, [start])]
    expanded_nodes = []

    while stack:
        node, path = stack.pop()
        expanded_nodes.append(node)
        if node == goal:
            return path, expanded_nodes, []  
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in reversed(neighbors):
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))

    return None, expanded_nodes, []  

# Uniform Cost Search (UCS)
def ucs_search(graph, start, goal):
    visited = set()
    priority_queue = [(0, start, [start])]
    expanded_nodes = []

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
        expanded_nodes.append(node)
        if node == goal:
            return path, expanded_nodes, []  
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    new_cost = cost + graph[node][neighbor]
                    heapq.heappush(priority_queue, (new_cost, neighbor, path + [neighbor]))

    return None, expanded_nodes, []  

# A* Search
def astar_search(graph, start, goal, heuristics):
    visited = set()
    priority_queue = [(heuristics[start], start, [start])]
    expanded_nodes = []

    while priority_queue:
        if len(priority_queue) == 0:
            break  
        _, node, path = heapq.heappop(priority_queue)
        expanded_nodes.append(node)
        if node == goal:
            return path, expanded_nodes

        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    cost = path_cost(path) + graph[node][neighbor]
                    heapq.heappush(priority_queue, (cost + heuristics[neighbor], neighbor, path + [neighbor]))

    return None, expanded_nodes

# Greedy Search
def greedy_search(graph, start, goal, heuristics):
    visited = set()
    priority_queue = [(heuristics[start], start, [start])]
    expanded_nodes = []

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)
        expanded_nodes.append(node)
        if node == goal:
            return path, expanded_nodes
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    heapq.heappush(priority_queue, (heuristics[neighbor], neighbor, path + [neighbor]))

    unexpanded_nodes = [node for node in graph if node not in visited]
    return None, expanded_nodes, unexpanded_nodes


def path_cost(path):
    cost = 0
    for i in range(len(path) - 1):
        cost += custom_weighted_graph[path[i]][path[i + 1]]
    return cost


start_node_custom = 'S'
goal_node_custom = 'G'


bfs_path, bfs_expanded, bfs_unexpanded = bfs_search(custom_weighted_graph, start_node_custom, goal_node_custom)
print("BFS Path:", bfs_path if bfs_path else "Not Found")
print("BFS Expanded Nodes:", bfs_expanded)
print("BFS Unexpanded Nodes:", bfs_unexpanded)

dfs_path, dfs_expanded, dfs_unexpanded = dfs_search(custom_weighted_graph, start_node_custom, goal_node_custom)
print("DFS Path:", dfs_path if dfs_path else "Not Found")
print("DFS Expanded Nodes:", dfs_expanded)
print("DFS Unexpanded Nodes:", dfs_unexpanded)

ucs_path, ucs_expanded, ucs_unexpanded = ucs_search(custom_weighted_graph, start_node_custom, goal_node_custom)
print("UCS Path:", ucs_path if ucs_path else "Not Found")
print("UCS Expanded Nodes:", ucs_expanded)
print("UCS Unexpanded Nodes:", ucs_unexpanded)
