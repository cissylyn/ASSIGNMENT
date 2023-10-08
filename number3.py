# MUSIIMENTA CISSYLYN 22/U/6402
import heapq
from collections import deque

# NUMBER 3
tree_search_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'C': 3, 'S': 1, 'A': 2},
    'C': {'A': 2, 'D': 4, 'B': 3, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'D': 1, 'C': 4}
}
heuristic_values = {'S': 7, 'B': 7, 'A': 5, 'C': 4, 'D': 1, 'G': 0}

def tree_search_bfs(graph, start_node, goal_node):
    queue = deque([(start_node, [])])  

    while queue:
        current_node, path = queue.popleft()

        if current_node == goal_node:
            print("BFS Tree Search Path from 'S' to 'G':", ' -> '.join(path + [current_node]))
            return

        for neighbor in graph.get(current_node, []):
            if neighbor not in path:
                queue.append((neighbor, path + [current_node]))

    print("No valid path found")

def tree_search_dfs(graph, start_node, goal_node, visited=None, path=None):
    if visited is None:
        visited = set()
    if path is None:
        path = []

    visited.add(start_node)
    path = path + [start_node]

    if start_node == goal_node:
        print(" DFS Tree Search Path from 'S' to 'G':", ' -> '.join(path))
        return path

    for neighbor in graph.get(start_node, []):
        if neighbor not in visited:
            new_path = tree_search_dfs(graph, neighbor, goal_node, visited, path)
            if new_path:
                return new_path

    return None

def tree_search_ucs(graph, start_node, goal_node):
    priority_queue = [(0, start_node, [])]  

    while priority_queue:
        cost, current_node, path = heapq.heappop(priority_queue)

        if current_node == goal_node:
            print("UCS Tree Search Path from 'S' to 'G':", ' -> '.join(path + [current_node]))
            return

        for neighbor, weight in graph.get(current_node, {}).items():
            if neighbor not in path:
                heapq.heappush(priority_queue, (cost + weight, neighbor, path + [current_node]))

    print("No valid path found")

def tree_search_greedy(graph, start_node, goal_node, heuristic):
    priority_queue = [(heuristic[start_node], start_node, [])]  

    while priority_queue:
        h, current_node, path = heapq.heappop(priority_queue)

        if current_node == goal_node:
            print("Greedy Tree Search Path from 'S' to 'G':", ' -> '.join(path + [current_node]))
            return

        for neighbor, weight in graph.get(current_node, {}).items():
            if neighbor not in path:
                heapq.heappush(priority_queue, (heuristic[neighbor], neighbor, path + [current_node]))

    print("No valid path found")

def astar_search(graph, start, goal, heuristic):
    open_set = [(0, start)]  # Priority queue: (f(n), node)
    g_values = {node: float('infinity') for node in graph}
    g_values[start] = 0
    came_from = {}  # Initialize the dictionary to store parent nodes

    while open_set:
        f, current_node = heapq.heappop(open_set)

        if current_node == goal:
            # Path found, reconstruct and return it
            path = []
            while current_node != start:
                path.insert(0, current_node)  # Insert node at the beginning of the path
                current_node = came_from[current_node]
            path.insert(0, start)
            print("A* Search Path from 'S' to 'G':", ' -> '.join(path))  # Print the path
            return path

        for neighbor, weight in graph.get(current_node, {}).items():
            tentative_g = g_values[current_node] + weight
            if tentative_g < g_values.get(neighbor, float('infinity')):
                # This path to the neighbor is better than any previous one
                g_values[neighbor] = tentative_g
                f = tentative_g + heuristic.get(neighbor, 0)  # Use 0 as default heuristic value if not provided
                heapq.heappush(open_set, (f, neighbor))
                came_from[neighbor] = current_node

    print("No valid path found using A* algorithm.")
    return None  # No path found

start_node = 'S'
goal_node = 'G'

tree_search_bfs(tree_search_graph, start_node, goal_node)
tree_search_dfs(tree_search_graph, start_node, goal_node)
tree_search_ucs(tree_search_graph, start_node, goal_node)
tree_search_greedy(tree_search_graph, start_node, goal_node, heuristic_values)
astar_search(tree_search_graph, start_node, goal_node, heuristic_values)
