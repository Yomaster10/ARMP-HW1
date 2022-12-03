import math

class Node:
    def __init__(self, center, name):
        self.center = center
        self.name = name

    def __eq__(self, other): #O(1)
        if self.name == other.name:
            return True
        if self.center[0] == other.center[0] and self.center[1] == other.center[1]:
            return True
        return False

class Graph:
    def __init__(self, nodes):
        self.nodes = nodes
        self.graph = {}
    
    def Get_Outgoing_Edges(self, node_name): #O(|V|)
        connections = []
        for out_node in self.nodes: #O(|V|)
            if self.graph[node_name].get(out_node.name, False) != False:
                connections.append(out_node)
        return connections
    
    def Get_Edge_Value(self, node1, node2): #O(1)
        return self.graph[node1][node2]

def Get_Edge(node1, node2, lines): #O(|E|)
    """
    Identifies the LineString connecting two nodes, if it exists
    :param node1: The xy location of the first node
    :param node2: The xy location of the second node
    :param lines: A list of LineStrings holding the edges of the visibility graph
    :return: The LineString connecting the two input nodes (if it exists)
    """
    for l in lines: #O(|E|)
        if [node1,node2] == list(l.coords) or [node2,node1] == list(l.coords):
            return l
    return False

def Create_Graph(lines, source, dest): # O(|V|^2 * |E|)
    """
    Creates a searchable graph from a visibility graph, given the start and goal nodes
    :param lines: A list of LineStrings holding the edges of the visibility graph
    :param source: The xy location of the start node
    :param dest: The xy location of the goal node
    :return: The searchable graph object, as well as the start and goal node objects
    """
    start_node = Node(source, 'Start')
    goal_node = Node(dest, 'Goal')
    nodes = [start_node, goal_node]

    i = 0
    for l in lines: #O(|E|)
        [n1,n2] = list(l.coords)
        N1 = Node(n1, f'N{i}'); N2 = Node(n2, f'N{i+1}')
        bad1 = False; bad2 = False
        for n in nodes: #O(|V|), we note that this for loop changes in size as we iterate over the edges
            if N1 == n:
                bad1 = True
            if N2 == n:
                bad2 = True
        if not bad1:
            i += 2
            nodes.append(N1)
        if not bad2:
            i += 2
            nodes.append(N2)
    # Preparing the node list is O(|E||V|)

    graph = Graph(nodes)
    for n1 in nodes: #O(|V|)
        graph.graph[n1.name] = {}       
        for n2 in nodes: #O(|V|)
            line = Get_Edge(n1.center, n2.center, lines) #O(|E|)
            if line:
                graph.graph[n1.name][n2.name] = line.length

    # Building the graph is O(|V|^2 * |E|) - not the most efficient way to go about it
    return graph, start_node, goal_node

def Dijkstra(lines, source, dest): # O(|V|^2 * |E|), due to the graph construction
    """
    Conduct a Dijsktra search on a visibility graph, given the start and goal nodes
    :param lines: A list of LineStrings holding the edges of the visibility graph
    :param source: The xy location of the start node
    :param dest: The xy location of the goal node
    :return: The best path from the start node to the goal node and its cost
    """
    graph, start_node, goal_node = Create_Graph(lines, source, dest) # O(|V|^2 * |E|) 
    unvisited_nodes = graph.nodes
    shortest_path = {}
    previous_nodes = {}
    
    for node in unvisited_nodes: # O(|V|), doesn't affect the complexity of Dijkstra
        shortest_path[node.name] = math.inf
    shortest_path['Start'] = 0
    
    while len(unvisited_nodes) != 0: # O(|V|)
        current_min_node = None
        for node in unvisited_nodes: # O(log|V|) since the number of unvisited nodes decreases with every iteraton
            if current_min_node is None:
                current_min_node = node
            elif shortest_path[node.name] < shortest_path[current_min_node.name]:
                current_min_node = node

        # The code block below retrieves the current node's neighbors and updates their distances
        neighbors = graph.Get_Outgoing_Edges(current_min_node.name) # O(|V|)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node.name] + graph.Get_Edge_Value(current_min_node.name, neighbor.name)
            if tentative_value < shortest_path[neighbor.name]:
                shortest_path[neighbor.name] = tentative_value
                # We also update the best path to the current node
                previous_nodes[neighbor.name] = current_min_node
        unvisited_nodes.remove(current_min_node)
    # The complexity of the Dijkstra search itself is O(|V|*(|V|+log|V|))=O(|V|^2 + |V|log|V|)=O(|V|^2)

    # The complexity from here onward is negligible, since it can't be greater than O(|V|^2)
    if 'Goal' not in previous_nodes:
        print("\tPath could not be obtained...")
        return None, None

    node = goal_node; path = []
    while node.name != 'Start':
        path.append(node)
        node = previous_nodes[node.name]
    # Add the start node manually
    path.append(start_node)
    
    path_value = shortest_path[goal_node.name]
    best_path = [i.center for i in list(reversed(path))]
    print(f"Dijkstra search completed, shortest path has cost {path_value:.3f} and contains {len(best_path)} nodes.")

    return best_path, path_value