"""
Graph Map — Competition Track Graph Loader

Loads the BFMC competition track from a GraphML file.
The track is represented as a directed graph where:
- Nodes have (x, y) coordinates on the physical track
- Edges represent drivable connections between nodes
- Dotted edges represent special connections (e.g., lane changes,
  alternative paths that are not part of the normal driving lane)

The graph is used by PathPlanning to compute routes and by
DecisionMaking to identify upcoming road features (crossings,
roundabouts, etc.) based on node connectivity patterns.
"""

import xml.etree.ElementTree as ET
import math
from typing import Dict, List, Tuple, Optional


class GraphNode:
    """Represents a node (waypoint) on the competition track."""

    __slots__ = ("id", "x", "y")

    def __init__(self, node_id: str, x: float, y: float):
        self.id = node_id
        self.x = x
        self.y = y

    def distance_to(self, other: "GraphNode") -> float:
        """Euclidean distance to another node."""
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def __repr__(self):
        return f"GraphNode(id={self.id}, x={self.x}, y={self.y})"


class GraphEdge:
    """Represents a directed edge (road segment) between two nodes."""

    __slots__ = ("source", "target", "dotted", "weight")

    def __init__(self, source: str, target: str, dotted: bool = False, weight: float = 0.0):
        self.source = source
        self.target = target
        self.dotted = dotted
        self.weight = weight  # will be set to Euclidean distance

    def __repr__(self):
        return f"GraphEdge({self.source} -> {self.target}, dotted={self.dotted}, weight={self.weight:.2f})"


class GraphMap:
    """
    Loads and provides access to the competition track graph.
    
    The track graph is loaded from a GraphML XML file containing:
    - Nodes with x, y coordinates
    - Directed edges with a 'dotted' attribute
    
    Provides methods for:
    - Finding nearest nodes to a given position
    - Getting neighbors (successors) of a node
    - Computing shortest paths (Dijkstra)
    - Identifying intersection nodes (nodes with multiple outgoing edges)
    
    Usage:
        graph = GraphMap("Competition_track_graph.graphml")
        nearest = graph.get_nearest_node(4.5, 6.8)
        neighbors = graph.get_neighbors(nearest.id)
        path = graph.shortest_path("1", "50")
    """

    def __init__(self, graphml_path: str):
        """
        Load the track graph from a GraphML file.
        
        Args:
            graphml_path: Path to the .graphml file.
        """
        self.nodes: Dict[str, GraphNode] = {}
        self.edges: List[GraphEdge] = []

        # Adjacency list: node_id -> list of (neighbor_id, edge)
        self._adjacency: Dict[str, List[Tuple[str, GraphEdge]]] = {}

        # Reverse adjacency: node_id -> list of (predecessor_id, edge)
        self._reverse_adjacency: Dict[str, List[Tuple[str, GraphEdge]]] = {}

        self._load_graphml(graphml_path)
        self._build_adjacency()

    def _load_graphml(self, path: str):
        """Parse the GraphML file and populate nodes and edges."""
        tree = ET.parse(path)
        root = tree.getroot()

        # GraphML uses namespaces
        ns = {"graphml": "http://graphml.graphdrawing.org/xmlns"}

        # Determine key mappings (d0=x, d1=y, d2=dotted)
        key_map = {}
        for key_elem in root.findall("graphml:key", ns):
            key_id = key_elem.get("id")
            attr_name = key_elem.get("attr.name")
            key_map[key_id] = attr_name

        graph = root.find("graphml:graph", ns)
        if graph is None:
            raise ValueError(f"No <graph> element found in {path}")

        # Parse nodes
        for node_elem in graph.findall("graphml:node", ns):
            node_id = node_elem.get("id")
            x = 0.0
            y = 0.0
            for data_elem in node_elem.findall("graphml:data", ns):
                key = data_elem.get("key")
                attr_name = key_map.get(key, key)
                value = data_elem.text.strip() if data_elem.text else "0"
                if attr_name == "x":
                    x = float(value)
                elif attr_name == "y":
                    y = float(value)

            self.nodes[node_id] = GraphNode(node_id, x, y)

        # Parse edges
        for edge_elem in graph.findall("graphml:edge", ns):
            source = edge_elem.get("source")
            target = edge_elem.get("target")
            dotted = False
            for data_elem in edge_elem.findall("graphml:data", ns):
                key = data_elem.get("key")
                attr_name = key_map.get(key, key)
                if attr_name == "dotted":
                    dotted = data_elem.text.strip().lower() == "true"

            # Calculate edge weight (Euclidean distance)
            weight = 0.0
            if source in self.nodes and target in self.nodes:
                weight = self.nodes[source].distance_to(self.nodes[target])

            edge = GraphEdge(source, target, dotted, weight)
            self.edges.append(edge)

    def _build_adjacency(self):
        """Build adjacency lists from the edge list."""
        for node_id in self.nodes:
            self._adjacency[node_id] = []
            self._reverse_adjacency[node_id] = []

        for edge in self.edges:
            if edge.source in self.nodes and edge.target in self.nodes:
                self._adjacency[edge.source].append((edge.target, edge))
                self._reverse_adjacency[edge.target].append((edge.source, edge))

    # ────────────────────── Query Methods ──────────────────────

    def get_node(self, node_id: str) -> Optional[GraphNode]:
        """Get a node by its ID."""
        return self.nodes.get(node_id)

    def get_neighbors(self, node_id: str, include_dotted: bool = True) -> List[Tuple[str, GraphEdge]]:
        """
        Get all outgoing neighbors of a node.
        
        Args:
            node_id: The node to query.
            include_dotted: If False, exclude dotted (non-standard) edges.
        
        Returns:
            List of (neighbor_id, edge) tuples.
        """
        neighbors = self._adjacency.get(node_id, [])
        if not include_dotted:
            neighbors = [(nid, e) for nid, e in neighbors if not e.dotted]
        return neighbors

    def get_predecessors(self, node_id: str, include_dotted: bool = True) -> List[Tuple[str, GraphEdge]]:
        """Get all incoming predecessors of a node."""
        preds = self._reverse_adjacency.get(node_id, [])
        if not include_dotted:
            preds = [(nid, e) for nid, e in preds if not e.dotted]
        return preds

    def get_nearest_node(self, x: float, y: float) -> Optional[GraphNode]:
        """
        Find the node closest to the given (x, y) coordinates.
        
        Args:
            x: X coordinate on the track.
            y: Y coordinate on the track.
        
        Returns:
            The nearest GraphNode, or None if graph is empty.
        """
        if not self.nodes:
            return None

        best_node = None
        best_dist = float("inf")
        for node in self.nodes.values():
            d = math.sqrt((node.x - x) ** 2 + (node.y - y) ** 2)
            if d < best_dist:
                best_dist = d
                best_node = node
        return best_node

    def get_nearest_nodes(self, x: float, y: float, k: int = 5) -> List[Tuple[GraphNode, float]]:
        """
        Find the k nearest nodes to the given position.
        
        Returns:
            List of (node, distance) tuples sorted by distance.
        """
        distances = []
        for node in self.nodes.values():
            d = math.sqrt((node.x - x) ** 2 + (node.y - y) ** 2)
            distances.append((node, d))
        distances.sort(key=lambda t: t[1])
        return distances[:k]

    def is_intersection(self, node_id: str) -> bool:
        """
        Check if a node is an intersection (has multiple outgoing non-dotted edges).
        
        Intersection nodes have 2+ outgoing edges, meaning the car must
        choose a direction.
        """
        neighbors = self.get_neighbors(node_id, include_dotted=False)
        return len(neighbors) >= 2

    def get_intersection_nodes(self) -> List[str]:
        """Get all intersection node IDs (nodes with 2+ outgoing edges)."""
        return [nid for nid in self.nodes if self.is_intersection(nid)]

    def shortest_path(self, start_id: str, end_id: str, avoid_dotted: bool = False) -> Optional[List[str]]:
        """
        Compute shortest path between two nodes using Dijkstra's algorithm.
        
        Args:
            start_id: Starting node ID.
            end_id: Destination node ID.
            avoid_dotted: If True, do not use dotted edges.
        
        Returns:
            List of node IDs forming the shortest path, or None if no path exists.
        """
        import heapq

        if start_id not in self.nodes or end_id not in self.nodes:
            return None

        dist = {nid: float("inf") for nid in self.nodes}
        prev = {nid: None for nid in self.nodes}
        dist[start_id] = 0.0

        # Priority queue: (distance, node_id)
        pq = [(0.0, start_id)]

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == end_id:
                break

            for neighbor_id, edge in self.get_neighbors(u, include_dotted=not avoid_dotted):
                new_dist = dist[u] + edge.weight
                if new_dist < dist[neighbor_id]:
                    dist[neighbor_id] = new_dist
                    prev[neighbor_id] = u
                    heapq.heappush(pq, (new_dist, neighbor_id))

        # Reconstruct path
        if dist[end_id] == float("inf"):
            return None

        path = []
        current = end_id
        while current is not None:
            path.append(current)
            current = prev[current]
        path.reverse()
        return path

    def get_path_length(self, path: List[str]) -> float:
        """Calculate total length of a path (list of node IDs)."""
        total = 0.0
        for i in range(len(path) - 1):
            if path[i] in self.nodes and path[i + 1] in self.nodes:
                total += self.nodes[path[i]].distance_to(self.nodes[path[i + 1]])
        return total

    @property
    def node_count(self) -> int:
        return len(self.nodes)

    @property
    def edge_count(self) -> int:
        return len(self.edges)

    def __repr__(self):
        return f"GraphMap(nodes={self.node_count}, edges={self.edge_count})"
