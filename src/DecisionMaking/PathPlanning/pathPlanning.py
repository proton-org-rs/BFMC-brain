"""
Path Planning Module

Responsible for:
1. Loading the competition track graph (GraphML)
2. Loading a planned route from a JSON file (planned_activities.json — NOT YET IMPLEMENTED)
3. Providing the current route (sequence of node IDs)
4. Determining upcoming road features based on the route and graph topology

The Path Planning module feeds into the Decision Making component,
which uses upcoming features to decide the appropriate driving mode.

Future: planned_activities.json will contain:
    {
        "route": ["1", "150", "149", ...],
        "activities": [
            {"at_node": "150", "action": "TURN_LEFT"},
            {"at_node": "42",  "action": "PARK", "spot": 2},
            ...
        ]
    }
"""

import os
import math
from typing import List, Optional, Dict, Any

from src.DecisionMaking.PathPlanning.graphMap import GraphMap, GraphNode


class PathPlanning:
    """
    Path Planning component for the autonomous vehicle.
    
    Loads the competition track graph and provides route-related
    information to the Decision Making component.
    
    Args:
        graphml_path: Path to the competition track GraphML file.
        logger: Logger for debug messages.
    """

    def __init__(self, graphml_path: str, logger=None):
        self.logger = logger
        self._log("Initializing Path Planning...")

        # Load the competition track graph
        self.graph = GraphMap(graphml_path)
        self._log(f"Graph loaded: {self.graph}")

        # Current planned route (list of node IDs)
        self._route: List[str] = []

        # Current position index in the route
        self._route_index: int = 0

        # Planned activities (from JSON — placeholder, not loaded yet)
        self._planned_activities: List[Dict[str, Any]] = []

        # Precompute intersection nodes for quick lookup
        self._intersection_nodes = set(self.graph.get_intersection_nodes())
        self._log(f"Found {len(self._intersection_nodes)} intersection nodes")

    def _log(self, message: str):
        if self.logger:
            self.logger.info(f"[PathPlanning] {message}")

    # ──────────────────── Route Management ────────────────────────

    def set_route(self, node_ids: List[str]):
        """
        Set the planned route manually (list of node IDs).
        
        Args:
            node_ids: Ordered list of node IDs forming the route.
        """
        self._route = node_ids
        self._route_index = 0
        self._log(f"Route set with {len(node_ids)} waypoints")

    def compute_route(self, start_id: str, end_id: str) -> bool:
        """
        Compute the shortest route between two nodes on the track.
        
        Args:
            start_id: Starting node ID.
            end_id: Destination node ID.
        
        Returns:
            True if a route was found, False otherwise.
        """
        path = self.graph.shortest_path(start_id, end_id)
        if path is None:
            self._log(f"No route found from {start_id} to {end_id}")
            return False

        self._route = path
        self._route_index = 0
        length = self.graph.get_path_length(path)
        self._log(f"Route computed: {len(path)} waypoints, length={length:.2f}m")
        return True

    def get_route(self) -> List[str]:
        """Get the current planned route."""
        return self._route.copy()

    def get_route_length(self) -> float:
        """Get total length of the current route."""
        return self.graph.get_path_length(self._route)

    # ──────────────────── Position Tracking ───────────────────────

    def update_position(self, x: float, y: float) -> Optional[str]:
        """
        Update the vehicle's position and advance the route index.
        
        Finds the nearest node on the route to the given position and
        updates the route progress accordingly.
        
        Args:
            x: Current x coordinate.
            y: Current y coordinate.
        
        Returns:
            The ID of the nearest route node, or None if no route is set.
        """
        if not self._route:
            return None

        # Find closest node in the remaining route
        best_idx = self._route_index
        best_dist = float("inf")

        # Search a window around current position (performance)
        search_start = max(0, self._route_index - 2)
        search_end = min(len(self._route), self._route_index + 15)

        for i in range(search_start, search_end):
            node = self.graph.get_node(self._route[i])
            if node is None:
                continue
            d = math.sqrt((node.x - x) ** 2 + (node.y - y) ** 2)
            if d < best_dist:
                best_dist = d
                best_idx = i

        # Only advance forward (don't go backwards on the route)
        if best_idx >= self._route_index:
            self._route_index = best_idx

        return self._route[self._route_index]

    def get_current_node_id(self) -> Optional[str]:
        """Get the current node ID on the route."""
        if not self._route or self._route_index >= len(self._route):
            return None
        return self._route[self._route_index]

    def get_current_node(self) -> Optional[GraphNode]:
        """Get the current GraphNode on the route."""
        node_id = self.get_current_node_id()
        if node_id is None:
            return None
        return self.graph.get_node(node_id)

    def get_progress(self) -> float:
        """Get route progress as a percentage (0.0 to 100.0)."""
        if not self._route:
            return 0.0
        return (self._route_index / max(1, len(self._route) - 1)) * 100.0

    def is_route_complete(self) -> bool:
        """Check if the vehicle has reached the end of the route."""
        if not self._route:
            return True
        return self._route_index >= len(self._route) - 1

    # ──────────────────── Look-Ahead Features ─────────────────────

    def get_upcoming_nodes(self, count: int = 5) -> List[str]:
        """
        Get the next N node IDs on the route (look-ahead).
        
        Args:
            count: Number of upcoming nodes to return.
        
        Returns:
            List of upcoming node IDs.
        """
        if not self._route:
            return []

        start = self._route_index + 1
        end = min(len(self._route), start + count)
        return self._route[start:end]

    def is_intersection_ahead(self, look_ahead: int = 5) -> Optional[str]:
        """
        Check if there's an intersection in the upcoming nodes.
        
        Args:
            look_ahead: How many nodes ahead to check.
        
        Returns:
            The node ID of the nearest upcoming intersection, or None.
        """
        upcoming = self.get_upcoming_nodes(look_ahead)
        for nid in upcoming:
            if nid in self._intersection_nodes:
                return nid
        return None

    def get_distance_to_node(self, target_node_id: str) -> float:
        """
        Calculate the route distance from the current position to a target node.
        
        Args:
            target_node_id: The target node ID.
        
        Returns:
            Distance along the route, or float('inf') if not reachable.
        """
        if not self._route:
            return float("inf")

        try:
            target_idx = self._route.index(target_node_id, self._route_index)
        except ValueError:
            return float("inf")

        # Sum distances along route segments
        total = 0.0
        for i in range(self._route_index, target_idx):
            n1 = self.graph.get_node(self._route[i])
            n2 = self.graph.get_node(self._route[i + 1])
            if n1 and n2:
                total += n1.distance_to(n2)
        return total

    def get_upcoming_features(self, look_ahead: int = 10) -> List[Dict[str, Any]]:
        """
        Identify upcoming road features on the route.
        
        Analyzes the graph topology for the next N nodes to identify:
        - Intersections (nodes with 2+ outgoing edges)
        - Dead ends (nodes with 0 outgoing)
        - High-connectivity nodes (roundabouts candidate)
        
        Args:
            look_ahead: How many nodes ahead to analyze.
        
        Returns:
            List of dicts: [{"node_id": "...", "type": "intersection", "distance": ...}, ...]
        """
        features = []
        upcoming = self.get_upcoming_nodes(look_ahead)

        for nid in upcoming:
            node = self.graph.get_node(nid)
            if node is None:
                continue

            out_edges = self.graph.get_neighbors(nid, include_dotted=False)
            in_edges = self.graph.get_predecessors(nid, include_dotted=False)
            out_count = len(out_edges)
            in_count = len(in_edges)

            distance = self.get_distance_to_node(nid)

            if out_count >= 3 or in_count >= 3:
                features.append({
                    "node_id": nid,
                    "type": "roundabout_candidate",
                    "out_degree": out_count,
                    "in_degree": in_count,
                    "distance": distance,
                })
            elif out_count >= 2:
                features.append({
                    "node_id": nid,
                    "type": "intersection",
                    "out_degree": out_count,
                    "in_degree": in_count,
                    "distance": distance,
                })
            elif out_count == 0:
                features.append({
                    "node_id": nid,
                    "type": "dead_end",
                    "distance": distance,
                })

        return features

    # ──────────────────── Nearest Node Queries ────────────────────

    def get_nearest_node_to_position(self, x: float, y: float) -> Optional[GraphNode]:
        """Find the nearest graph node to a given (x, y) position."""
        return self.graph.get_nearest_node(x, y)

    # ──────────────────── Planned Activities (Future) ─────────────

    def load_planned_activities(self, json_path: str):
        """
        Load planned activities from a JSON file.
        
        NOT YET IMPLEMENTED — placeholder for future implementation.
        The JSON will contain the pre-planned route and actions
        at specific nodes (turns, parking, etc.).
        
        Args:
            json_path: Path to the planned_activities.json file.
        """
        # TODO: Implement when planned_activities.json format is defined
        self._log(f"[STUB] load_planned_activities({json_path}) — not yet implemented")
        pass

    def get_planned_activity_at_node(self, node_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the planned activity at a specific node.
        
        NOT YET IMPLEMENTED — placeholder for future implementation.
        
        Args:
            node_id: The node ID to check for planned activities.
        
        Returns:
            Activity dict, or None if no activity planned at this node.
        """
        # TODO: Implement when planned_activities.json is loaded
        for activity in self._planned_activities:
            if activity.get("at_node") == node_id:
                return activity
        return None

    def __repr__(self):
        return (
            f"PathPlanning(graph={self.graph}, "
            f"route_len={len(self._route)}, "
            f"progress={self.get_progress():.1f}%)"
        )
