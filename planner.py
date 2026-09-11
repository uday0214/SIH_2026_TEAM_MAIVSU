"""
A* Pathfinding Planner on a moving local spatial lattice.
Directs the autonomous vehicle smoothly around potholes, traffic, and road curvature.
"""

import math
import heapq
from typing import List, Tuple, Optional, Set
from config import (
    PLANNER_CELL_SIZE, PLANNER_LOOKAHEAD_DIST,
    SAFETY_MARGIN_CAR, SAFETY_MARGIN_POTHOLE
)

class GridNode:
    __slots__ = ('col', 'row', 'x', 'y', 'g', 'h', 'f', 'parent')

    def __init__(self, col: int, row: int, x: float, y: float):
        self.col = col
        self.row = row
        self.x = x
        self.y = y
        self.g = float('inf')
        self.h = 0.0
        self.f = float('inf')
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f


class AStarPlanner:
    def __init__(self, road):
        self.road = road
        self.cell_size = PLANNER_CELL_SIZE
        self.lookahead_dist = PLANNER_LOOKAHEAD_DIST
        
        # Diagnostics
        self.last_path: List[Tuple[float, float]] = []
        self.last_explored_cells: List[Tuple[float, float]] = []
        self.last_obstacle_cells: List[Tuple[float, float]] = []
        self.nodes_explored_count = 0

    def plan_path(self, car_x: float, car_y: float, car_speed: float,
                  potholes: list, traffic: list) -> List[Tuple[float, float]]:
        """
        Runs A* search from car's current location up to lookahead horizon.
        Returns a smoothed list of world coordinates (x, y).
        """
        cell = self.cell_size
        H = self.lookahead_dist
        
        # 1. Define local bounding grid
        # Car is at the bottom of the grid, driving upwards (decreasing world Y)
        start_world_y = car_y
        goal_world_y = car_y - H
        
        road_center_ahead = self.road.get_road_center(goal_world_y)
        road_center_now = self.road.get_road_center(car_y)
        
        # Grid X span covers road boundaries + margins
        left_bound = min(self.road.get_road_edges(car_y)[0], self.road.get_road_edges(goal_world_y)[0]) - 50
        right_bound = max(self.road.get_road_edges(car_y)[1], self.road.get_road_edges(goal_world_y)[1]) + 50
        
        grid_min_x = math.floor(left_bound / cell) * cell
        grid_max_x = math.ceil(right_bound / cell) * cell
        
        num_cols = int((grid_max_x - grid_min_x) / cell) + 1
        num_rows = int(H / cell) + 2

        # Coordinate conversion helpers
        def world_to_grid(wx: float, wy: float) -> Tuple[int, int]:
            c = int(round((wx - grid_min_x) / cell))
            r = int(round((start_world_y - wy) / cell))
            return max(0, min(num_cols - 1, c)), max(0, min(num_rows - 1, r))

        def grid_to_world(c: int, r: int) -> Tuple[float, float]:
            wx = grid_min_x + c * cell
            wy = start_world_y - r * cell
            return wx, wy

        # Start and Goal
        start_col, start_row = world_to_grid(car_x, car_y)
        # Goal x prefers road centerline at lookahead distance
        goal_col, goal_row = world_to_grid(road_center_ahead, goal_world_y)
        goal_wx, goal_wy = grid_to_world(goal_col, goal_row)

        # 2. Pre-evaluate Obstacle Grid & Costs
        # Map dynamic traffic forward in time according to relative distance
        predicted_traffic = []
        effective_car_spd = max(120.0, car_speed)
        for t in traffic:
            # Estimate time when car reaches this traffic's Y
            delta_y = car_y - t.y
            if delta_y > -40:  # In front of or right next to car
                t_arrival = max(0.0, delta_y / effective_car_spd)
                pred_y = t.y - t.speed * t_arrival
                road_cx_pred = self.road.get_road_center(pred_y)
                pred_x = road_cx_pred + t.lateral_offset
                predicted_traffic.append((pred_x, pred_y, t))
            else:
                predicted_traffic.append((t.x, t.y, t))

        # Check cell traversability
        obstacle_cells = []
        
        def compute_cell_cost(c: int, r: int) -> float:
            wx, wy = grid_to_world(c, r)
            
            # Check road boundaries
            left_e, right_e, cx, rw = self.road.get_road_edges(wy)
            margin = 18.0
            if wx < left_e + margin or wx > right_e - margin:
                # Off asphalt or on dirt shoulder
                off = self.road.get_offroad_penalty(wx, wy)
                return 400.0 + off * 25.0

            # Distance from road center (soft preference to stay centered unless dodging)
            center_dist = abs(wx - cx) / (rw * 0.5)
            penalty = center_dist * 6.0

            # Check potholes
            for p in potholes:
                dx = wx - p.x
                dy = wy - p.y
                dist_sq = dx * dx + dy * dy
                safe_r = p.effective_radius + SAFETY_MARGIN_POTHOLE
                if dist_sq <= (safe_r * safe_r):
                    return float('inf') # Impassable
                elif dist_sq <= ((safe_r + 28) ** 2):
                    # Buffer repulsive cost
                    dist = math.sqrt(dist_sq)
                    penalty += (safe_r + 28 - dist) * 12.0

            # Check traffic vehicles (both actual & predicted)
            for pred_x, pred_y, t in predicted_traffic:
                # Check bounding distance
                dx = abs(wx - pred_x)
                dy = abs(wy - pred_y)
                safe_w = (t.width / 2.0) + SAFETY_MARGIN_CAR + 4
                safe_l = (t.length / 2.0) + SAFETY_MARGIN_CAR + 10
                if dx < safe_w and dy < safe_l:
                    return float('inf') # Collision box
                elif dx < safe_w + 24 and dy < safe_l + 32:
                    # Vehicle safety buffer
                    penalty += 75.0 + (safe_w + 24 - dx) * 5.0 + (safe_l + 32 - dy) * 4.0

            return penalty

        # 3. Initialize A* Data Structures
        nodes = {}
        def get_node(c: int, r: int) -> GridNode:
            key = (c, r)
            if key not in nodes:
                wx, wy = grid_to_world(c, r)
                nodes[key] = GridNode(c, r, wx, wy)
            return nodes[key]

        start_node = get_node(start_col, start_row)
        start_node.g = 0.0
        start_node.h = math.hypot(start_node.x - goal_wx, start_node.y - goal_wy)
        start_node.f = start_node.h

        open_set = []
        heapq.heappush(open_set, (start_node.f, 0, start_node))
        
        closed_set: Set[Tuple[int, int]] = set()
        explored_coords = []
        tie_breaker = 1

        best_node = start_node
        best_progress_r = start_row

        # Motion primitives for vehicle driving forward:
        # (delta_col, delta_row, base_cost_weight)
        # Moving up in grid (decreasing wy) corresponds to row + 1, + 2
        moves = [
            (0, 1, 1.0),      # Straight forward
            (-1, 1, 1.35),    # Slight left
            (1, 1, 1.35),     # Slight right
            (-2, 1, 2.1),     # Swerve left
            (2, 1, 2.1),      # Swerve right
            (-1, 2, 2.3),     # Steep forward left
            (1, 2, 2.3),      # Steep forward right
            (0, 2, 2.0),      # Faster forward leap
        ]

        # 4. Search Loop
        max_iterations = 650
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            _, _, current = heapq.heappop(open_set)
            
            coord = (current.col, current.row)
            if coord in closed_set:
                continue
            closed_set.add(coord)
            explored_coords.append((current.x, current.y))

            # Track progress towards goal row
            if current.row > best_progress_r:
                best_progress_r = current.row
                best_node = current

            # Termination condition: reached goal row or near horizon
            if current.row >= num_rows - 2 or (abs(current.row - goal_row) <= 1 and abs(current.col - goal_col) <= 2):
                best_node = current
                break

            # Explore neighbors
            for dc, dr, move_weight in moves:
                nc = current.col + dc
                nr = current.row + dr

                if not (0 <= nc < num_cols and 0 <= nr < num_rows):
                    continue
                if (nc, nr) in closed_set:
                    continue

                # Evaluate environment cost
                cell_penalty = compute_cell_cost(nc, nr)
                if math.isinf(cell_penalty):
                    obstacle_cells.append(grid_to_world(nc, nr))
                    continue

                neighbor = get_node(nc, nr)
                step_dist = math.hypot(dc * cell, dr * cell)
                tentative_g = current.g + step_dist * move_weight + cell_penalty

                # Steering smoothness penalty
                if current.parent:
                    prev_dc = current.col - current.parent.col
                    steering_diff = abs(dc - prev_dc)
                    tentative_g += steering_diff * 4.0

                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    # Heuristic: Euclidean distance to goal point
                    neighbor.h = math.hypot(neighbor.x - goal_wx, neighbor.y - goal_wy)
                    neighbor.f = neighbor.g + neighbor.h
                    tie_breaker += 1
                    heapq.heappush(open_set, (neighbor.f, tie_breaker, neighbor))

        self.nodes_explored_count = len(closed_set)
        self.last_explored_cells = explored_coords[::3] # downsample for debug rendering
        self.last_obstacle_cells = obstacle_cells[::2]

        # 5. Reconstruct Path
        raw_path = []
        curr = best_node
        while curr:
            raw_path.append((curr.x, curr.y))
            curr = curr.parent
        raw_path.reverse()

        # Fallback: if search couldn't progress, steer towards road center ahead
        if len(raw_path) <= 1:
            raw_path = [
                (car_x, car_y),
                (self.road.get_road_center(car_y - 120), car_y - 120),
                (road_center_ahead, goal_world_y)
            ]

        # 6. Path Smoothing (Chaikin / spline smoothing for fluid vehicle steering)
        smoothed = self._smooth_path(raw_path)
        self.last_path = smoothed
        return smoothed

    def _smooth_path(self, path: List[Tuple[float, float]], iterations: int = 2) -> List[Tuple[float, float]]:
        """Applies Chaikin's corner-cutting algorithm for butter-smooth steering curves."""
        if len(path) < 3:
            return path

        pts = list(path)
        for _ in range(iterations):
            new_pts = [pts[0]]
            for i in range(len(pts) - 1):
                p0 = pts[i]
                p1 = pts[i+1]
                q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
                r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
                new_pts.append(q)
                new_pts.append(r)
            new_pts.append(pts[-1])
            pts = new_pts

        return pts
