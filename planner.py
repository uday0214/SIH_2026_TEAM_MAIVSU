"""
A* Pathfinding Planner on a moving local spatial lattice.
Directs the autonomous vehicle smoothly around potholes, traffic, and pedestrians
with strict curvature and steering constraints.
"""

import math
import heapq
from typing import List, Tuple, Optional, Set
from config import (
    PLANNER_CELL_SIZE, PLANNER_LOOKAHEAD_DIST,
    SAFETY_MARGIN_CAR, SAFETY_MARGIN_POTHOLE, SAFETY_MARGIN_PEDESTRIAN
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
                  potholes: list, traffic: list, pedestrians: list = None) -> List[Tuple[float, float]]:
        """
        Runs constrained A* search from car's current location up to lookahead horizon.
        Returns a smoothed list of world coordinates (x, y).
        """
        if pedestrians is None:
            pedestrians = []

        cell = self.cell_size
        H = self.lookahead_dist
        
        start_world_y = car_y
        goal_world_y = car_y - H
        
        road_center_ahead = self.road.get_road_center(goal_world_y)
        
        # Grid X bounds
        left_bound = min(self.road.get_road_edges(car_y)[0], self.road.get_road_edges(goal_world_y)[0]) - 40
        right_bound = max(self.road.get_road_edges(car_y)[1], self.road.get_road_edges(goal_world_y)[1]) + 40
        
        grid_min_x = math.floor(left_bound / cell) * cell
        grid_max_x = math.ceil(right_bound / cell) * cell
        
        num_cols = int((grid_max_x - grid_min_x) / cell) + 1
        num_rows = int(H / cell) + 2

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
        goal_col, goal_row = world_to_grid(road_center_ahead, goal_world_y)
        goal_wx, goal_wy = grid_to_world(goal_col, goal_row)

        # Dynamic traffic projection
        predicted_traffic = []
        effective_car_spd = max(110.0, car_speed)
        for t in traffic:
            delta_y = car_y - t.y
            if delta_y > -40:
                t_arrival = max(0.0, delta_y / effective_car_spd)
                pred_y = t.y - t.speed * t_arrival
                # Track predicted x based on road curve and vehicle lateral target
                pred_road_cx = self.road.get_road_center(pred_y)
                pred_x = pred_road_cx + t.lateral_offset
                predicted_traffic.append((pred_x, pred_y, t))
            else:
                predicted_traffic.append((t.x, t.y, t))

        # Pedestrian projection
        predicted_pedestrians = []
        for ped in pedestrians:
            delta_y = car_y - ped.y
            if delta_y > -30 and ped.state == 'CROSSING':
                t_arr = max(0.0, delta_y / effective_car_spd)
                pred_px = ped.x + ped.cross_speed_x * t_arr
                predicted_pedestrians.append((pred_px, ped.y, ped))
            else:
                predicted_pedestrians.append((ped.x, ped.y, ped))

        obstacle_cells = []

        def compute_cell_cost(c: int, r: int) -> float:
            wx, wy = grid_to_world(c, r)
            
            # Road boundaries constraint
            left_e, right_e, cx, rw = self.road.get_road_edges(wy)
            margin = 16.0
            if wx < left_e + margin or wx > right_e - margin:
                off = self.road.get_offroad_penalty(wx, wy)
                return 500.0 + off * 30.0

            # Mild centerline attraction (stabilizes steering down the road)
            center_dist = abs(wx - cx) / (rw * 0.5)
            penalty = center_dist * 4.0

            # 1. Potholes
            for p in potholes:
                dx = wx - p.x
                dy = wy - p.y
                dist_sq = dx * dx + dy * dy
                safe_r = p.effective_radius + SAFETY_MARGIN_POTHOLE
                if dist_sq <= (safe_r * safe_r):
                    return float('inf')
                elif dist_sq <= ((safe_r + 26) ** 2):
                    dist = math.sqrt(dist_sq)
                    penalty += (safe_r + 26 - dist) * 14.0

            # 2. Dynamic Traffic Vehicles
            for pred_x, pred_y, t in predicted_traffic:
                dx = abs(wx - pred_x)
                dy = abs(wy - pred_y)
                safe_w = (t.width / 2.0) + SAFETY_MARGIN_CAR + 4
                safe_l = (t.length / 2.0) + SAFETY_MARGIN_CAR + 8
                if dx < safe_w and dy < safe_l:
                    return float('inf')
                elif dx < safe_w + 24 and dy < safe_l + 30:
                    penalty += 80.0 + (safe_w + 24 - dx) * 5.0 + (safe_l + 30 - dy) * 4.0

            # 3. Pedestrians
            for pred_px, pred_py, ped in predicted_pedestrians:
                dx = wx - pred_px
                dy = wy - pred_py
                dist_sq = dx * dx + dy * dy
                safe_r = ped.radius + SAFETY_MARGIN_PEDESTRIAN + 6
                if dist_sq <= (safe_r * safe_r):
                    return float('inf') # Pedestrian collision box strictly forbidden
                elif dist_sq <= ((safe_r + 28) ** 2):
                    dist = math.sqrt(dist_sq)
                    penalty += 110.0 + (safe_r + 28 - dist) * 9.0

            return penalty

        # A* Graph Setup
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

        # Constrained forward driving moves:
        # Heavier penalty on sharp lateral steps prevents erratic zig-zagging
        moves = [
            (0, 1, 1.0),      # Straight forward (most preferred)
            (-1, 1, 1.55),    # Gentle left
            (1, 1, 1.55),     # Gentle right
            (0, 2, 2.0),      # Cruising forward
            (-1, 2, 2.2),     # Gradual left forward
            (1, 2, 2.2),      # Gradual right forward
            (-2, 1, 4.2),     # Sharp swerve left (penalized, used only if blocked)
            (2, 1, 4.2),      # Sharp swerve right (penalized, used only if blocked)
        ]

        max_iterations = 600
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            _, _, current = heapq.heappop(open_set)
            
            coord = (current.col, current.row)
            if coord in closed_set:
                continue
            closed_set.add(coord)
            explored_coords.append((current.x, current.y))

            if current.row > best_progress_r:
                best_progress_r = current.row
                best_node = current

            if current.row >= num_rows - 2:
                best_node = current
                break

            for dc, dr, move_weight in moves:
                nc = current.col + dc
                nr = current.row + dr

                if not (0 <= nc < num_cols and 0 <= nr < num_rows):
                    continue
                if (nc, nr) in closed_set:
                    continue

                cell_penalty = compute_cell_cost(nc, nr)
                if math.isinf(cell_penalty):
                    obstacle_cells.append(grid_to_world(nc, nr))
                    continue

                neighbor = get_node(nc, nr)
                step_dist = math.hypot(dc * cell, dr * cell)
                tentative_g = current.g + step_dist * move_weight + cell_penalty

                # Strict steering smoothness constraint:
                # Penalize rapid steering reversal or lateral direction flip
                if current.parent:
                    prev_dc = current.col - current.parent.col
                    steering_diff = abs(dc - prev_dc)
                    tentative_g += steering_diff * 12.0

                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = math.hypot(neighbor.x - goal_wx, neighbor.y - goal_wy)
                    neighbor.f = neighbor.g + neighbor.h
                    tie_breaker += 1
                    heapq.heappush(open_set, (neighbor.f, tie_breaker, neighbor))

        self.nodes_explored_count = len(closed_set)
        self.last_explored_cells = explored_coords[::3]
        self.last_obstacle_cells = obstacle_cells[::2]

        raw_path = []
        curr = best_node
        while curr:
            raw_path.append((curr.x, curr.y))
            curr = curr.parent
        raw_path.reverse()

        if len(raw_path) <= 1:
            raw_path = [
                (car_x, car_y),
                (self.road.get_road_center(car_y - 120), car_y - 120),
                (road_center_ahead, goal_world_y)
            ]

        smoothed = self._smooth_path(raw_path)
        self.last_path = smoothed
        return smoothed

    def _smooth_path(self, path: List[Tuple[float, float]], iterations: int = 3) -> List[Tuple[float, float]]:
        """Applies 3 iterations of Chaikin's corner-cutting algorithm for silky trajectories."""
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
