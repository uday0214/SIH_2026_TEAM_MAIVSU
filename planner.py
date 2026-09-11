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
        self.step_s = 16.0   # Longitudinal resolution along road (px)
        self.step_d = 13.0   # Lateral resolution across road (px)
        self.cell_size = int(self.step_s)
        self.lookahead_dist = PLANNER_LOOKAHEAD_DIST
        
        # Diagnostics
        self.last_path: List[Tuple[float, float]] = []
        self.last_explored_cells: List[Tuple[float, float]] = []
        self.last_obstacle_cells: List[Tuple[float, float]] = []
        self.nodes_explored_count = 0

    def plan_path(self, car_x: float, car_y: float, car_speed: float,
                  potholes: list, traffic: list, pedestrians: list = None,
                  cows: list = None) -> List[Tuple[float, float]]:
        """
        Runs road-aligned A* search (Frenet spatial lattice) from car's location
        up to the lookahead horizon. The path tracer strictly points in the direction
        of the road curvature ahead rather than straight up to the window top.
        """
        if pedestrians is None:
            pedestrians = []
        if cows is None:
            cows = []

        step_s = self.step_s
        step_d = self.step_d
        H = self.lookahead_dist
        
        num_rows = int(H / step_s) + 1
        max_cols = 16  # Lateral index: -16 to +16 covers up to ~416px dynamic road width

        start_world_y = car_y
        goal_row = num_rows - 1
        goal_world_y = start_world_y - goal_row * step_s
        goal_cx = self.road.get_road_center(goal_world_y)

        # Coordinate transforms between road-aligned lattice (c, r) and world (wx, wy):
        # Column c is lateral offset d = c * step_d from the road centerline cx(wy).
        # Moving forward with dc = 0 follows the EXACT road curvature down the highway!
        def grid_to_world(c: int, r: int) -> Tuple[float, float]:
            wy = start_world_y - r * step_s
            cx = self.road.get_road_center(wy)
            wx = cx + c * step_d
            return wx, wy

        def world_to_grid(wx: float, wy: float) -> Tuple[int, int]:
            r = int(round((start_world_y - wy) / step_s))
            cx = self.road.get_road_center(wy)
            c = int(round((wx - cx) / step_d))
            return max(-max_cols, min(max_cols, c)), max(0, min(num_rows - 1, r))

        start_col, start_row = world_to_grid(car_x, car_y)
        goal_col = 0  # Road centerline ahead
        goal_wx, goal_wy = grid_to_world(goal_col, goal_row)

        # Dynamic traffic projection: Reserve BOTH current and predicted future positions
        predicted_traffic = []
        effective_car_spd = max(110.0, car_speed)
        for t in traffic:
            delta_y = car_y - t.y
            if delta_y > -40:
                t_arrival = max(0.0, delta_y / effective_car_spd)
                pred_y = t.y - t.speed * t_arrival
                pred_road_cx = self.road.get_road_center(pred_y)
                pred_x = pred_road_cx + t.lateral_offset
                predicted_traffic.append((t.x, t.y, pred_x, pred_y, t))
            else:
                predicted_traffic.append((t.x, t.y, t.x, t.y, t))

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
            margin = 18.0
            if wx < left_e + margin or wx > right_e - margin:
                off = self.road.get_offroad_penalty(wx, wy)
                return 700.0 + off * 40.0

            # Virtual Lane Discipline (Models rough lane structure of Indian highways)
            # Three organic highway corridors: Left (-0.22 W), Center (0.0), Right (+0.22 W)
            d = c * step_d
            lane_l = -rw * 0.22
            lane_c = 0.0
            lane_r = rw * 0.22
            dist_to_lane = min(abs(d - lane_l), abs(d - lane_c), abs(d - lane_r))
            # Gentle lane discipline penalty encourages sticking to corridors
            penalty = dist_to_lane * 1.8 + (abs(d) / (rw * 0.5)) * 2.5

            # 1. Potholes (Strongly avoid by routing around; high finite penalty allows crawling if road blocked)
            for p in potholes:
                dx = wx - p.x
                dy = wy - p.y
                dist_sq = dx * dx + dy * dy
                safe_r = p.effective_radius + SAFETY_MARGIN_POTHOLE + 6.0
                if dist_sq <= (safe_r * safe_r):
                    dist = math.sqrt(dist_sq)
                    penalty += 1150.0 + (safe_r - dist) * 38.0
                elif dist_sq <= ((safe_r + 34.0) ** 2):
                    dist = math.sqrt(dist_sq)
                    penalty += (safe_r + 34.0 - dist) * 15.0

            # 2. Dynamic Traffic Vehicles (Never path into current OR future vehicle position!)
            for curr_x, curr_y, pred_x, pred_y, t in predicted_traffic:
                dx_c = abs(wx - curr_x)
                dy_c = abs(wy - curr_y)
                dx_p = abs(wx - pred_x)
                dy_p = abs(wy - pred_y)

                safe_w = (t.width / 2.0) + SAFETY_MARGIN_CAR + 6.0
                safe_l = (t.length / 2.0) + SAFETY_MARGIN_CAR + 14.0

                # Strict collision exclusion zone for both current and predicted positions
                if (dx_c < safe_w and dy_c < safe_l) or (dx_p < safe_w and dy_p < safe_l):
                    return float('inf')

                # Wide repulsive cushion to steer around well ahead of time
                cush_w = safe_w + 26.0
                cush_l = safe_l + 38.0
                if (dx_c < cush_w and dy_c < cush_l) or (dx_p < cush_w and dy_p < cush_l):
                    eff_dx = min(dx_c, dx_p)
                    eff_dy = min(dy_c, dy_p)
                    penalty += 160.0 + (cush_w - eff_dx) * 7.0 + (cush_l - eff_dy) * 5.0

            # 3. Pedestrians
            for pred_px, pred_py, ped in predicted_pedestrians:
                dx = wx - pred_px
                dy = wy - pred_py
                dist_sq = dx * dx + dy * dy
                safe_r = ped.radius + SAFETY_MARGIN_PEDESTRIAN + 8.0
                if dist_sq <= (safe_r * safe_r):
                    return float('inf')
                elif dist_sq <= ((safe_r + 30.0) ** 2):
                    dist = math.sqrt(dist_sq)
                    penalty += 130.0 + (safe_r + 30.0 - dist) * 10.0

            # 4. Indian Bovines (Cows): Predict and adjust path far ahead
            for cow in cows:
                dx = wx - cow.x
                dy = wy - cow.y
                dist_sq = dx * dx + dy * dy
                if getattr(cow, 'is_resting', False):
                    safe_r = cow.radius + 24.0
                    if dist_sq <= (safe_r * safe_r):
                        return float('inf')
                    elif dist_sq <= ((safe_r + 50.0) ** 2):
                        dist = math.sqrt(dist_sq)
                        penalty += 190.0 + (safe_r + 50.0 - dist) * 14.0
                else:
                    safe_r = cow.radius + 18.0
                    if dist_sq <= (safe_r * safe_r):
                        return float('inf')
                    elif dist_sq <= ((safe_r + 34.0) ** 2):
                        dist = math.sqrt(dist_sq)
                        penalty += 110.0 + (safe_r + 34.0 - dist) * 8.0

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

        # Smooth, Highway-Realistic Moves:
        # Avoids sharp cuts by requiring adequate longitudinal progress for lateral lane shifts.
        # No instantaneous (-2, 1) or (-1, 1) angular snaps!
        moves = [
            (0, 1, 1.0),      # Follow road corridor directly forward (most preferred)
            (0, 2, 1.6),      # Fast forward along road corridor
            (0, 3, 2.2),      # Long forward along road corridor
            (-1, 2, 2.2),     # Gentle, gradual left drift
            (1, 2, 2.2),      # Gentle, gradual right drift
            (-1, 3, 2.6),     # Smooth left lane change
            (1, 3, 2.6),      # Smooth right lane change
            (-2, 3, 3.8),     # Natural left bypass
            (2, 3, 3.8),      # Natural right bypass
            (-2, 4, 4.4),     # Wide gradual lane change left
            (2, 4, 4.4),      # Wide gradual lane change right
        ]

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

            if current.row > best_progress_r:
                best_progress_r = current.row
                best_node = current

            if current.row >= goal_row:
                best_node = current
                break

            for dc, dr, move_weight in moves:
                nc = current.col + dc
                nr = current.row + dr

                if not (-max_cols <= nc <= max_cols and 0 <= nr < num_rows):
                    continue
                if (nc, nr) in closed_set:
                    continue

                cell_penalty = compute_cell_cost(nc, nr)
                if math.isinf(cell_penalty):
                    obstacle_cells.append(grid_to_world(nc, nr))
                    continue

                neighbor = get_node(nc, nr)
                step_dist = math.hypot(dc * step_d, dr * step_s)
                tentative_g = current.g + step_dist * move_weight + cell_penalty

                # Steering smoothness constraint relative to road (suppresses zig-zag cuts):
                if current.parent:
                    prev_dc = current.col - current.parent.col
                    steering_diff = abs(dc - prev_dc)
                    tentative_g += steering_diff * 16.0

                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = math.hypot(neighbor.x - goal_wx, neighbor.y - goal_wy) + abs(nc) * 4.0
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
                (goal_cx, goal_world_y)
            ]

        smoothed = self._smooth_path(raw_path)
        self.last_path = smoothed
        return smoothed

    def _smooth_path(self, path: List[Tuple[float, float]], iterations: int = 3) -> List[Tuple[float, float]]:
        """
        Applies Chaikin's corner-cutting algorithm followed by moving-average
        curvature relaxation for silky, continuous highway trajectories.
        """
        if len(path) < 3:
            return path

        # 1. Chaikin corner cutting
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

        # 2. Moving average relaxation to eliminate any micro-inflections
        smoothed = [pts[0]]
        for i in range(1, len(pts) - 1):
            sx = 0.20 * pts[i-1][0] + 0.60 * pts[i][0] + 0.20 * pts[i+1][0]
            sy = 0.20 * pts[i-1][1] + 0.60 * pts[i][1] + 0.20 * pts[i+1][1]
            smoothed.append((sx, sy))
        smoothed.append(pts[-1])
        return smoothed
