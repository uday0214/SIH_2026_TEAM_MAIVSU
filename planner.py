"""
Hierarchical Dual A* Pathfinding Planner on Frenet Road Lattices:
1. LongRangeAStarPlanner (Global / Strategic Corridor):
   Operates over a ~500px highway horizon, establishing long-range corridor direction,
   virtual lane discipline, road curvature tracing, and early bypass planning around
   distant traffic clusters and cow herds.
2. ShortRangeAStarPlanner (Local / Reactive Obstacle Navigator):
   Operates over a high-resolution ~185px horizon, actively negotiating immediate
   micro-obstacles: dynamic vehicle hitboxes & clearance zones, pothole obstacles at speed,
   pedestrians, and close-quarters maneuvers, anchored by the long-range path.
3. DualAStarPlanner (Unified Coordinator):
   Executes both algorithms synergistically, splicing the short-range reactive path
   onto the long-range strategic corridor for seamless vehicular guidance.
"""

import math
import heapq
from typing import List, Tuple, Optional, Set
from config import (
    PLANNER_CELL_SIZE, PLANNER_LOOKAHEAD_DIST,
    PLANNER_LONG_LOOKAHEAD, PLANNER_SHORT_LOOKAHEAD,
    SAFETY_MARGIN_CAR, SAFETY_MARGIN_POTHOLE, SAFETY_MARGIN_PEDESTRIAN,
    PLAYER_WIDTH, POTHOLE_OBSTACLE_SPEED_THRESHOLD
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


def _chaikin_smooth(path: List[Tuple[float, float]], iterations: int = 3) -> List[Tuple[float, float]]:
    """Applies Chaikin's corner-cutting algorithm for silky trajectories."""
    if len(path) < 3:
        return list(path)

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


class LongRangeAStarPlanner:
    """
    Global Strategic Planner:
    Plans ~500px down the highway along road curvature and virtual lanes.
    Identifies clear corridors well in advance, avoiding slow clusters and cross-median cuts.
    """
    def __init__(self, road):
        self.road = road
        self.step_s = 20.0   # Longitudinal resolution (px)
        self.step_d = 14.0   # Lateral resolution (px)
        self.cell_size = int(self.step_s)
        self.lookahead_dist = PLANNER_LONG_LOOKAHEAD
        
        self.last_path: List[Tuple[float, float]] = []
        self.last_explored_cells: List[Tuple[float, float]] = []
        self.last_obstacle_cells: List[Tuple[float, float]] = []
        self.nodes_explored_count = 0

    def plan_path(self, car_x: float, car_y: float, car_speed: float,
                   potholes: list, traffic: list, pedestrians: list = None,
                   cows: list = None) -> List[Tuple[float, float]]:
        if pedestrians is None:
            pedestrians = []
        if cows is None:
            cows = []

        step_s = self.step_s
        step_d = self.step_d
        H = self.lookahead_dist

        num_rows = int(H / step_s) + 1
        max_cols = 16

        start_world_y = car_y
        goal_row = num_rows - 1
        goal_world_y = start_world_y - goal_row * step_s
        goal_cx = self.road.get_road_center(goal_world_y)

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

        # Strategic Traffic Projection
        predicted_traffic = []
        effective_car_spd = max(110.0, car_speed)
        for t in traffic:
            delta_y = car_y - t.y
            if delta_y > -40:
                t_arrival = max(0.0, delta_y / effective_car_spd)
                pred_y = t.y - t.speed * t_arrival
                if hasattr(t, 'lane_idx') and hasattr(t, 'sub_lane_jitter'):
                    lanes_pred, _ = self.road.get_virtual_lanes(pred_y)
                    if 0 <= t.lane_idx < len(lanes_pred):
                        pred_x = lanes_pred[t.lane_idx] + t.sub_lane_jitter
                    else:
                        pred_road_cx = self.road.get_road_center(pred_y)
                        pred_x = pred_road_cx + getattr(t, 'lateral_offset', 0.0)
                else:
                    pred_road_cx = self.road.get_road_center(pred_y)
                    pred_x = pred_road_cx + getattr(t, 'lateral_offset', 0.0)
                predicted_traffic.append((pred_x, pred_y, t))
            else:
                predicted_traffic.append((t.x, t.y, t))

        # Strategic Cow / Herd Projection
        predicted_pedestrians = []
        for ped in pedestrians:
            delta_y = car_y - ped.y
            if delta_y > -30 and ped.state == 'CROSSING':
                t_arr = max(0.0, delta_y / effective_car_spd)
                pred_px = ped.x + ped.cross_speed_x * t_arr
                predicted_pedestrians.append((pred_px, ped.y, ped))
            else:
                predicted_pedestrians.append((ped.x, ped.y, ped))

        speed_pothole_factor = 1.40 if car_speed >= 200.0 else (0.90 if car_speed <= 80.0 else (0.90 + ((car_speed - 80.0)/120.0)*0.50))

        # Lane structure & corridor availability check
        car_lane_idx, car_lane_cx, car_lane_off = self.road.get_nearest_virtual_lane(car_x, car_y)
        car_side = self.road.get_road_side(car_x, car_y)
        car_half_w = PLAYER_WIDTH * 0.5

        room_available_ahead = True
        for pred_x, pred_y, t in predicted_traffic:
            dy = car_y - pred_y
            dx = abs(car_x - pred_x)
            vtype = getattr(t, 'vtype', 'CAR')
            margin_w = 20.0 if vtype in ('TRUCK', 'BUS') else (12.0 if vtype == 'CAR' else 7.0)
            if 0.0 < dy < 240.0 and dx < (car_half_w + t.width * 0.5 + margin_w):
                room_available_ahead = False
                break

        if room_available_ahead:
            for cow in cows:
                dy = car_y - cow.y
                dx = abs(car_x - cow.x)
                if 0.0 < dy < 220.0 and dx < (car_half_w + cow.width * 0.5 + 18.0):
                    room_available_ahead = False
                    break

        if room_available_ahead and car_speed > POTHOLE_OBSTACLE_SPEED_THRESHOLD:
            for p in potholes:
                dy = car_y - p.y
                dx = abs(car_x - p.x)
                if 0.0 < dy < 160.0 and dx < (p.effective_radius + car_half_w + 4.0):
                    room_available_ahead = False
                    break

        start_col, start_row = world_to_grid(car_x, car_y)
        goal_col = int(round(car_lane_off / step_d)) if room_available_ahead else 0
        goal_wx, goal_wy = grid_to_world(goal_col, goal_row)

        obstacle_cells = set()

        def compute_cell_cost(c: int, r: int) -> float:
            wx, wy = grid_to_world(c, r)

            left_e, right_e, cx, rw = self.road.get_road_edges(wy)
            margin = 17.0
            if wx < left_e + margin or wx > right_e - margin:
                off = self.road.get_offroad_penalty(wx, wy)
                return 600.0 + off * 35.0

            # Virtual Lane Guidance
            lanes, lane_w = self.road.get_virtual_lanes(wy)
            nearest_lane_idx, nearest_lane_cx, nearest_lane_off = self.road.get_nearest_virtual_lane(wx, wy)
            dist_to_lane_center = abs(wx - nearest_lane_cx)
            penalty = (dist_to_lane_center / (lane_w * 0.5)) * 6.0

            if room_available_ahead:
                cell_side = self.road.get_road_side(wx, wy)
                if car_side != 'CENTER' and cell_side != 'CENTER' and cell_side != car_side:
                    penalty += 480.0
                dist_from_our_lane = abs(wx - (cx + car_lane_off))
                if dist_from_our_lane > lane_w * 0.32:
                    penalty += 85.0 + (dist_from_our_lane - lane_w * 0.32) * 12.0

            # 1. Potholes (+50% increased penalty)
            for p in potholes:
                dx = wx - p.x
                dy = wy - p.y
                dist_sq = dx * dx + dy * dy
                safe_r = p.effective_radius + SAFETY_MARGIN_POTHOLE
                if car_speed > POTHOLE_OBSTACLE_SPEED_THRESHOLD:
                    if dist_sq <= (safe_r * safe_r):
                        return float('inf')
                    elif dist_sq <= ((safe_r + 24.0) ** 2):
                        dist = math.sqrt(dist_sq)
                        penalty += (safe_r + 24.0 - dist) * 21.0 * speed_pothole_factor
                else:
                    if dist_sq <= (safe_r * safe_r):
                        dist = math.sqrt(dist_sq)
                        penalty += (390.0 + (safe_r - dist) * 27.0) * speed_pothole_factor
                    elif dist_sq <= ((safe_r + 20.0) ** 2):
                        dist = math.sqrt(dist_sq)
                        penalty += (safe_r + 20.0 - dist) * 18.0 * speed_pothole_factor

            # 2. Dynamic Traffic
            for pred_x, pred_y, t in predicted_traffic:
                dx = abs(wx - pred_x)
                dy = abs(wy - pred_y)
                vtype = getattr(t, 'vtype', 'CAR')
                half_w = t.width * 0.5
                half_l = t.length * 0.5

                if vtype in ('TRUCK', 'BUS'):
                    margin_w = SAFETY_MARGIN_CAR + 4.0
                    margin_l = SAFETY_MARGIN_CAR + 8.0
                    caution_buf = 24.0
                elif vtype == 'CAR':
                    margin_w = 13.0
                    margin_l = 16.0
                    caution_buf = 18.0
                elif vtype == 'AUTO':
                    margin_w = 8.0
                    margin_l = 10.0
                    caution_buf = 14.0
                else:  # BIKE
                    margin_w = 5.0
                    margin_l = 7.0
                    caution_buf = 10.0

                impassable_w = half_w + margin_w
                impassable_l = half_l + margin_l

                if dx < impassable_w and dy < impassable_l:
                    return float('inf')
                elif dx < (impassable_w + caution_buf) and dy < (impassable_l + caution_buf + 6.0):
                    penalty += 70.0 + (impassable_w + caution_buf - dx) * 4.5 + (impassable_l + caution_buf + 6.0 - dy) * 4.0

            # 3. Pedestrians
            for pred_px, pred_py, ped in predicted_pedestrians:
                dx = wx - pred_px
                dy = wy - pred_py
                dist_sq = dx * dx + dy * dy
                safe_r = ped.radius + SAFETY_MARGIN_PEDESTRIAN + 6
                if dist_sq <= (safe_r * safe_r):
                    return float('inf')
                elif dist_sq <= ((safe_r + 28) ** 2):
                    dist = math.sqrt(dist_sq)
                    penalty += 110.0 + (safe_r + 28 - dist) * 9.0

            # 4. Cows
            for cow in cows:
                dx = wx - cow.x
                dy = wy - cow.y
                dist_sq = dx * dx + dy * dy
                if getattr(cow, 'is_resting', False):
                    safe_r = cow.radius + 22.0
                    if dist_sq <= (safe_r * safe_r):
                        return float('inf')
                    elif dist_sq <= ((safe_r + 48.0) ** 2):
                        dist = math.sqrt(dist_sq)
                        penalty += 170.0 + (safe_r + 48.0 - dist) * 13.0
                else:
                    safe_r = cow.radius + 18.0
                    if dist_sq <= (safe_r * safe_r):
                        return float('inf')
                    elif dist_sq <= ((safe_r + 32.0) ** 2):
                        dist = math.sqrt(dist_sq)
                        penalty += 100.0 + (safe_r + 32.0 - dist) * 8.0

            return penalty

        # A* Search
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
        closed_set = set()
        explored_coords = []
        tie_breaker = 0

        moves = [
            (0, 1, 1.0),
            (-1, 1, 1.35),
            (1, 1, 1.35),
            (-2, 1, 2.10),
            (2, 1, 2.10),
        ]

        best_node = start_node
        best_row = start_row

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if (current.col, current.row) in closed_set:
                continue
            closed_set.add((current.col, current.row))
            explored_coords.append((current.x, current.y))

            if current.row > best_row:
                best_row = current.row
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
                    obstacle_cells.add(grid_to_world(nc, nr))
                    continue

                neighbor = get_node(nc, nr)
                step_dist = math.hypot(dc * step_d, dr * step_s)
                curviness_penalty = 0.0
                if current.parent:
                    prev_dc = current.col - current.parent.col
                    curviness = abs(dc - prev_dc)
                    curviness_penalty += curviness * 28.0
                    if (prev_dc * dc) < 0:
                        curviness_penalty += 45.0
                    if current.parent.parent:
                        g_dc = current.parent.col - current.parent.parent.col
                        if (g_dc * prev_dc < 0) or (g_dc * dc > 0 and prev_dc * dc < 0):
                            curviness_penalty += 60.0

                curviness_penalty += abs(dc) * 4.0
                tentative_g = current.g + step_dist * move_weight + cell_penalty + curviness_penalty

                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = math.hypot(neighbor.x - goal_wx, neighbor.y - goal_wy)
                    if room_available_ahead:
                        neighbor.h += abs(nc - goal_col) * 5.0
                    neighbor.f = neighbor.g + neighbor.h
                    tie_breaker += 1
                    heapq.heappush(open_set, (neighbor.f, tie_breaker, neighbor))

        self.nodes_explored_count = len(closed_set)
        self.last_explored_cells = explored_coords[::3]
        self.last_obstacle_cells = list(obstacle_cells)

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

        smoothed = _chaikin_smooth(raw_path, iterations=3)
        self.last_path = smoothed
        return smoothed


class ShortRangeAStarPlanner:
    """
    Local Reactive Planner:
    Plans ~185px ahead at fine spatial resolution (12px longitudinal, 8.5px lateral).
    Directly navigates around immediate potholes, nearby vehicles, pedestrians, and tight gaps,
    anchored to and guided by the Long-Range strategic corridor path.
    """
    def __init__(self, road):
        self.road = road
        self.step_s = 12.0   # High-resolution longitudinal step (px)
        self.step_d = 8.5    # High-resolution lateral step (px)
        self.cell_size = int(self.step_s)
        self.lookahead_dist = PLANNER_SHORT_LOOKAHEAD

        self.last_path: List[Tuple[float, float]] = []
        self.last_explored_cells: List[Tuple[float, float]] = []
        self.last_obstacle_cells: List[Tuple[float, float]] = []
        self.nodes_explored_count = 0

    def plan_path(self, car_x: float, car_y: float, car_speed: float,
                   potholes: list, traffic: list, pedestrians: list = None,
                   cows: list = None,
                   long_range_ref: List[Tuple[float, float]] = None) -> List[Tuple[float, float]]:
        if pedestrians is None:
            pedestrians = []
        if cows is None:
            cows = []

        step_s = self.step_s
        step_d = self.step_d
        H = self.lookahead_dist

        num_rows = int(H / step_s) + 1
        max_cols = 18

        start_world_y = car_y
        goal_row = num_rows - 1
        goal_world_y = start_world_y - goal_row * step_s

        # Helper: query reference x from long-range path at world y
        def get_reference_x(wy: float) -> float:
            if not long_range_ref or len(long_range_ref) < 2:
                return self.road.get_road_center(wy)
            # Find bracket in long_range_ref
            if wy >= long_range_ref[0][1]:
                return long_range_ref[0][0]
            if wy <= long_range_ref[-1][1]:
                return long_range_ref[-1][0]
            for i in range(len(long_range_ref) - 1):
                p0 = long_range_ref[i]
                p1 = long_range_ref[i+1]
                if p0[1] >= wy >= p1[1]:
                    span = p0[1] - p1[1]
                    if span < 0.01:
                        return p0[0]
                    t = (p0[1] - wy) / span
                    return p0[0] + t * (p1[0] - p0[0])
            return self.road.get_road_center(wy)

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

        # Local dynamic traffic projection
        predicted_traffic = []
        effective_car_spd = max(100.0, car_speed)
        for t in traffic:
            delta_y = car_y - t.y
            if delta_y > -35:
                t_arrival = max(0.0, delta_y / effective_car_spd)
                pred_y = t.y - t.speed * t_arrival
                if hasattr(t, 'lane_idx') and hasattr(t, 'sub_lane_jitter'):
                    lanes_pred, _ = self.road.get_virtual_lanes(pred_y)
                    if 0 <= t.lane_idx < len(lanes_pred):
                        pred_x = lanes_pred[t.lane_idx] + t.sub_lane_jitter
                    else:
                        pred_road_cx = self.road.get_road_center(pred_y)
                        pred_x = pred_road_cx + getattr(t, 'lateral_offset', 0.0)
                else:
                    pred_road_cx = self.road.get_road_center(pred_y)
                    pred_x = pred_road_cx + getattr(t, 'lateral_offset', 0.0)
                predicted_traffic.append((pred_x, pred_y, t))
            else:
                predicted_traffic.append((t.x, t.y, t))

        predicted_pedestrians = []
        for ped in pedestrians:
            delta_y = car_y - ped.y
            if delta_y > -25 and ped.state == 'CROSSING':
                t_arr = max(0.0, delta_y / effective_car_spd)
                pred_px = ped.x + ped.cross_speed_x * t_arr
                predicted_pedestrians.append((pred_px, ped.y, ped))
            else:
                predicted_pedestrians.append((ped.x, ped.y, ped))

        speed_pothole_factor = 1.40 if car_speed >= 200.0 else (0.90 if car_speed <= 80.0 else (0.90 + ((car_speed - 80.0)/120.0)*0.50))

        start_col, start_row = world_to_grid(car_x, car_y)
        goal_target_x = get_reference_x(goal_world_y)
        goal_cx = self.road.get_road_center(goal_world_y)
        goal_col = int(round((goal_target_x - goal_cx) / step_d))
        goal_wx, goal_wy = grid_to_world(goal_col, goal_row)

        obstacle_cells = set()

        def compute_cell_cost(c: int, r: int) -> float:
            wx, wy = grid_to_world(c, r)

            left_e, right_e, cx, rw = self.road.get_road_edges(wy)
            margin = 16.0
            if wx < left_e + margin or wx > right_e - margin:
                off = self.road.get_offroad_penalty(wx, wy)
                return 650.0 + off * 40.0

            # Backbone Attraction to Long-Range Reference Path:
            # Keeps the local trajectory anchored to the strategic corridor when clear
            ref_x = get_reference_x(wy)
            dist_to_backbone = abs(wx - ref_x)
            penalty = (dist_to_backbone / 24.0) * 8.5

            # 1. Potholes: Pure Obstacles at speed > 6-7 km/h, 50% increased crawl penalty
            for p in potholes:
                dx = wx - p.x
                dy = wy - p.y
                dist_sq = dx * dx + dy * dy
                safe_r = p.effective_radius + SAFETY_MARGIN_POTHOLE
                if car_speed > POTHOLE_OBSTACLE_SPEED_THRESHOLD:
                    if dist_sq <= (safe_r * safe_r):
                        return float('inf')  # Impassable pure obstacle (reddish boxes)
                    elif dist_sq <= ((safe_r + 22.0) ** 2):
                        dist = math.sqrt(dist_sq)
                        penalty += (safe_r + 22.0 - dist) * 21.0 * speed_pothole_factor
                else:
                    if dist_sq <= (safe_r * safe_r):
                        dist = math.sqrt(dist_sq)
                        penalty += (390.0 + (safe_r - dist) * 27.0) * speed_pothole_factor
                    elif dist_sq <= ((safe_r + 20.0) ** 2):
                        dist = math.sqrt(dist_sq)
                        penalty += (safe_r + 20.0 - dist) * 18.0 * speed_pothole_factor

            # 2. Dynamic Traffic Vehicles (Pure Obstacle Hitbox + Dynamic Clearance Envelope)
            for pred_x, pred_y, t in predicted_traffic:
                dx = abs(wx - pred_x)
                dy = abs(wy - pred_y)
                vtype = getattr(t, 'vtype', 'CAR')
                half_w = t.width * 0.5
                half_l = t.length * 0.5

                if vtype in ('TRUCK', 'BUS'):
                    margin_w = SAFETY_MARGIN_CAR + 4.0   # 22 px
                    margin_l = SAFETY_MARGIN_CAR + 8.0   # 26 px
                    caution_buf = 22.0
                elif vtype == 'CAR':
                    margin_w = 13.0
                    margin_l = 16.0
                    caution_buf = 16.0
                elif vtype == 'AUTO':
                    margin_w = 8.0
                    margin_l = 10.0
                    caution_buf = 12.0
                else:  # BIKE
                    margin_w = 5.0
                    margin_l = 7.0
                    caution_buf = 8.0

                impassable_w = half_w + margin_w
                impassable_l = half_l + margin_l

                if dx < impassable_w and dy < impassable_l:
                    return float('inf')
                elif dx < (impassable_w + caution_buf) and dy < (impassable_l + caution_buf + 6.0):
                    penalty += 75.0 + (impassable_w + caution_buf - dx) * 4.5 + (impassable_l + caution_buf + 6.0 - dy) * 4.0

            # 3. Pedestrians
            for pred_px, pred_py, ped in predicted_pedestrians:
                dx = wx - pred_px
                dy = wy - pred_py
                dist_sq = dx * dx + dy * dy
                safe_r = ped.radius + SAFETY_MARGIN_PEDESTRIAN + 5.0
                if dist_sq <= (safe_r * safe_r):
                    return float('inf')
                elif dist_sq <= ((safe_r + 26) ** 2):
                    dist = math.sqrt(dist_sq)
                    penalty += 120.0 + (safe_r + 26 - dist) * 10.0

            # 4. Cows
            for cow in cows:
                dx = wx - cow.x
                dy = wy - cow.y
                dist_sq = dx * dx + dy * dy
                safe_r = cow.radius + 20.0
                if dist_sq <= (safe_r * safe_r):
                    return float('inf')
                elif dist_sq <= ((safe_r + 36.0) ** 2):
                    dist = math.sqrt(dist_sq)
                    penalty += 140.0 + (safe_r + 36.0 - dist) * 11.0

            return penalty

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
        closed_set = set()
        explored_coords = []
        tie_breaker = 0

        # High-resolution local lattice moves
        moves = [
            (0, 1, 1.0),
            (-1, 1, 1.25),
            (1, 1, 1.25),
            (-2, 1, 1.85),
            (2, 1, 1.85),
            (-3, 1, 2.70),
            (3, 1, 2.70),
        ]

        best_node = start_node
        best_row = start_row

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if (current.col, current.row) in closed_set:
                continue
            closed_set.add((current.col, current.row))
            explored_coords.append((current.x, current.y))

            if current.row > best_row:
                best_row = current.row
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
                    obstacle_cells.add(grid_to_world(nc, nr))
                    continue

                neighbor = get_node(nc, nr)
                step_dist = math.hypot(dc * step_d, dr * step_s)

                curviness_penalty = 0.0
                if current.parent:
                    prev_dc = current.col - current.parent.col
                    curviness = abs(dc - prev_dc)
                    curviness_penalty += curviness * 24.0
                    if (prev_dc * dc) < 0:
                        curviness_penalty += 40.0

                curviness_penalty += abs(dc) * 3.5
                tentative_g = current.g + step_dist * move_weight + cell_penalty + curviness_penalty

                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = math.hypot(neighbor.x - goal_wx, neighbor.y - goal_wy) + abs(nc - goal_col) * 3.5
                    neighbor.f = neighbor.g + neighbor.h
                    tie_breaker += 1
                    heapq.heappush(open_set, (neighbor.f, tie_breaker, neighbor))

        self.nodes_explored_count = len(closed_set)
        self.last_explored_cells = explored_coords[::2]
        self.last_obstacle_cells = list(obstacle_cells)

        raw_path = []
        curr = best_node
        while curr:
            raw_path.append((curr.x, curr.y))
            curr = curr.parent
        raw_path.reverse()

        if len(raw_path) <= 1:
            raw_path = [
                (car_x, car_y),
                (get_reference_x(car_y - 90.0), car_y - 90.0),
                (goal_wx, goal_wy)
            ]

        smoothed = _chaikin_smooth(raw_path, iterations=3)
        self.last_path = smoothed
        return smoothed


class DualAStarPlanner:
    """
    Unified Coordinator combining Long-Range (Global) and Short-Range (Local) A* Planners:
    1. Runs LongRangeAStarPlanner to determine the strategic macro highway corridor.
    2. Runs ShortRangeAStarPlanner anchored to the strategic path to execute micro-avoidance.
    3. Blends them into a continuous, silky trajectory.
    """
    def __init__(self, road):
        self.road = road
        self.long_range = LongRangeAStarPlanner(road)
        self.short_range = ShortRangeAStarPlanner(road)

        self.step_s = self.short_range.step_s
        self.step_d = self.short_range.step_d
        self.cell_size = self.short_range.cell_size
        self.lookahead_dist = self.long_range.lookahead_dist

        self.last_path: List[Tuple[float, float]] = []
        self.last_long_path: List[Tuple[float, float]] = []
        self.last_short_path: List[Tuple[float, float]] = []
        self.last_explored_cells: List[Tuple[float, float]] = []
        self.last_obstacle_cells: List[Tuple[float, float]] = []
        self.nodes_explored_count = 0

    def plan_path(self, car_x: float, car_y: float, car_speed: float,
                   potholes: list, traffic: list, pedestrians: list = None,
                   cows: list = None) -> List[Tuple[float, float]]:
        # 1. Global / Long-Range strategic trajectory
        long_path = self.long_range.plan_path(car_x, car_y, car_speed, potholes, traffic, pedestrians, cows)

        # 2. Local / Short-Range reactive trajectory anchored to long-range corridor
        short_path = self.short_range.plan_path(car_x, car_y, car_speed, potholes, traffic, pedestrians, cows, long_range_ref=long_path)

        # 3. Splice short-range reactive path smoothly onto long-range corridor
        splice_y = car_y - (self.short_range.lookahead_dist - 20.0)
        lead_in = [pt for pt in short_path if pt[1] >= splice_y]
        if not lead_in:
            lead_in = short_path

        continuation = [pt for pt in long_path if pt[1] < lead_in[-1][1] - 4.0]
        combined_raw = lead_in + continuation

        # Smooth transition at the splice joint
        final_path = _chaikin_smooth(combined_raw, iterations=2)

        # Update diagnostics & telemetry
        self.last_path = final_path
        self.last_long_path = long_path
        self.last_short_path = short_path
        self.last_explored_cells = self.short_range.last_explored_cells
        self.last_obstacle_cells = list(set(self.short_range.last_obstacle_cells + self.long_range.last_obstacle_cells))
        self.nodes_explored_count = self.short_range.nodes_explored_count + self.long_range.nodes_explored_count

        return final_path


# Maintain seamless backward compatibility with existing imports
AStarPlanner = DualAStarPlanner
