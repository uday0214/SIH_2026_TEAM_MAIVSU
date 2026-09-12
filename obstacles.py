"""
Obstacles module: Dynamic Potholes, Collision-Free Traffic (Truck, Bus, Car, Auto, Bike with IDM-like
following and complete stop-at-rest support), and Jaywalking Pedestrians.
"""

import math
import random
import pygame
from typing import Tuple, Optional, List
from config import (
    COLOR_POTHOLE_INNER, COLOR_POTHOLE_RIM,
    SAFETY_MARGIN_CAR, SAFETY_MARGIN_POTHOLE, SAFETY_MARGIN_PEDESTRIAN,
    REST_ACCEL_FACTOR
)

class Pothole:
    def __init__(self, x: float, y: float, rx: float, ry: float):
        self.x = x
        self.y = y
        self.rx = rx  # radius x
        self.ry = ry  # radius y
        self.effective_radius = max(rx, ry)
        self.rim_points = []
        num_pts = 11
        for i in range(num_pts):
            ang = (i / num_pts) * 2 * math.pi
            r_jitter = random.uniform(0.82, 1.18)
            px = math.cos(ang) * rx * r_jitter
            py = math.sin(ang) * ry * r_jitter
            self.rim_points.append((px, py))
        self.hit = False

    def get_bounding_radius(self) -> float:
        return self.effective_radius + SAFETY_MARGIN_POTHOLE

    def contains_point(self, px: float, py: float, margin: float = 0.0) -> bool:
        dx = (px - self.x) / (self.rx + margin)
        dy = (py - self.y) / (self.ry + margin)
        return (dx * dx + dy * dy) <= 1.0

    def draw(self, surface: pygame.Surface, camera_y: float):
        sy = self.y - camera_y
        h = surface.get_height()
        if sy < -60 or sy > h + 60:
            return

        screen_pts = [(int(self.x + px), int(sy + py)) for px, py in self.rim_points]
        if len(screen_pts) >= 3:
            pygame.draw.polygon(surface, COLOR_POTHOLE_RIM, screen_pts)

        inner_rect = pygame.Rect(
            int(self.x - self.rx * 0.75),
            int(sy - self.ry * 0.75),
            int(self.rx * 1.5),
            int(self.ry * 1.5)
        )
        pygame.draw.ellipse(surface, COLOR_POTHOLE_INNER, inner_rect)
        pygame.draw.ellipse(surface, (15, 12, 10), inner_rect, 2)


class Pedestrian:
    """
    Pedestrian walking along roadside shoulder who may unpredictably
    cross the road without warning.
    """
    SHIRT_COLORS = [
        (230, 80, 40),   # Saffron
        (240, 240, 245), # White kurta
        (30, 140, 210),  # Blue
        (220, 190, 40),  # Yellow
        (160, 40, 140),  # Magenta
        (40, 160, 80)    # Green
    ]

    def __init__(self, x: float, y: float, side: int):
        self.x = x
        self.y = y
        self.side = side
        self.radius = 8.0
        self.hit = False

        self.state = 'WALKING_SHOULDER'
        self.shirt_color = random.choice(self.SHIRT_COLORS)
        self.skin_color = (195, 145, 105)
        self.hair_color = (25, 20, 20)

        self.walk_speed_y = random.uniform(18.0, 30.0) * random.choice([-1, 1])
        self.cross_speed_x = random.uniform(32.0, 46.0) * (-self.side)
        
        self.cross_timer = random.uniform(1.5, 5.5)
        self.walk_phase = random.uniform(0.0, 6.28)

        # Deadlock resolution state:
        self.is_standing = False
        self.is_moving = True
        self.stand_timer = 0.0
        self.has_right_of_way = False

    def update(self, dt: float, road, traffic=None, player_car=None):
        left, right, cx, rw = road.get_road_edges(self.y)

        vehicles_to_check = []
        if traffic:
            vehicles_to_check.extend(traffic)
        if player_car:
            vehicles_to_check.append(player_car)

        # 1. Threat & Proximity Assessment
        approaching_fast_threat = False
        yielding_vehicle_present = False
        closest_yielding_v = None
        scramble_dx = 0.0

        for v in vehicles_to_check:
            dy = v.y - self.y # positive if vehicle is south approaching north
            dx = abs(v.x - self.x)
            if 0 < dy < 110 and dx < (v.width * 0.5 + 26.0):
                if v.speed > 22.0:
                    approaching_fast_threat = True
                    if dy < 45.0 and dx < (v.width * 0.5 + 16.0):
                        dist_to_left = abs(self.x - left)
                        dist_to_right = abs(self.x - right)
                        step_dir = -1.0 if dist_to_left < dist_to_right else 1.0
                        scramble_dx = step_dir * 75.0
                    break
                else:
                    # Vehicle is stopping / yielding!
                    yielding_vehicle_present = True
                    closest_yielding_v = v

        if scramble_dx != 0.0:
            self.x += scramble_dx * dt
            self.walk_phase += dt * 12.0
            self.is_moving = True
            self.is_standing = False
            return

        # 2. Deadlock Resolution Protocol (Biased toward pedestrian moving first)
        if self.state == 'CROSSING':
            if approaching_fast_threat and not self.has_right_of_way:
                self.is_standing = True
                self.is_moving = False
                self.stand_timer += dt
                return

            if yielding_vehicle_present or self.stand_timer > 0.45:
                # Vehicle yielded or stopped: take right of way and cross first!
                self.has_right_of_way = True
                self.is_standing = False
                self.is_moving = True
                self.stand_timer = 0.0
                
                if closest_yielding_v and abs(self.x - closest_yielding_v.x) > (closest_yielding_v.width * 0.5 + 24.0):
                    self.has_right_of_way = False
        else:
            self.is_standing = False
            self.is_moving = True
            self.stand_timer = 0.0

        # 3. Walking progression (brisk crossing pace when taking right-of-way)
        pace_multiplier = 1.35 if self.has_right_of_way else 1.0
        self.walk_phase += dt * 8.0 * pace_multiplier

        if self.state == 'WALKING_SHOULDER':
            self.cross_timer -= dt
            target_edge = (left - 14) if self.side == -1 else (right + 14)
            self.x += (target_edge - self.x) * min(1.0, 4.0 * dt)
            self.y += self.walk_speed_y * dt

            if self.cross_timer <= 0:
                self.state = 'CROSSING'

        elif self.state == 'CROSSING':
            self.x += self.cross_speed_x * pace_multiplier * dt
            self.y += (self.walk_speed_y * 0.35) * dt

            if self.side == -1 and self.x > right + 18:
                self.side = 1
                self.state = 'WALKING_SHOULDER'
                self.has_right_of_way = False
                self.cross_timer = random.uniform(6.0, 12.0)
            elif self.side == 1 and self.x < left - 18:
                self.side = -1
                self.state = 'WALKING_SHOULDER'
                self.has_right_of_way = False
                self.cross_timer = random.uniform(6.0, 12.0)

    def get_bounding_radius(self) -> float:
        return self.radius + SAFETY_MARGIN_PEDESTRIAN

    def contains_point(self, px: float, py: float, margin: float = 0.0) -> bool:
        dx = px - self.x
        dy = py - self.y
        r = self.radius + margin
        return (dx * dx + dy * dy) <= (r * r)

    def draw(self, surface: pygame.Surface, camera_y: float):
        sy = self.y - camera_y
        h = surface.get_height()
        if sy < -30 or sy > h + 30:
            return

        ix = int(self.x)
        isy = int(sy)

        leg_offset = math.sin(self.walk_phase) * 4.0
        pygame.draw.circle(surface, (40, 40, 45), (int(ix + leg_offset), isy + 4), 3)
        pygame.draw.circle(surface, (40, 40, 45), (int(ix - leg_offset), isy - 4), 3)

        torso_rect = pygame.Rect(ix - 6, isy - 4, 12, 8)
        pygame.draw.ellipse(surface, self.shirt_color, torso_rect)
        pygame.draw.ellipse(surface, (20, 20, 20), torso_rect, 1)

        pygame.draw.circle(surface, self.skin_color, (ix, isy), 4)
        pygame.draw.circle(surface, self.hair_color, (ix, isy - 1), 3)


class Cow:
    """
    Indian Highway Bovine (Sacred Cow / Zebu / Desi Cattle).
    Moves in herds of max 4 cows along road shoulders/edges,
    or calmly rests in the middle of the road.
    """
    BREEDS = [
        {
            'name': 'WHITE_ZEBU',
            'body': (246, 243, 237),
            'shade': (212, 207, 198),
            'muzzle': (235, 192, 195),
            'horns': (120, 115, 110),
            'patches': None
        },
        {
            'name': 'BROWN_DESI',
            'body': (175, 120, 75),
            'shade': (135, 85, 50),
            'muzzle': (105, 70, 45),
            'horns': (65, 60, 55),
            'patches': None
        },
        {
            'name': 'SPOTTED',
            'body': (244, 244, 246),
            'shade': (205, 205, 210),
            'muzzle': (230, 185, 185),
            'horns': (90, 85, 80),
            'patches': [(42, 42, 45), (55, 52, 50)]
        },
        {
            'name': 'GREY_GYR',
            'body': (188, 190, 194),
            'shade': (145, 148, 154),
            'muzzle': (130, 130, 135),
            'horns': (60, 55, 55),
            'patches': None
        }
    ]

    def __init__(self, x: float, y: float, state: str = 'WALKING_EDGE', side: int = -1, herd_id: int = 0):
        self.x = x
        self.y = y
        self.state = state  # 'WALKING_EDGE', 'RESTING', or 'CLEARING_ROAD'
        self.is_resting = (state == 'RESTING')
        self.is_moving = not self.is_resting
        self.side = side  # -1: left edge, +1: right edge, 0: middle
        self.herd_id = herd_id

        self.breed = random.choice(self.BREEDS)
        self.width = 22.0
        self.length = 40.0
        self.radius = 17.0
        self.hit = False

        if self.is_resting:
            self.speed = 0.0
            self.angle_deg = random.uniform(-25.0, 25.0)
        else:
            self.speed = random.uniform(10.0, 16.0) * random.choice([-1, 1])
            self.angle_deg = 0.0 if self.speed < 0 else 180.0

        self.tail_phase = random.uniform(0.0, 6.28)
        self.chew_phase = random.uniform(0.0, 6.28)
        self.walk_phase = random.uniform(0.0, 6.28)

        # Motivation by horn & deadlock
        self.is_motivated_to_clear = False
        self.clearing_dir = 1  # -1 left, +1 right
        self.deadlock_timer = 0.0
        self.startled_timer = 0.0
        self.horn_reaction_anim = 0.0

    def trigger_motivation(self, road):
        """Called when a horn is sounded or a vehicle deadlock forms ahead of the cow."""
        if not self.is_motivated_to_clear:
            self.is_motivated_to_clear = True
            self.startled_timer = 7.0
            self.horn_reaction_anim = 2.0
            
            left, right, cx, _ = road.get_road_edges(self.y)
            # Pick nearest shoulder
            self.clearing_dir = -1 if self.x < cx else 1
            
            if self.is_resting:
                self.is_resting = False
                self.is_moving = True
                self.state = 'CLEARING_ROAD'

    def update(self, dt: float, road, traffic=None, player_car=None):
        self.tail_phase += dt * 3.2
        self.chew_phase += dt * 3.8

        if self.horn_reaction_anim > 0:
            self.horn_reaction_anim = max(0.0, self.horn_reaction_anim - dt)
        if self.startled_timer > 0:
            self.startled_timer = max(0.0, self.startled_timer - dt)

        # 1. Perception: Horn Hearing
        horn_heard = False
        if player_car and getattr(player_car, 'honk_timer', 0.0) > 0.0:
            dist_p = math.hypot(player_car.x - self.x, player_car.y - self.y)
            if dist_p < 210.0:
                horn_heard = True

        if not horn_heard and traffic:
            for veh in traffic:
                if getattr(veh, 'is_honking', False):
                    dist_t = math.hypot(veh.x - self.x, veh.y - self.y)
                    if dist_t < 190.0:
                        horn_heard = True
                        break

        if horn_heard:
            self.trigger_motivation(road)

        # 2. Perception: Deadlock Detection
        # Detect if any vehicle is blocked and stopped in front of the cow
        blocked_vehicle_present = False
        vehicles_checking = []
        if player_car:
            vehicles_checking.append(player_car)
        if traffic:
            vehicles_checking.extend(traffic)

        for v in vehicles_checking:
            dy = v.y - self.y  # positive if vehicle is south (approaching)
            dx = abs(v.x - self.x)
            if 0 < dy < 95.0 and dx < 42.0 and v.speed < 12.0:
                blocked_vehicle_present = True
                break

        if blocked_vehicle_present:
            self.deadlock_timer += dt
            if self.deadlock_timer > 0.65:
                # Deadlock detected! Motivate cow to stand up and walk out of the way
                self.trigger_motivation(road)
        else:
            self.deadlock_timer = max(0.0, self.deadlock_timer - dt * 0.4)

        # 3. Execution: Moving out of the way
        if self.is_motivated_to_clear:
            left, right, cx, rw = road.get_road_edges(self.y)
            self.walk_phase += dt * 4.2
            
            # Move actively toward the chosen shoulder
            self.x += self.clearing_dir * 28.0 * dt
            self.y -= 12.0 * dt  # also slow amble forward
            
            road_ang = math.degrees(road.get_tangent_angle(self.y))
            self.angle_deg = road_ang + self.clearing_dir * 38.0

            # Check if reached safe road shoulder
            if (self.clearing_dir == -1 and self.x <= left + 8.0) or \
               (self.clearing_dir == 1 and self.x >= right - 8.0):
                self.is_motivated_to_clear = False
                self.state = 'WALKING_EDGE'
                self.side = self.clearing_dir
                self.speed = random.uniform(9.0, 15.0) * (-1 if self.clearing_dir == -1 else 1)

        elif self.is_moving:
            self.walk_phase += dt * 2.8
            self.y += self.speed * dt

            left, right, cx, rw = road.get_road_edges(self.y)
            if self.side == -1:
                min_x = left - 24.0
                max_x = left + 22.0
            elif self.side == 1:
                min_x = right - 22.0
                max_x = right + 24.0
            else:
                min_x = cx - 35.0
                max_x = cx + 35.0

            self.x = max(min_x, min(max_x, self.x + math.sin(self.walk_phase * 0.4) * 3.5 * dt))

            road_ang = math.degrees(road.get_tangent_angle(self.y))
            base_ang = road_ang if self.speed < 0 else (road_ang + 180.0)
            self.angle_deg = base_ang + math.sin(self.walk_phase) * 5.0
        else:
            road_ang = math.degrees(road.get_tangent_angle(self.y))
            self.angle_deg = road_ang + math.sin(self.chew_phase * 0.5) * 3.5

    def draw(self, surface: pygame.Surface, camera_y: float):
        sy = self.y - camera_y
        h = surface.get_height()
        if sy < -70 or sy > h + 70:
            return

        sw, sh = 68, 68
        cow_surf = pygame.Surface((sw, sh), pygame.SRCALPHA)
        cx, cy = sw // 2, sh // 2

        body_col = self.breed['body']
        shade_col = self.breed['shade']
        muzzle_col = self.breed['muzzle']
        horn_col = self.breed['horns']

        # Ground contact shadow
        shadow_rect = pygame.Rect(cx - 10, cy - 18, 20, 36)
        pygame.draw.ellipse(cow_surf, (20, 24, 20, 95), shadow_rect)

        if self.is_resting:
            # RESTING COW (Sitting comfortably, chewing cud)
            torso_rect = pygame.Rect(cx - 11, cy - 17, 22, 34)
            pygame.draw.ellipse(cow_surf, body_col, torso_rect)
            pygame.draw.ellipse(cow_surf, shade_col, torso_rect, width=2)

            if self.breed['patches']:
                pcol = self.breed['patches'][0]
                pygame.draw.ellipse(cow_surf, pcol, (cx - 8, cy - 10, 10, 14))
                pygame.draw.ellipse(cow_surf, pcol, (cx + 1, cy + 1, 8, 11))

            # Dorsal hump
            hump_rect = pygame.Rect(cx - 5, cy - 14, 10, 8)
            pygame.draw.ellipse(cow_surf, shade_col, hump_rect)

            # Head
            head_rect = pygame.Rect(cx - 6, cy - 23, 12, 11)
            pygame.draw.ellipse(cow_surf, body_col, head_rect)
            pygame.draw.ellipse(cow_surf, muzzle_col, (cx - 4, cy - 25, 8, 5))
            pygame.draw.circle(cow_surf, (40, 30, 30), (cx - 2, cy - 24), 1)
            pygame.draw.circle(cow_surf, (40, 30, 30), (cx + 2, cy - 24), 1)

            # Curved horns
            pygame.draw.arc(cow_surf, horn_col, (cx - 11, cy - 25, 9, 10), 0.5, 2.7, 2)
            pygame.draw.arc(cow_surf, horn_col, (cx + 2, cy - 25, 9, 10), 0.4, 2.6, 2)

            # Ears
            pygame.draw.line(cow_surf, body_col, (cx - 6, cy - 20), (cx - 11, cy - 19), 2)
            pygame.draw.line(cow_surf, body_col, (cx + 6, cy - 20), (cx + 11, cy - 19), 2)

            # Swishing tail curled beside
            tail_swish = math.sin(self.tail_phase) * 3.0
            pygame.draw.line(cow_surf, shade_col, (cx, cy + 16), (cx + 4 + tail_swish, cy + 19), 2)
            pygame.draw.circle(cow_surf, (35, 30, 25), (int(cx + 4 + tail_swish), cy + 20), 2)
        else:
            # WALKING OR CLEARING COW
            leg_off1 = math.sin(self.walk_phase) * 3.5
            leg_off2 = -leg_off1
            pygame.draw.circle(cow_surf, shade_col, (cx - 9, int(cy - 12 + leg_off1)), 3)
            pygame.draw.circle(cow_surf, shade_col, (cx + 9, int(cy - 12 + leg_off2)), 3)
            pygame.draw.circle(cow_surf, shade_col, (cx - 9, int(cy + 12 + leg_off2)), 3)
            pygame.draw.circle(cow_surf, shade_col, (cx + 9, int(cy + 12 + leg_off1)), 3)

            torso_rect = pygame.Rect(cx - 10, cy - 18, 20, 36)
            pygame.draw.ellipse(cow_surf, body_col, torso_rect)
            pygame.draw.ellipse(cow_surf, shade_col, torso_rect, width=2)

            if self.breed['patches']:
                pcol = self.breed['patches'][0]
                pygame.draw.ellipse(cow_surf, pcol, (cx - 6, cy - 8, 9, 13))
                pygame.draw.ellipse(cow_surf, pcol, (cx + 1, cy + 2, 7, 10))

            # Prominent Indian Zebu hump
            pygame.draw.ellipse(cow_surf, shade_col, (cx - 6, cy - 15, 12, 9))
            pygame.draw.ellipse(cow_surf, body_col, (cx - 5, cy - 14, 10, 7))

            # Neck & Head
            head_rect = pygame.Rect(cx - 6, cy - 25, 12, 12)
            pygame.draw.ellipse(cow_surf, body_col, head_rect)
            pygame.draw.ellipse(cow_surf, muzzle_col, (cx - 4, cy - 28, 8, 6))
            pygame.draw.circle(cow_surf, (40, 30, 30), (cx - 2, cy - 27), 1)
            pygame.draw.circle(cow_surf, (40, 30, 30), (cx + 2, cy - 27), 1)

            # Horns
            pygame.draw.line(cow_surf, horn_col, (cx - 5, cy - 24), (cx - 11, cy - 27), 2)
            pygame.draw.line(cow_surf, horn_col, (cx - 11, cy - 27), (cx - 9, cy - 30), 2)
            pygame.draw.line(cow_surf, horn_col, (cx + 5, cy - 24), (cx + 11, cy - 27), 2)
            pygame.draw.line(cow_surf, horn_col, (cx + 11, cy - 27), (cx + 9, cy - 30), 2)

            # Floppy ears (perked up slightly when motivated)
            ear_y_off = -2 if self.is_motivated_to_clear else 0
            pygame.draw.ellipse(cow_surf, body_col, (cx - 12, cy - 22 + ear_y_off, 6, 4))
            pygame.draw.ellipse(cow_surf, body_col, (cx + 6, cy - 22 + ear_y_off, 6, 4))

            # Tail
            swish = math.sin(self.tail_phase) * 4.5
            pygame.draw.line(cow_surf, shade_col, (cx, cy + 18), (cx + swish, cy + 26), 2)
            pygame.draw.circle(cow_surf, (30, 25, 20), (int(cx + swish), cy + 27), 2)

        rot_surf = pygame.transform.rotate(cow_surf, -self.angle_deg)
        rot_rect = rot_surf.get_rect(center=(int(self.x), int(sy)))
        surface.blit(rot_surf, rot_rect)

        # Draw alert indicator if startled by horn or clearing road
        if self.horn_reaction_anim > 0:
            alert_y = int(sy - 34)
            pygame.draw.circle(surface, (255, 215, 30), (int(self.x), alert_y), 7)
            pygame.draw.circle(surface, (20, 20, 20), (int(self.x), alert_y), 7, width=1)
            pygame.draw.line(surface, (20, 20, 20), (int(self.x), alert_y - 4), (int(self.x), alert_y + 1), 2)
            pygame.draw.circle(surface, (20, 20, 20), (int(self.x), alert_y + 4), 1)


VEHICLE_SIZE_RANK = {
    'BIKE': 1,
    'AUTO': 2,
    'CAR': 3,
    'PLAYER': 3,
    'TRUCK': 4,
    'BUS': 5
}

def negotiate_deadlock_priority(v_self, v_other, road) -> Tuple[bool, str]:
    """
    Evaluates deadlock right-of-way between two vehicles using principled rules:
    1. Size hierarchy: Smaller, more agile vehicles move first (Bike < Auto < Car < Truck < Bus).
    2. Future trajectory collision projection: Check future points along headings.
    3. Available Escape Space: The vehicle with more lateral and forward maneuvering room moves first.
    4. Deterministic spatial tie-breaker: Avoids symmetric lock.
    """
    type_self = getattr(v_self, 'vtype', 'CAR')
    type_other = getattr(v_other, 'vtype', 'CAR')
    rank_self = VEHICLE_SIZE_RANK.get(type_self, 3)
    rank_other = VEHICLE_SIZE_RANK.get(type_other, 3)

    # Principle a: Smaller vehicle moves first
    if rank_self < rank_other:
        return True, f"Smaller vehicle priority ({type_self} < {type_other})"
    elif rank_self > rank_other:
        return False, f"Yielding to smaller vehicle ({type_other} < {type_self})"

    # Principle b: Check trajectory collision & Available Escape Space
    h_self = getattr(v_self, 'heading', 0.0)
    h_other = getattr(v_other, 'heading', 0.0)

    # Direction vectors
    d_self_x = math.sin(h_self)
    d_self_y = -math.cos(h_self)
    d_other_x = math.sin(h_other)
    d_other_y = -math.cos(h_other)

    # 1.2s future lookahead position
    dt_pred = 1.2
    fut_self_x = v_self.x + d_self_x * 24.0 * dt_pred
    fut_self_y = v_self.y + d_self_y * 24.0 * dt_pred
    fut_other_x = v_other.x + d_other_x * 24.0 * dt_pred
    fut_other_y = v_other.y + d_other_y * 24.0 * dt_pred

    current_dist = math.hypot(v_self.x - v_other.x, v_self.y - v_other.y)
    future_dist = math.hypot(fut_self_x - fut_other_x, fut_self_y - fut_other_y)

    # Calculate available maneuvering space for both vehicles
    left_self, right_self, _, _ = road.get_road_edges(v_self.y)
    left_other, right_other, _, _ = road.get_road_edges(v_other.y)

    # Free lateral escape space away from the other vehicle
    if v_other.x >= v_self.x:
        lat_space_self = max(0.0, v_self.x - left_self)
        lat_space_other = max(0.0, right_other - v_other.x)
    else:
        lat_space_self = max(0.0, right_self - v_self.x)
        lat_space_other = max(0.0, v_other.x - left_other)

    # Total clearance score
    total_space_self = lat_space_self + (v_other.y - v_self.y if v_self.y < v_other.y else 0.0)
    total_space_other = lat_space_other + (v_self.y - v_other.y if v_other.y < v_self.y else 0.0)

    if abs(total_space_self - total_space_other) > 12.0:
        if total_space_self > total_space_other:
            return True, "More available escape corridor space ahead"
        else:
            return False, "Holding position; other vehicle has more open space to maneuver"

    # Principle d: Deterministic spatial tie-breaker (vehicle further downroad moves first)
    if v_self.y < v_other.y:
        return True, "Downroad position priority"
    return False, "Holding for leading downroad vehicle"


class TrafficVehicle:
    """
    Dynamic traffic vehicle with realistic vehicular kinematics:
    1. Collision avoidance with other traffic, pedestrians, and main car.
    2. Ability to come to a complete stop (speed = 0.0) and restart smoothly.
    3. Strict road boundary clamping: NEVER goes offroad!
    4. Smooth kinematic lane cuts without teleporting.
    """
    TYPES = ['TRUCK', 'BUS', 'CAR', 'AUTO', 'BIKE']

    def __init__(self, x: float, y: float, vtype: str, speed: float, lateral_offset: float = 0.0,
                 lane_idx: int = 0, sub_lane_jitter: float = 0.0):
        self.x = x
        self.y = y
        self.vtype = vtype
        self.cruising_speed = speed
        self.speed = speed
        self.lateral_offset = lateral_offset
        self.lane_idx = lane_idx
        self.sub_lane_jitter = sub_lane_jitter
        self.heading = 0.0
        self.angle_deg = 0.0
        self.hit = False
        self.target_x = x
        self.is_honking = False
        self.honk_timer = 0.0

        # Kinetic lateral speed
        self.vx = 0.0
        self.accel = 130.0
        self.decel = 320.0
        self.is_braking = False

        if vtype == 'TRUCK':
            self.width = 34
            self.length = 78
            self.color_body = (210, 75, 20)     # Orange
            self.color_top = (35, 105, 175)     # Blue cargo
            self.max_lat_spd = 22.0
            self.lat_accel = 1.2
            self.cut_interval = (9.0, 18.0)
            self.cut_prob = 0.20
        elif vtype == 'BUS':
            self.width = 32
            self.length = 88
            self.color_body = (175, 32, 38)     # Maroon/red
            self.color_top = (235, 230, 210)    # Cream roof
            self.max_lat_spd = 20.0
            self.lat_accel = 1.0
            self.cut_interval = (10.0, 19.0)
            self.cut_prob = 0.18
        elif vtype == 'AUTO':
            self.width = 22
            self.length = 36
            self.color_body = (34, 139, 34)    # Green
            self.color_top = (245, 195, 35)     # Yellow
            self.max_lat_spd = 42.0
            self.lat_accel = 3.0
            self.cut_interval = (5.5, 10.5)
            self.cut_prob = 0.38
        elif vtype == 'BIKE':
            self.width = 13
            self.length = 26
            self.color_body = (45, 48, 52)      # Dark frame
            self.color_top = (235, 75, 30)      # Helmet
            self.max_lat_spd = 60.0
            self.lat_accel = 4.0
            self.cut_interval = (4.5, 8.5)
            self.cut_prob = 0.45
        else: # CAR
            self.width = 25
            self.length = 48
            self.color_body = random.choice([
                (225, 225, 230),  # White
                (185, 32, 32),    # Red
                (170, 175, 180),  # Silver
                (215, 180, 45)    # Taxi yellow
            ])
            self.color_top = (45, 45, 50)
            self.max_lat_spd = 35.0
            self.lat_accel = 2.2
            self.cut_interval = (6.5, 13.0)
            self.cut_prob = 0.30

        self.cut_timer = random.uniform(self.cut_interval[0] * 0.5, self.cut_interval[1])

    def update(self, dt: float, road, all_traffic=None, player_car=None, pedestrians=None, cows=None):
        if self.honk_timer > 0:
            self.honk_timer -= dt
            self.is_honking = (self.honk_timer > 0)

        # 1. Autonomous collision avoidance & speed control (supports stopping to 0)
        desired_speed = self.cruising_speed

        # A. Check vehicles ahead & Deadlock Negotiation
        if all_traffic:
            for other in all_traffic:
                if other is self:
                    continue
                dy = self.y - other.y
                dx = abs(self.x - other.x)
                if 0 < dy < 155 and dx < (self.width + other.width) * 0.65:
                    min_gap = (self.length + other.length) * 0.5 + 32.0
                    if dy < min_gap:
                        # Check for deadlock (both vehicles near standstill)
                        if self.speed < 25.0 and other.speed < 25.0:
                            has_priority, _ = negotiate_deadlock_priority(self, other, road)
                            if has_priority:
                                desired_speed = max(desired_speed, 24.0)  # Creep forward out of deadlock
                            else:
                                desired_speed = 0.0  # Yield to higher priority / smaller vehicle
                                # Lateral yield nudge towards road shoulder to open corridor
                                if other.x > self.x:
                                    self.sub_lane_jitter = max(-24.0, self.sub_lane_jitter - 12.0 * dt)
                                else:
                                    self.sub_lane_jitter = min(24.0, self.sub_lane_jitter + 12.0 * dt)
                        else:
                            desired_speed = 0.0 # Full stop at rest to prevent collision
                    else:
                        gap_factor = max(0.0, min(1.0, (dy - min_gap) / 65.0))
                        desired_speed = min(desired_speed, other.speed * gap_factor)

        # B. Check player car ahead & Deadlock Negotiation
        if player_car:
            dy = self.y - player_car.y
            dx = abs(self.x - player_car.x)
            if 0 < dy < 155 and dx < (self.width + player_car.width) * 0.65:
                min_gap = (self.length + player_car.length) * 0.5 + 32.0
                if dy < min_gap:
                    if self.speed < 25.0 and player_car.speed < 25.0:
                        has_priority, _ = negotiate_deadlock_priority(self, player_car, road)
                        if has_priority:
                            desired_speed = max(desired_speed, 24.0)
                        else:
                            desired_speed = 0.0
                            if player_car.x > self.x:
                                self.sub_lane_jitter = max(-24.0, self.sub_lane_jitter - 12.0 * dt)
                            else:
                                self.sub_lane_jitter = min(24.0, self.sub_lane_jitter + 12.0 * dt)
                    else:
                        desired_speed = 0.0 # Full stop behind player car
                else:
                    gap_factor = max(0.0, min(1.0, (dy - min_gap) / 65.0))
                    desired_speed = min(desired_speed, player_car.speed * gap_factor)

        # C. Check crossing pedestrians ahead
        if pedestrians:
            for ped in pedestrians:
                dy = self.y - ped.y
                dx = abs(self.x - ped.x)
                if 0 < dy < 125 and dx < (self.width * 0.5 + ped.radius + 20.0):
                    if dy < 50.0:
                        desired_speed = 0.0 # Full stop with safe margin for pedestrians!
                        self.is_honking = True
                        self.honk_timer = 0.45
                    else:
                        desired_speed = min(desired_speed, 18.0)

        # D. Check cows ahead (predict behavior and adjust trajectory from far off)
        if cows:
            for cow in cows:
                dy = self.y - cow.y
                dx = abs(self.x - cow.x)
                if 0 < dy < 190 and dx < (self.width * 0.5 + cow.width * 0.5 + 30.0):
                    left_e, right_e, _, _ = road.get_road_edges(self.y)
                    safe_l = left_e + self.width * 0.65 + 6.0
                    safe_r = right_e - self.width * 0.65 - 6.0
                    if cow.x >= self.x:
                        self.target_x = max(safe_l, min(self.target_x, cow.x - cow.width * 0.5 - self.width * 0.5 - 24.0))
                    else:
                        self.target_x = min(safe_r, max(self.target_x, cow.x + cow.width * 0.5 + self.width * 0.5 + 24.0))

                    if dy < 60.0:
                        desired_speed = 0.0 # Full stop before cow to prevent hitting
                        self.is_honking = True
                        self.honk_timer = 0.5
                    elif dy < 125.0:
                        desired_speed = min(desired_speed, 32.0)

        # E. Execute acceleration / braking (Reduced acceleration from rest by 20%)
        if desired_speed < self.speed:
            self.speed = max(desired_speed, self.speed - self.decel * dt)
            self.is_braking = True
        else:
            rest_scale = REST_ACCEL_FACTOR if self.speed < 40.0 else (REST_ACCEL_FACTOR + (1.0 - REST_ACCEL_FACTOR) * min(1.0, (self.speed - 40.0) / 40.0))
            effective_accel = self.accel * rest_scale
            self.speed = min(desired_speed, self.speed + effective_accel * dt)
            self.is_braking = False

        self.speed = max(0.0, self.speed) # Can come to complete rest

        # Move forward along road
        self.y -= self.speed * dt
        left, right, road_cx, rw = road.get_road_edges(self.y)

        # 2. Road boundaries bounds
        safe_left = left + self.width * 0.65 + 6.0
        safe_right = right - self.width * 0.65 - 6.0

        lanes, lane_w = road.get_virtual_lanes(self.y)
        num_lanes = len(lanes)
        self.lane_idx = min(num_lanes - 1, max(0, self.lane_idx))

        # 3. Random lane cut decision (checks gap clearance first)
        self.cut_timer -= dt
        if self.cut_timer <= 0:
            if random.random() < self.cut_prob:
                possible_lanes = [idx for idx in range(num_lanes) if idx != self.lane_idx]
                if possible_lanes:
                    cand_lane_idx = random.choice(possible_lanes)
                    if self.vtype == 'BIKE':
                        cand_jitter = random.uniform(-lane_w * 0.18, lane_w * 0.18)
                    elif self.vtype in ('TRUCK', 'BUS'):
                        cand_jitter = random.uniform(-lane_w * 0.08, lane_w * 0.08)
                    else:
                        cand_jitter = random.uniform(-lane_w * 0.10, lane_w * 0.10)

                    cand_x = lanes[cand_lane_idx] + cand_jitter
                    cand_x = max(safe_left + 4, min(safe_right - 4, cand_x))

                    # Check if target lateral lane is clear of neighbors
                    is_clear = True
                    if all_traffic:
                        for other in all_traffic:
                            if other is self:
                                continue
                            if abs(other.y - self.y) < 70 and abs(other.x - cand_x) < (self.width + other.width) * 0.65:
                                is_clear = False
                                break
                    if player_car and abs(player_car.y - self.y) < 70 and abs(player_car.x - cand_x) < (self.width + player_car.width) * 0.65:
                        is_clear = False

                    if is_clear:
                        self.lane_idx = cand_lane_idx
                        self.sub_lane_jitter = cand_jitter

            self.cut_timer = random.uniform(self.cut_interval[0], self.cut_interval[1])

        # 4. Smooth lateral kinematics with STRICT ROAD CLAMPING (Never go offroad!)
        # Stationary vehicles cannot turn or slide laterally in place!
        if self.speed <= 1.0:
            self.vx = 0.0
        else:
            # Maintain virtual lane position with slight within-lane jitter
            target_x = lanes[self.lane_idx] + self.sub_lane_jitter
            target_x = max(safe_left, min(safe_right, target_x))
            self.lateral_offset = target_x - road_cx

            lateral_error = target_x - self.x
            desired_vx = max(-self.max_lat_spd, min(self.max_lat_spd, lateral_error * 2.0))
            self.vx += (desired_vx - self.vx) * min(1.0, self.lat_accel * dt)
            self.x += self.vx * dt

            # Enforce strict boundary clamp:
            if self.x < safe_left:
                self.x = safe_left
                self.vx = max(0.0, self.vx)
            elif self.x > safe_right:
                self.x = safe_right
                self.vx = min(0.0, self.vx)

            tilt_angle = math.atan2(self.vx, max(40.0, self.speed)) * 0.60

            # 5. Heading calculation with smooth lane-change sway (only when in motion)
            road_tangent = road.get_tangent_angle(self.y)
            self.heading = road_tangent + tilt_angle
            self.angle_deg = -math.degrees(self.heading)

    def get_collision_rect(self) -> pygame.Rect:
        return pygame.Rect(
            int(self.x - self.width / 2),
            int(self.y - self.length / 2),
            int(self.width),
            int(self.length)
        )

    def get_bounding_radius(self) -> float:
        return max(self.width, self.length) / 2.0 + SAFETY_MARGIN_CAR

    def draw(self, surface: pygame.Surface, camera_y: float):
        sy = self.y - camera_y
        h = surface.get_height()
        if sy < -100 or sy > h + 100:
            return

        surf_w = int(max(self.width, self.length) * 1.5)
        surf_h = surf_w
        veh_surf = pygame.Surface((surf_w, surf_h), pygame.SRCALPHA)

        cx = surf_w // 2
        cy = surf_h // 2
        hw = self.width // 2
        hl = self.length // 2

        body_rect = pygame.Rect(cx - hw, cy - hl, self.width, self.length)
        pygame.draw.rect(veh_surf, (20, 20, 20), body_rect, border_radius=4)
        pygame.draw.rect(veh_surf, self.color_body, body_rect.inflate(-2, -2), border_radius=3)

        if self.vtype == 'BUS':
            dest_rect = pygame.Rect(cx - hw + 3, cy - hl + 3, self.width - 6, 7)
            pygame.draw.rect(veh_surf, (240, 200, 20), dest_rect, border_radius=2)
            windshield = pygame.Rect(cx - hw + 3, cy - hl + 11, self.width - 6, 12)
            pygame.draw.rect(veh_surf, (180, 230, 255), windshield, border_radius=2)
            for wy in range(cy - hl + 28, cy + hl - 16, 10):
                pygame.draw.rect(veh_surf, (190, 230, 250), (cx - hw + 2, wy, 4, 6), border_radius=1)
                pygame.draw.rect(veh_surf, (190, 230, 250), (cx + hw - 6, wy, 4, 6), border_radius=1)
            rack_rect = pygame.Rect(cx - hw + 6, cy - 14, self.width - 12, 28)
            pygame.draw.rect(veh_surf, (110, 105, 95), rack_rect, width=1, border_radius=2)
            pygame.draw.rect(veh_surf, (45, 75, 110), rack_rect.inflate(-3, -4), border_radius=2)
            pygame.draw.line(veh_surf, (180, 220, 245), (cx - hw + 4, cy + hl - 4), (cx + hw - 4, cy + hl - 4), 2)
            tail_col = (255, 30, 20) if (self.is_braking or self.speed < 5.0) else (170, 20, 15)
            pygame.draw.circle(veh_surf, tail_col, (cx - hw + 3, cy + hl - 2), 2)
            pygame.draw.circle(veh_surf, tail_col, (cx + hw - 3, cy + hl - 2), 2)

        elif self.vtype == 'AUTO':
            front_pt = (cx, cy - hl)
            pygame.draw.polygon(veh_surf, (20, 20, 20), [
                (cx - hw, cy - hl + 10),
                (cx + hw, cy - hl + 10),
                front_pt
            ])
            canopy_rect = pygame.Rect(cx - hw + 2, cy - hl + 6, self.width - 4, self.length - 12)
            pygame.draw.rect(veh_surf, self.color_top, canopy_rect, border_radius=3)
            pygame.draw.line(veh_surf, (180, 230, 255), (cx - hw + 4, cy - hl + 7), (cx + hw - 4, cy - hl + 7), 2)
            tail_col = (255, 30, 20) if (self.is_braking or self.speed < 5.0) else (160, 20, 15)
            pygame.draw.circle(veh_surf, tail_col, (cx - hw + 3, cy + hl - 2), 2)
            pygame.draw.circle(veh_surf, tail_col, (cx + hw - 3, cy + hl - 2), 2)

        elif self.vtype == 'TRUCK':
            cabin_rect = pygame.Rect(cx - hw + 2, cy - hl + 2, self.width - 4, 22)
            cargo_rect = pygame.Rect(cx - hw + 2, cy - hl + 26, self.width - 4, self.length - 28)
            pygame.draw.rect(veh_surf, (240, 160, 20), cabin_rect, border_radius=3)
            pygame.draw.rect(veh_surf, self.color_top, cargo_rect, border_radius=2)
            pygame.draw.line(veh_surf, (200, 240, 255), (cx - hw + 4, cy - hl + 14), (cx + hw - 4, cy - hl + 14), 3)
            bumper_col = (255, 40, 30) if (self.is_braking or self.speed < 5.0) else (255, 230, 0)
            pygame.draw.line(veh_surf, bumper_col, (cx - hw + 2, cy + hl - 2), (cx + hw - 2, cy + hl - 2), 3)

        elif self.vtype == 'BIKE':
            pygame.draw.circle(veh_surf, self.color_top, (cx, cy - 2), 5)
            pygame.draw.line(veh_surf, (20, 20, 20), (cx - 3, cy - 4), (cx + 3, cy - 4), 2)
            pygame.draw.line(veh_surf, (30, 30, 30), (cx - hw + 1, cy - hl + 6), (cx + hw - 1, cy - hl + 6), 2)
            pygame.draw.circle(veh_surf, (255, 255, 200), (cx, cy - hl + 2), 2)
            tail_col = (255, 30, 20) if (self.is_braking or self.speed < 5.0) else (160, 20, 15)
            pygame.draw.circle(veh_surf, tail_col, (cx, cy + hl - 2), 2)

        else: # CAR
            roof_rect = pygame.Rect(cx - hw + 3, cy - hl + 12, self.width - 6, self.length - 24)
            pygame.draw.rect(veh_surf, self.color_top, roof_rect, border_radius=3)
            pygame.draw.line(veh_surf, (200, 235, 255), (cx - hw + 4, cy - hl + 12), (cx + hw - 4, cy - hl + 12), 3)
            pygame.draw.line(veh_surf, (160, 200, 230), (cx - hw + 4, cy + hl - 12), (cx + hw - 4, cy + hl - 12), 2)
            tail_col = (255, 30, 20) if (self.is_braking or self.speed < 5.0) else (160, 20, 15)
            pygame.draw.circle(veh_surf, tail_col, (cx - hw + 3, cy + hl - 2), 2)
            pygame.draw.circle(veh_surf, tail_col, (cx + hw - 3, cy + hl - 2), 2)

        rotated_surf = pygame.transform.rotate(veh_surf, self.angle_deg)
        rot_rect = rotated_surf.get_rect(center=(int(self.x), int(sy)))
        surface.blit(rotated_surf, rot_rect)

    def get_velocity_vector(self) -> Tuple[float, float]:
        """Returns instantaneous velocity vector (vx, vy) in world coordinates."""
        return (self.vx, -self.speed)

    def get_predicted_trajectory(self, road, duration: float = 2.0, steps: int = 5) -> List[Tuple[float, float]]:
        """
        Projects future positions along the vehicle's heading and lane trajectory
        over the specified duration (seconds).
        """
        if self.speed <= 1.0 and abs(self.vx) <= 1.0:
            return [(self.x, self.y)]

        pts = [(self.x, self.y)]
        dt_step = duration / max(1, steps)
        curr_y = self.y
        left, right, road_cx, _ = road.get_road_edges(curr_y)
        safe_left = left + self.width * 0.65 + 6.0
        safe_right = right - self.width * 0.65 - 6.0

        for i in range(1, steps + 1):
            tau = i * dt_step
            fut_y = self.y - self.speed * tau
            lanes, _ = road.get_virtual_lanes(fut_y)
            if 0 <= self.lane_idx < len(lanes):
                tgt_x = lanes[self.lane_idx] + self.sub_lane_jitter
            else:
                tgt_x = road.get_road_center(fut_y) + getattr(self, 'lateral_offset', 0.0)

            tgt_x = max(safe_left, min(safe_right, tgt_x))
            blend = min(1.0, tau * self.lat_accel)
            fut_x = (self.x + self.vx * tau) * (1.0 - blend) + tgt_x * blend
            pts.append((fut_x, fut_y))

        return pts

    def draw_direction_vector(self, surface: pygame.Surface, camera_y: float, road=None):
        """
        Renders the directional velocity vector and projected trajectory path in debug view.
        """
        sy = self.y - camera_y
        h = surface.get_height()
        if sy < -180 or sy > h + 180:
            return

        if self.speed <= 1.5 and abs(self.vx) <= 1.0:
            # Standstill vehicle: small amber warning ring
            pygame.draw.circle(surface, (255, 180, 50, 70), (int(self.x), int(sy)), int(self.width * 0.65), 1)
            return

        # Vector magnitude scales with speed
        vec_len = max(32.0, min(130.0, self.length * 0.45 + self.speed * 0.65))

        # Velocity direction unit vector
        speed_mag = math.hypot(self.vx, -self.speed)
        if speed_mag > 1.0:
            dir_x = self.vx / speed_mag
            dir_y = -self.speed / speed_mag
        else:
            dir_x = math.sin(self.heading)
            dir_y = -math.cos(self.heading)

        front_dist = self.length * 0.45
        front_x = self.x + math.sin(self.heading) * front_dist
        front_y = sy - math.cos(self.heading) * front_dist
        tip_x = front_x + dir_x * vec_len
        tip_y = front_y + dir_y * vec_len

        # Styling: Orange-red if lane cutting, Gold if fast, Cyan if steady cruising
        is_cutting = abs(self.vx) > 3.5
        if is_cutting:
            vec_color = (255, 110, 40, 230)
            glow_color = (255, 80, 20, 95)
        elif self.speed > 130.0:
            vec_color = (255, 215, 0, 235)
            glow_color = (255, 190, 0, 95)
        else:
            vec_color = (50, 230, 255, 215)
            glow_color = (0, 190, 255, 85)

        # Draw projected trajectory path if road is available
        if road:
            traj_pts = self.get_predicted_trajectory(road, duration=1.8, steps=5)
            if len(traj_pts) >= 2:
                screen_traj = [(int(px), int(py - camera_y)) for px, py in traj_pts]
                for i in range(len(screen_traj) - 1):
                    p1 = screen_traj[i]
                    p2 = screen_traj[i+1]
                    pygame.draw.line(surface, glow_color, p1, p2, 4)
                    pygame.draw.line(surface, vec_color, p1, p2, 2)
                for pt in screen_traj[1:]:
                    pygame.draw.circle(surface, vec_color, pt, 3)

        # Draw main direction vector line
        pygame.draw.line(surface, glow_color, (int(front_x), int(front_y)), (int(tip_x), int(tip_y)), 5)
        pygame.draw.line(surface, vec_color, (int(front_x), int(front_y)), (int(tip_x), int(tip_y)), 2)

        # Draw arrowhead
        arrow_size = 8.5
        angle = math.atan2(dir_y, dir_x)
        left_angle = angle + math.pi * 0.82
        right_angle = angle - math.pi * 0.82

        p_left = (tip_x + math.cos(left_angle) * arrow_size, tip_y + math.sin(left_angle) * arrow_size)
        p_right = (tip_x + math.cos(right_angle) * arrow_size, tip_y + math.sin(right_angle) * arrow_size)

        pygame.draw.polygon(surface, vec_color, [
            (int(tip_x), int(tip_y)),
            (int(p_left[0]), int(p_left[1])),
            (int(p_right[0]), int(p_right[1]))
        ])



class ObstacleManager:
    def __init__(self, road):
        self.road = road
        self.potholes = []
        self.traffic = []
        self.pedestrians = []
        self.cows = []

        self.next_pothole_y = -180
        self.next_traffic_y = -150
        self.next_pedestrian_y = -180
        self.next_cow_y = -320

        self.traffic_density = 0.5  # 0.1 (Sparse) to 1.0 (Rush Hour)
        self.max_pedestrians = 7    # Independent separate quota (not counted as traffic)

        self.potholes_avoided = 0
        self.traffic_overtaken = 0
        self.pedestrians_avoided = 0
        self.cows_navigated = 0

        # Pre-populate road with initial lively traffic, pedestrians, and cows
        self._prepopulate_world(0.0)

    def _prepopulate_world(self, start_y: float):
        """Spawns an initial distribution of vehicles, pedestrians, and cows ahead of the player."""
        # Pre-populate 11 traffic vehicles across the road ahead
        curr_y = start_y - 140.0
        for _ in range(11):
            curr_y -= random.uniform(85.0, 160.0)
            left, right, cx, rw = self.road.get_road_edges(curr_y)
            vtype = random.choices(
                ['AUTO', 'TRUCK', 'CAR', 'BIKE', 'BUS'],
                weights=[0.24, 0.18, 0.28, 0.18, 0.12]
            )[0]
            spd, off, lane_idx, jitter = self._get_vehicle_spawn_params(vtype, rw, curr_y)
            self.traffic.append(TrafficVehicle(cx + off, curr_y, vtype, spd, off, lane_idx, jitter))

        # Pre-populate 5 pedestrians
        ped_y = start_y - 180.0
        for _ in range(5):
            ped_y -= random.uniform(180.0, 270.0)
            left, right, cx, rw = self.road.get_road_edges(ped_y)
            side = random.choice([-1, 1])
            pedx = (left - random.uniform(6, 20)) if side == -1 else (right + random.uniform(6, 20))
            self.pedestrians.append(Pedestrian(pedx, ped_y, side))

        # Pre-populate 2 cow herds
        cow_y = start_y - 340.0
        self._spawn_cow_herd(cow_y, herd_id=1)
        self._spawn_cow_herd(cow_y - 500.0, herd_id=2)

    def _get_vehicle_spawn_params(self, vtype: str, rw: float, y: float = 0.0):
        lanes, lane_w = self.road.get_virtual_lanes(y)
        num_lanes = len(lanes)
        if vtype in ('TRUCK', 'BUS'):
            speed = random.uniform(80.0, 105.0) if vtype == 'TRUCK' else random.uniform(90.0, 115.0)
            lane_idx = 0 if random.random() < 0.65 else (1 if num_lanes > 2 else 0)
            jitter = random.uniform(-lane_w * 0.08, lane_w * 0.08)
        elif vtype == 'AUTO':
            speed = random.uniform(96.0, 124.0)
            lane_idx = 0 if random.random() < 0.55 else min(1, num_lanes - 1)
            jitter = random.uniform(-lane_w * 0.12, lane_w * 0.12)
        elif vtype == 'BIKE':
            speed = random.uniform(130.0, 170.0)
            lane_idx = random.randrange(num_lanes)
            jitter = random.uniform(-lane_w * 0.18, lane_w * 0.18)
        else: # CAR
            speed = random.uniform(115.0, 150.0)
            lane_idx = random.randrange(num_lanes)
            jitter = random.uniform(-lane_w * 0.10, lane_w * 0.10)

        target_x = lanes[lane_idx] + jitter
        offset = target_x - self.road.get_road_center(y)
        return speed, offset, lane_idx, jitter

    def _spawn_cow_herd(self, base_y: float, herd_id: int):
        herd_size = random.choices([1, 2, 3, 4], weights=[0.25, 0.38, 0.24, 0.13])[0]
        herd_mode = random.choices(['EDGE', 'RESTING_MIDDLE'], weights=[0.66, 0.34])[0]
        left, right, cx, rw = self.road.get_road_edges(base_y)

        if herd_mode == 'RESTING_MIDDLE':
            # Resting in road (max 2 resting together so road remains passable)
            rest_count = min(herd_size, 2)
            for i in range(rest_count):
                cow_offset = random.uniform(-rw * 0.14, rw * 0.14)
                cow_y = base_y + i * random.uniform(22.0, 36.0)
                self.cows.append(Cow(cx + cow_offset, cow_y, state='RESTING', side=0, herd_id=herd_id))
        else:
            # Edge walking/grazing herd
            side = random.choice([-1, 1])
            for i in range(herd_size):
                cow_y = base_y + i * random.uniform(26.0, 44.0)
                cow_left, cow_right, _, _ = self.road.get_road_edges(cow_y)
                base_x = (cow_left + random.uniform(4.0, 20.0)) if side == -1 else (cow_right - random.uniform(4.0, 20.0))
                self.cows.append(Cow(base_x, cow_y, state='WALKING_EDGE', side=side, herd_id=herd_id))

    def update(self, dt: float, player_ref):
        if hasattr(player_ref, 'y'):
            player_y = player_ref.y
            player_car = player_ref
        else:
            player_y = float(player_ref)
            player_car = None

        # Update dynamic traffic
        for veh in self.traffic:
            veh.update(dt, self.road, self.traffic, player_car, self.pedestrians, self.cows)

        # Update pedestrians
        for ped in self.pedestrians:
            ped.update(dt, self.road, self.traffic, player_car)

        # Update cows
        for cow in self.cows:
            cow.update(dt, self.road, self.traffic, player_car)

        spawn_horizon = player_y - 1300

        # 1. Potholes
        while self.next_pothole_y > spawn_horizon:
            py = self.next_pothole_y
            left, right, cx, rw = self.road.get_road_edges(py)

            ptype = random.choices(['SMALL', 'MEDIUM', 'LARGE'], weights=[0.42, 0.44, 0.14])[0]
            if ptype == 'SMALL':
                rx = random.uniform(9.0, 14.0)
                ry = random.uniform(8.0, 13.0)
            elif ptype == 'MEDIUM':
                rx = random.uniform(17.0, 26.0)
                ry = random.uniform(14.0, 22.0)
            else: # LARGE
                rx = random.uniform(32.0, 48.0)
                ry = random.uniform(20.0, 32.0)

            margin = rx + 18.0
            if right - left > margin * 2.2:
                px = random.uniform(left + margin, right - margin)
                self.potholes.append(Pothole(px, py, rx, ry))

            self.next_pothole_y -= random.uniform(260.0, 580.0)

        # 2. Dynamic Traffic (Scales strongly with traffic_density slider)
        # Quota: 8 at min density, 18 at medium (0.5), up to 28 at rush hour (1.0)
        max_active_traffic = int(7 + self.traffic_density * 21)

        # Ensure spawn point does not lag behind player
        if self.next_traffic_y > player_y - 450:
            self.next_traffic_y = player_y - 550

        while self.next_traffic_y > spawn_horizon and len(self.traffic) < max_active_traffic:
            ty = self.next_traffic_y
            left, right, cx, rw = self.road.get_road_edges(ty)

            vtype = random.choices(
                ['AUTO', 'TRUCK', 'CAR', 'BIKE', 'BUS'],
                weights=[0.24, 0.18, 0.28, 0.18, 0.12]
            )[0]

            spd, off, lane_idx, jitter = self._get_vehicle_spawn_params(vtype, rw, ty)
            tx = cx + off
            self.traffic.append(TrafficVehicle(tx, ty, vtype, spd, off, lane_idx, jitter))

            # Intervals scale dynamically with slider
            min_inv = 50.0 + (1.0 - self.traffic_density) * 140.0
            max_inv = 95.0 + (1.0 - self.traffic_density) * 230.0
            self.next_traffic_y -= random.uniform(min_inv, max_inv)

        # 3. Independent Pedestrian Quota (Not counted as traffic)
        if self.next_pedestrian_y > player_y - 400:
            self.next_pedestrian_y = player_y - 500

        while self.next_pedestrian_y > spawn_horizon and len(self.pedestrians) < self.max_pedestrians:
            pedy = self.next_pedestrian_y
            left, right, cx, rw = self.road.get_road_edges(pedy)
            side = random.choice([-1, 1])
            pedx = (left - random.uniform(6, 20)) if side == -1 else (right + random.uniform(6, 20))
            self.pedestrians.append(Pedestrian(pedx, pedy, side))
            self.next_pedestrian_y -= random.uniform(190.0, 340.0)

        # 4. Cow Herds
        if self.next_cow_y > player_y - 500:
            self.next_cow_y = player_y - 650

        while self.next_cow_y > spawn_horizon and len(self.cows) < 12:
            self._spawn_cow_herd(self.next_cow_y, herd_id=random.randint(10, 999))
            self.next_cow_y -= random.uniform(480.0, 850.0)

        # 5. Clean up behind and far ahead of player
        despawn_behind_y = player_y + 380
        despawn_ahead_y = player_y - 1650

        # Potholes
        remaining_potholes = []
        for p in self.potholes:
            if p.y > despawn_behind_y:
                self.potholes_avoided += 1
            elif p.y >= despawn_ahead_y:
                remaining_potholes.append(p)
        self.potholes = remaining_potholes

        # Traffic (Recycle both when passed or when speeding far ahead)
        remaining_traffic = []
        for t in self.traffic:
            if t.y > despawn_behind_y:
                self.traffic_overtaken += 1
            elif t.y >= despawn_ahead_y:
                remaining_traffic.append(t)
        self.traffic = remaining_traffic

        # Pedestrians
        remaining_pedestrians = []
        for ped in self.pedestrians:
            if ped.y > despawn_behind_y:
                self.pedestrians_avoided += 1
            elif ped.y >= despawn_ahead_y:
                remaining_pedestrians.append(ped)
        self.pedestrians = remaining_pedestrians

        # Cows
        remaining_cows = []
        for cow in self.cows:
            if cow.y > despawn_behind_y:
                self.cows_navigated += 1
            elif cow.y >= despawn_ahead_y:
                remaining_cows.append(cow)
        self.cows = remaining_cows

    def get_obstacles_in_range(self, min_y: float, max_y: float):
        p_in_range = [p for p in self.potholes if min_y <= p.y <= max_y]
        t_in_range = [t for t in self.traffic if min_y <= t.y <= max_y]
        ped_in_range = [ped for ped in self.pedestrians if min_y <= ped.y <= max_y]
        cow_in_range = [cow for cow in self.cows if min_y <= cow.y <= max_y]
        return p_in_range, t_in_range, ped_in_range, cow_in_range

    def draw(self, surface: pygame.Surface, camera_y: float):
        for p in self.potholes:
            p.draw(surface, camera_y)
        for cow in self.cows:
            cow.draw(surface, camera_y)
        for ped in self.pedestrians:
            ped.draw(surface, camera_y)
        for t in self.traffic:
            t.draw(surface, camera_y)

    def draw_traffic_vectors(self, surface: pygame.Surface, camera_y: float):
        for t in self.traffic:
            t.draw_direction_vector(surface, camera_y, self.road)
