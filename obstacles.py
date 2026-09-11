"""
Obstacles module: Dynamic Potholes, Collision-Free Traffic (Truck, Bus, Car, Auto, Bike with IDM-like
following and complete stop-at-rest support), and Jaywalking Pedestrians.
"""

import math
import random
import pygame
from config import (
    COLOR_POTHOLE_INNER, COLOR_POTHOLE_RIM,
    SAFETY_MARGIN_CAR, SAFETY_MARGIN_POTHOLE, SAFETY_MARGIN_PEDESTRIAN
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


class TrafficVehicle:
    """
    Dynamic traffic vehicle with realistic vehicular kinematics:
    1. Collision avoidance with other traffic, pedestrians, and main car.
    2. Ability to come to a complete stop (speed = 0.0) and restart smoothly.
    3. Strict road boundary clamping: NEVER goes offroad!
    4. Smooth kinematic lane cuts without teleporting.
    """
    TYPES = ['TRUCK', 'BUS', 'CAR', 'AUTO', 'BIKE']

    def __init__(self, x: float, y: float, vtype: str, speed: float, lateral_offset: float = 0.0):
        self.x = x
        self.y = y
        self.vtype = vtype
        self.cruising_speed = speed
        self.speed = speed
        self.lateral_offset = lateral_offset
        self.heading = 0.0
        self.angle_deg = 0.0
        self.hit = False

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

    def update(self, dt: float, road, all_traffic=None, player_car=None, pedestrians=None):
        # 1. Autonomous collision avoidance & speed control (supports stopping to 0)
        desired_speed = self.cruising_speed

        # A. Check vehicles ahead
        if all_traffic:
            for other in all_traffic:
                if other is self:
                    continue
                dy = self.y - other.y
                dx = abs(self.x - other.x)
                if 0 < dy < 140 and dx < (self.width + other.width) * 0.62:
                    min_gap = (self.length + other.length) * 0.5 + 24.0
                    if dy < min_gap:
                        desired_speed = 0.0 # Full stop at rest!
                    else:
                        gap_factor = max(0.0, min(1.0, (dy - min_gap) / 60.0))
                        desired_speed = min(desired_speed, other.speed * gap_factor)

        # B. Check player car ahead
        if player_car:
            dy = self.y - player_car.y
            dx = abs(self.x - player_car.x)
            if 0 < dy < 140 and dx < (self.width + player_car.width) * 0.62:
                min_gap = (self.length + player_car.length) * 0.5 + 24.0
                if dy < min_gap:
                    desired_speed = 0.0 # Full stop behind player car
                else:
                    gap_factor = max(0.0, min(1.0, (dy - min_gap) / 60.0))
                    desired_speed = min(desired_speed, player_car.speed * gap_factor)

        # C. Check crossing pedestrians ahead
        if pedestrians:
            for ped in pedestrians:
                dy = self.y - ped.y
                dx = abs(self.x - ped.x)
                if 0 < dy < 110 and dx < (self.width * 0.5 + ped.radius + 16.0):
                    if dy < 42.0:
                        desired_speed = 0.0 # Full stop for pedestrians!
                    else:
                        desired_speed = min(desired_speed, 20.0)

        # D. Execute acceleration / braking
        if desired_speed < self.speed:
            self.speed = max(desired_speed, self.speed - self.decel * dt)
            self.is_braking = True
        else:
            self.speed = min(desired_speed, self.speed + self.accel * dt)
            self.is_braking = False

        self.speed = max(0.0, self.speed) # Can come to complete rest

        # Move forward along road
        self.y -= self.speed * dt
        left, right, road_cx, rw = road.get_road_edges(self.y)

        # 2. Road boundaries bounds
        safe_left = left + self.width * 0.65 + 6.0
        safe_right = right - self.width * 0.65 - 6.0

        # 3. Random lane cut decision (checks gap clearance first)
        self.cut_timer -= dt
        if self.cut_timer <= 0:
            if random.random() < self.cut_prob:
                if self.vtype in ('TRUCK', 'BUS'):
                    cand_offset = random.choice([-rw * 0.16, 0.0, rw * 0.15])
                elif self.vtype == 'AUTO':
                    cand_offset = random.choice([-rw * 0.26, -rw * 0.10, rw * 0.20])
                elif self.vtype == 'BIKE':
                    cand_offset = random.uniform(-rw * 0.28, rw * 0.28)
                else: # CAR
                    cand_offset = random.choice([-rw * 0.22, 0.0, rw * 0.22])

                cand_x = road_cx + cand_offset
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
                    self.lateral_offset = cand_offset

            self.cut_timer = random.uniform(self.cut_interval[0], self.cut_interval[1])

        # 4. Smooth lateral kinematics with STRICT ROAD CLAMPING (Never go offroad!)
        target_x = road_cx + self.lateral_offset
        target_x = max(safe_left, min(safe_right, target_x))

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

        # 5. Heading calculation with smooth lane-change sway
        road_tangent = road.get_tangent_angle(self.y)
        tilt_angle = math.atan2(self.vx, max(40.0, self.speed)) * 0.60
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


class ObstacleManager:
    def __init__(self, road):
        self.road = road
        self.potholes = []
        self.traffic = []
        self.pedestrians = []
        
        self.next_pothole_y = -180
        self.next_traffic_y = -300
        self.next_pedestrian_y = -220
        self.traffic_density = 0.5 # 0.1 (Sparse) to 1.0 (Rush Hour)

        self.potholes_avoided = 0
        self.traffic_overtaken = 0
        self.pedestrians_avoided = 0

    def update(self, dt: float, player_ref):
        # Support both player_car instance or player_y float
        if hasattr(player_ref, 'y'):
            player_y = player_ref.y
            player_car = player_ref
        else:
            player_y = float(player_ref)
            player_car = None

        # Update dynamic traffic with inter-vehicle collision avoidance
        for veh in self.traffic:
            veh.update(dt, self.road, self.traffic, player_car, self.pedestrians)

        # Update pedestrians with vehicle threat detection & self-preservation
        for ped in self.pedestrians:
            ped.update(dt, self.road, self.traffic, player_car)

        spawn_horizon = player_y - 1100

        # Spawn potholes: Reduced probability by 55-60%, dynamically generated sizes
        while self.next_pothole_y > spawn_horizon:
            py = self.next_pothole_y
            left, right, cx, rw = self.road.get_road_edges(py)

            # Dynamically generated sizes (Small, Medium, Large Trench)
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

            # Reduced spawn frequency (260 - 580 px)
            self.next_pothole_y -= random.uniform(260.0, 580.0)

        # Spawn dynamic traffic based on traffic_density
        max_active_traffic = int(2 + self.traffic_density * 9)
        while self.next_traffic_y > spawn_horizon and len(self.traffic) < max_active_traffic:
            ty = self.next_traffic_y
            left, right, cx, rw = self.road.get_road_edges(ty)
            
            vtype = random.choices(
                ['AUTO', 'TRUCK', 'CAR', 'BIKE', 'BUS'],
                weights=[0.24, 0.18, 0.28, 0.18, 0.12]
            )[0]

            if vtype == 'TRUCK':
                speed = random.uniform(80.0, 105.0)
                offset = random.choice([-rw * 0.18, rw * 0.16])
            elif vtype == 'BUS':
                speed = random.uniform(92.0, 118.0)
                offset = random.choice([-rw * 0.20, 0.0])
            elif vtype == 'AUTO':
                speed = random.uniform(98.0, 126.0)
                offset = random.choice([-rw * 0.26, -rw * 0.12, rw * 0.20])
            elif vtype == 'BIKE':
                speed = random.uniform(145.0, 190.0)
                offset = random.uniform(-rw * 0.28, rw * 0.28)
            else: # CAR
                speed = random.uniform(125.0, 165.0)
                offset = random.choice([-rw * 0.22, 0.0, rw * 0.22])

            tx = cx + offset
            self.traffic.append(TrafficVehicle(tx, ty, vtype, speed, offset))
            
            min_inv = 110.0 + (1.0 - self.traffic_density) * 260.0
            max_inv = 200.0 + (1.0 - self.traffic_density) * 380.0
            self.next_traffic_y -= random.uniform(min_inv, max_inv)

        # Spawn pedestrians along shoulders
        while self.next_pedestrian_y > spawn_horizon:
            pedy = self.next_pedestrian_y
            left, right, cx, rw = self.road.get_road_edges(pedy)
            side = random.choice([-1, 1])
            pedx = (left - random.uniform(6, 22)) if side == -1 else (right + random.uniform(6, 22))
            self.pedestrians.append(Pedestrian(pedx, pedy, side))
            self.next_pedestrian_y -= random.uniform(170.0, 340.0)

        # Clean up behind player
        despawn_y = player_y + 400

        remaining_potholes = []
        for p in self.potholes:
            if p.y > despawn_y:
                self.potholes_avoided += 1
            else:
                remaining_potholes.append(p)
        self.potholes = remaining_potholes

        remaining_traffic = []
        for t in self.traffic:
            if t.y > despawn_y:
                self.traffic_overtaken += 1
            else:
                remaining_traffic.append(t)
        self.traffic = remaining_traffic

        remaining_pedestrians = []
        for ped in self.pedestrians:
            if ped.y > despawn_y:
                self.pedestrians_avoided += 1
            else:
                remaining_pedestrians.append(ped)
        self.pedestrians = remaining_pedestrians

    def get_obstacles_in_range(self, min_y: float, max_y: float):
        p_in_range = [p for p in self.potholes if min_y <= p.y <= max_y]
        t_in_range = [t for t in self.traffic if min_y <= t.y <= max_y]
        ped_in_range = [ped for ped in self.pedestrians if min_y <= ped.y <= max_y]
        return p_in_range, t_in_range, ped_in_range

    def draw(self, surface: pygame.Surface, camera_y: float):
        for p in self.potholes:
            p.draw(surface, camera_y)
        for ped in self.pedestrians:
            ped.draw(surface, camera_y)
        for t in self.traffic:
            t.draw(surface, camera_y)
