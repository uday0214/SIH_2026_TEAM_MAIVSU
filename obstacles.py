"""
Obstacles module: Potholes, Realistic Traffic (Truck, Bus, Car, Auto, Bike with smooth kinematics),
and Random Jaywalking Pedestrians.
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
        num_pts = 10
        for i in range(num_pts):
            ang = (i / num_pts) * 2 * math.pi
            r_jitter = random.uniform(0.85, 1.15)
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
        if sy < -50 or sy > h + 50:
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
    cut across the asphalt road without any warning.
    """
    SHIRT_COLORS = [
        (230, 80, 40),   # Bright saffron/orange
        (240, 240, 245), # White kurta
        (30, 140, 210),  # Blue shirt
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

        self.walk_speed_y = random.uniform(18.0, 32.0) * random.choice([-1, 1])
        self.cross_speed_x = random.uniform(34.0, 48.0) * (-self.side)
        
        self.cross_timer = random.uniform(1.2, 5.0)
        self.walk_phase = random.uniform(0.0, 6.28)

    def update(self, dt: float, road):
        self.walk_phase += dt * 8.0
        left, right, cx, rw = road.get_road_edges(self.y)

        if self.state == 'WALKING_SHOULDER':
            self.cross_timer -= dt
            target_edge = (left - 14) if self.side == -1 else (right + 14)
            self.x += (target_edge - self.x) * min(1.0, 4.0 * dt)
            self.y += self.walk_speed_y * dt

            if self.cross_timer <= 0:
                self.state = 'CROSSING'

        elif self.state == 'CROSSING':
            self.x += self.cross_speed_x * dt
            self.y += (self.walk_speed_y * 0.4) * dt

            if self.side == -1 and self.x > right + 18:
                self.side = 1
                self.state = 'WALKING_SHOULDER'
                self.cross_timer = random.uniform(6.0, 12.0)
            elif self.side == 1 and self.x < left - 18:
                self.side = -1
                self.state = 'WALKING_SHOULDER'
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
    No teleporting - smooth acceleration, yaw tilt, and distinct driving profiles
    for Truck, Bus, Car, Auto, and Bike.
    """
    TYPES = ['TRUCK', 'BUS', 'CAR', 'AUTO', 'BIKE']

    def __init__(self, x: float, y: float, vtype: str, speed: float, lateral_offset: float = 0.0):
        self.x = x
        self.y = y
        self.vtype = vtype
        self.speed = speed
        self.lateral_offset = lateral_offset
        self.heading = 0.0
        self.angle_deg = 0.0
        self.hit = False

        # Smooth kinematic lateral velocity (px/s)
        self.vx = 0.0

        # Unique driving profiles per vehicle type
        if vtype == 'TRUCK':
            self.width = 34
            self.length = 78
            self.color_body = (210, 75, 20)     # Indian highway truck orange
            self.color_top = (35, 105, 175)     # Cargo blue
            self.max_lat_spd = 24.0             # Very gentle lateral speed
            self.lat_accel = 1.3                # Slow, heavy inertia
            self.cut_interval = (9.0, 17.0)     # Long duration between cuts
            self.cut_prob = 0.22                # Rare sharp cut (~25% lower probability)
        elif vtype == 'BUS':
            self.width = 32
            self.length = 88
            self.color_body = (175, 32, 38)     # State transport maroon/red
            self.color_top = (235, 230, 210)    # Cream roof
            self.max_lat_spd = 22.0             # Very smooth, steady
            self.lat_accel = 1.1                # High inertia
            self.cut_interval = (10.0, 18.0)
            self.cut_prob = 0.20
        elif vtype == 'AUTO':
            self.width = 22
            self.length = 36
            self.color_body = (34, 139, 34)    # Green
            self.color_top = (245, 195, 35)     # Yellow canopy
            self.max_lat_spd = 45.0             # Nimble
            self.lat_accel = 3.2
            self.cut_interval = (5.0, 9.5)
            self.cut_prob = 0.42
        elif vtype == 'BIKE':
            self.width = 13
            self.length = 26
            self.color_body = (45, 48, 52)      # Dark frame
            self.color_top = (235, 75, 30)      # Helmet color
            self.max_lat_spd = 65.0             # Fastest, weaves nimbly
            self.lat_accel = 4.2
            self.cut_interval = (4.0, 7.5)
            self.cut_prob = 0.50
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
            self.max_lat_spd = 38.0
            self.lat_accel = 2.4
            self.cut_interval = (6.0, 12.0)
            self.cut_prob = 0.35

        # Timer until next lane change decision
        self.cut_timer = random.uniform(self.cut_interval[0] * 0.5, self.cut_interval[1])

    def update(self, dt: float, road):
        # 1. Forward progression along road
        self.y -= self.speed * dt
        left, right, road_cx, rw = road.get_road_edges(self.y)

        # 2. Random lane cutting decision (without indicators, reduced frequency)
        self.cut_timer -= dt
        if self.cut_timer <= 0:
            if random.random() < self.cut_prob:
                # Select a new lane target across road
                if self.vtype in ('TRUCK', 'BUS'):
                    # Large heavy vehicles shift smoothly between left and center
                    self.lateral_offset = random.choice([-rw * 0.18, 0.0, rw * 0.16])
                elif self.vtype == 'AUTO':
                    self.lateral_offset = random.choice([-rw * 0.28, -rw * 0.12, rw * 0.22])
                elif self.vtype == 'BIKE':
                    # Bikes weave nimbly through openings
                    self.lateral_offset = random.uniform(-rw * 0.32, rw * 0.32)
                else: # CAR
                    self.lateral_offset = random.choice([-rw * 0.24, 0.0, rw * 0.24])

            # Reset timer
            self.cut_timer = random.uniform(self.cut_interval[0], self.cut_interval[1])

        # 3. Smooth kinematic lateral interpolation (Zero Teleportation)
        target_x = road_cx + self.lateral_offset
        # Constrain within asphalt road boundaries
        target_x = max(left + self.width * 0.75, min(right - self.width * 0.75, target_x))

        lateral_error = target_x - self.x
        desired_vx = max(-self.max_lat_spd, min(self.max_lat_spd, lateral_error * 2.0))
        # Accelerate lateral velocity towards desired velocity
        self.vx += (desired_vx - self.vx) * min(1.0, self.lat_accel * dt)
        self.x += self.vx * dt

        # 4. Realistic heading calculation:
        # Base road tangent + smooth yaw tilt while performing lane cut
        road_tangent = road.get_tangent_angle(self.y)
        tilt_angle = math.atan2(self.vx, max(50.0, self.speed)) * 0.65
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
            # Long passenger bus
            # Destination board above windshield
            dest_rect = pygame.Rect(cx - hw + 3, cy - hl + 3, self.width - 6, 7)
            pygame.draw.rect(veh_surf, (240, 200, 20), dest_rect, border_radius=2)
            # Front windshield
            windshield = pygame.Rect(cx - hw + 3, cy - hl + 11, self.width - 6, 12)
            pygame.draw.rect(veh_surf, (180, 230, 255), windshield, border_radius=2)
            # Side passenger windows
            for wy in range(cy - hl + 28, cy + hl - 16, 10):
                pygame.draw.rect(veh_surf, (190, 230, 250), (cx - hw + 2, wy, 4, 6), border_radius=1)
                pygame.draw.rect(veh_surf, (190, 230, 250), (cx + hw - 6, wy, 4, 6), border_radius=1)
            # Roof luggage carrier rack
            rack_rect = pygame.Rect(cx - hw + 6, cy - 14, self.width - 12, 28)
            pygame.draw.rect(veh_surf, (110, 105, 95), rack_rect, width=1, border_radius=2)
            pygame.draw.rect(veh_surf, (45, 75, 110), rack_rect.inflate(-3, -4), border_radius=2)
            # Rear window & red tail lights
            pygame.draw.line(veh_surf, (180, 220, 245), (cx - hw + 4, cy + hl - 4), (cx + hw - 4, cy + hl - 4), 2)
            pygame.draw.circle(veh_surf, (255, 30, 20), (cx - hw + 3, cy + hl - 2), 2)
            pygame.draw.circle(veh_surf, (255, 30, 20), (cx + hw - 3, cy + hl - 2), 2)

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

        elif self.vtype == 'TRUCK':
            cabin_rect = pygame.Rect(cx - hw + 2, cy - hl + 2, self.width - 4, 22)
            cargo_rect = pygame.Rect(cx - hw + 2, cy - hl + 26, self.width - 4, self.length - 28)
            pygame.draw.rect(veh_surf, (240, 160, 20), cabin_rect, border_radius=3)
            pygame.draw.rect(veh_surf, self.color_top, cargo_rect, border_radius=2)
            pygame.draw.line(veh_surf, (200, 240, 255), (cx - hw + 4, cy - hl + 14), (cx + hw - 4, cy - hl + 14), 3)
            pygame.draw.line(veh_surf, (255, 230, 0), (cx - hw + 2, cy + hl - 2), (cx + hw - 2, cy + hl - 2), 2)

        elif self.vtype == 'BIKE':
            # Motorcycle with rider helmet
            pygame.draw.circle(veh_surf, self.color_top, (cx, cy - 2), 5)
            pygame.draw.line(veh_surf, (20, 20, 20), (cx - 3, cy - 4), (cx + 3, cy - 4), 2)
            pygame.draw.line(veh_surf, (30, 30, 30), (cx - hw + 1, cy - hl + 6), (cx + hw - 1, cy - hl + 6), 2)
            pygame.draw.circle(veh_surf, (255, 255, 200), (cx, cy - hl + 2), 2)
            pygame.draw.circle(veh_surf, (255, 30, 20), (cx, cy + hl - 2), 2)

        else: # CAR
            roof_rect = pygame.Rect(cx - hw + 3, cy - hl + 12, self.width - 6, self.length - 24)
            pygame.draw.rect(veh_surf, self.color_top, roof_rect, border_radius=3)
            pygame.draw.line(veh_surf, (200, 235, 255), (cx - hw + 4, cy - hl + 12), (cx + hw - 4, cy - hl + 12), 3)
            pygame.draw.line(veh_surf, (160, 200, 230), (cx - hw + 4, cy + hl - 12), (cx + hw - 4, cy + hl - 12), 2)

        rotated_surf = pygame.transform.rotate(veh_surf, self.angle_deg)
        rot_rect = rotated_surf.get_rect(center=(int(self.x), int(sy)))
        surface.blit(rotated_surf, rot_rect)


class ObstacleManager:
    def __init__(self, road):
        self.road = road
        self.potholes = []
        self.traffic = []
        self.pedestrians = []
        
        self.next_pothole_y = -150
        self.next_traffic_y = -300
        self.next_pedestrian_y = -180

        self.potholes_avoided = 0
        self.traffic_overtaken = 0
        self.pedestrians_avoided = 0

    def update(self, dt: float, player_y: float):
        for veh in self.traffic:
            veh.update(dt, self.road)

        for ped in self.pedestrians:
            ped.update(dt, self.road)

        spawn_horizon = player_y - 1100

        # Spawn potholes
        while self.next_pothole_y > spawn_horizon:
            py = self.next_pothole_y
            left, right, cx, rw = self.road.get_road_edges(py)
            margin = 35.0
            px = random.uniform(left + margin, right - margin)
            rx = random.uniform(14.0, 26.0)
            ry = random.uniform(12.0, 22.0)
            self.potholes.append(Pothole(px, py, rx, ry))
            self.next_pothole_y -= random.uniform(100.0, 240.0)

        # Spawn dynamic traffic (all 5 varieties with individual profiles)
        while self.next_traffic_y > spawn_horizon:
            ty = self.next_traffic_y
            left, right, cx, rw = self.road.get_road_edges(ty)
            
            vtype = random.choices(
                ['AUTO', 'TRUCK', 'CAR', 'BIKE', 'BUS'],
                weights=[0.24, 0.18, 0.28, 0.18, 0.12]
            )[0]

            if vtype == 'TRUCK':
                speed = random.uniform(80.0, 105.0)
                offset = random.choice([-rw * 0.20, rw * 0.18])
            elif vtype == 'BUS':
                speed = random.uniform(92.0, 118.0)
                offset = random.choice([-rw * 0.22, 0.0])
            elif vtype == 'AUTO':
                speed = random.uniform(98.0, 126.0)
                offset = random.choice([-rw * 0.28, rw * 0.24, -rw * 0.14])
            elif vtype == 'BIKE':
                speed = random.uniform(150.0, 195.0)
                offset = random.uniform(-rw * 0.32, rw * 0.32)
            else: # CAR
                speed = random.uniform(130.0, 170.0)
                offset = random.choice([-rw * 0.24, 0.0, rw * 0.24])

            tx = cx + offset
            self.traffic.append(TrafficVehicle(tx, ty, vtype, speed, offset))
            self.next_traffic_y -= random.uniform(210.0, 420.0)

        # Spawn pedestrians along shoulders
        while self.next_pedestrian_y > spawn_horizon:
            pedy = self.next_pedestrian_y
            left, right, cx, rw = self.road.get_road_edges(pedy)
            side = random.choice([-1, 1])
            pedx = (left - random.uniform(6, 22)) if side == -1 else (right + random.uniform(6, 22))
            self.pedestrians.append(Pedestrian(pedx, pedy, side))
            self.next_pedestrian_y -= random.uniform(150.0, 310.0)

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
