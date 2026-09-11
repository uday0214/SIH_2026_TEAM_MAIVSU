"""
Obstacles module: Potholes and Dynamic Traffic (Auto-rickshaws, Trucks, Cars, Scooters).
"""

import math
import random
import pygame
from config import (
    COLOR_POTHOLE_INNER, COLOR_POTHOLE_RIM,
    SAFETY_MARGIN_CAR, SAFETY_MARGIN_POTHOLE
)

class Pothole:
    def __init__(self, x: float, y: float, rx: float, ry: float):
        self.x = x
        self.y = y
        self.rx = rx  # radius x
        self.ry = ry  # radius y
        self.effective_radius = max(rx, ry)
        # Visual jaggedness points for rough edge
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
        """Ellipse containment check with safety margin."""
        dx = (px - self.x) / (self.rx + margin)
        dy = (py - self.y) / (self.ry + margin)
        return (dx * dx + dy * dy) <= 1.0

    def draw(self, surface: pygame.Surface, camera_y: float):
        sy = self.y - camera_y
        h = surface.get_height()
        if sy < -50 or sy > h + 50:
            return

        # 1. Draw outer cracked rim
        screen_pts = [(int(self.x + px), int(sy + py)) for px, py in self.rim_points]
        if len(screen_pts) >= 3:
            pygame.draw.polygon(surface, COLOR_POTHOLE_RIM, screen_pts)

        # 2. Draw dark inner cavity
        inner_rect = pygame.Rect(
            int(self.x - self.rx * 0.75),
            int(sy - self.ry * 0.75),
            int(self.rx * 1.5),
            int(self.ry * 1.5)
        )
        pygame.draw.ellipse(surface, COLOR_POTHOLE_INNER, inner_rect)
        
        # 3. Inner shadow rim
        pygame.draw.ellipse(surface, (15, 12, 10), inner_rect, 2)


class TrafficVehicle:
    TYPES = ['AUTO', 'TRUCK', 'CAR', 'SCOOTER']

    def __init__(self, x: float, y: float, vtype: str, speed: float, lateral_offset: float = 0.0):
        self.x = x
        self.y = y
        self.vtype = vtype
        self.speed = speed # pixels/sec (negative Y direction, moving forward)
        self.lateral_offset = lateral_offset # target offset from road center
        self.heading = 0.0 # radians
        self.angle_deg = 0.0
        self.hit = False

        if vtype == 'AUTO':
            self.width = 22
            self.length = 36
            self.color_body = (34, 139, 34)   # Indian auto green
            self.color_top = (245, 195, 35)    # Yellow canopy
        elif vtype == 'TRUCK':
            self.width = 34
            self.length = 74
            self.color_body = (210, 80, 20)    # Vibrant orange/red Indian highway truck
            self.color_top = (40, 110, 180)    # Blue painted cargo bed
        elif vtype == 'SCOOTER':
            self.width = 14
            self.length = 26
            self.color_body = (70, 70, 70)
            self.color_top = (230, 90, 40)     # Helmet color
        else: # CAR
            self.width = 25
            self.length = 48
            self.color_body = random.choice([
                (220, 220, 225),  # White
                (180, 30, 30),    # Red
                (170, 175, 180),  # Silver
                (210, 180, 50)    # Taxi yellow
            ])
            self.color_top = (45, 45, 50)

    def update(self, dt: float, road):
        # Move forward along road
        self.y -= self.speed * dt

        # Smoothly track road curvature and maintain slight lane/offset
        road_cx = road.get_road_center(self.y)
        target_x = road_cx + self.lateral_offset

        # Smooth drift towards desired offset
        self.x += (target_x - self.x) * min(1.0, 3.0 * dt)

        # Compute heading from road curvature
        self.heading = road.get_tangent_angle(self.y)
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
        if sy < -80 or sy > h + 80:
            return

        # Create temporary vehicle surface for rotated drawing
        surf_w = int(max(self.width, self.length) * 1.5)
        surf_h = surf_w
        veh_surf = pygame.Surface((surf_w, surf_h), pygame.SRCALPHA)

        cx = surf_w // 2
        cy = surf_h // 2
        hw = self.width // 2
        hl = self.length // 2

        # Draw vehicle base
        body_rect = pygame.Rect(cx - hw, cy - hl, self.width, self.length)
        pygame.draw.rect(veh_surf, (20, 20, 20), body_rect, border_radius=4)
        pygame.draw.rect(veh_surf, self.color_body, body_rect.inflate(-2, -2), border_radius=3)

        if self.vtype == 'AUTO':
            # Three-wheeler taper in front
            front_pt = (cx, cy - hl)
            pygame.draw.polygon(veh_surf, (20, 20, 20), [
                (cx - hw, cy - hl + 10),
                (cx + hw, cy - hl + 10),
                front_pt
            ])
            # Auto canopy
            canopy_rect = pygame.Rect(cx - hw + 2, cy - hl + 6, self.width - 4, self.length - 12)
            pygame.draw.rect(veh_surf, self.color_top, canopy_rect, border_radius=3)
            # Windshield
            pygame.draw.line(veh_surf, (180, 230, 255), (cx - hw + 4, cy - hl + 7), (cx + hw - 4, cy - hl + 7), 2)

        elif self.vtype == 'TRUCK':
            # Cabin in front, cargo bed in rear
            cabin_rect = pygame.Rect(cx - hw + 2, cy - hl + 2, self.width - 4, 22)
            cargo_rect = pygame.Rect(cx - hw + 2, cy - hl + 26, self.width - 4, self.length - 28)
            pygame.draw.rect(veh_surf, (240, 160, 20), cabin_rect, border_radius=3)
            pygame.draw.rect(veh_surf, self.color_top, cargo_rect, border_radius=2)
            # Windshield
            pygame.draw.line(veh_surf, (200, 240, 255), (cx - hw + 4, cy - hl + 14), (cx + hw - 4, cy - hl + 14), 3)
            # Decorative yellow/red striped rear bumper
            pygame.draw.line(veh_surf, (255, 230, 0), (cx - hw + 2, cy + hl - 2), (cx + hw - 2, cy + hl - 2), 2)

        elif self.vtype == 'SCOOTER':
            # Slim chassis & rider helmet
            pygame.draw.circle(veh_surf, self.color_top, (cx, cy), 6) # Helmet
            pygame.draw.circle(veh_surf, (30, 30, 30), (cx, cy - 8), 3) # Headlight / handlebars

        else: # CAR
            # Windshield and cabin roof
            roof_rect = pygame.Rect(cx - hw + 3, cy - hl + 12, self.width - 6, self.length - 24)
            pygame.draw.rect(veh_surf, self.color_top, roof_rect, border_radius=3)
            # Front windshield
            pygame.draw.line(veh_surf, (200, 235, 255), (cx - hw + 4, cy - hl + 12), (cx + hw - 4, cy - hl + 12), 3)
            # Rear windshield
            pygame.draw.line(veh_surf, (160, 200, 230), (cx - hw + 4, cy + hl - 12), (cx + hw - 4, cy + hl - 12), 2)

        # Rotate according to heading
        rotated_surf = pygame.transform.rotate(veh_surf, self.angle_deg)
        rot_rect = rotated_surf.get_rect(center=(int(self.x), int(sy)))
        surface.blit(rotated_surf, rot_rect)


class ObstacleManager:
    def __init__(self, road):
        self.road = road
        self.potholes = []
        self.traffic = []
        
        # Generation trackers (in world Y coordinates, decreasing)
        self.next_pothole_y = -150
        self.next_traffic_y = -300

        # Stats
        self.potholes_avoided = 0
        self.traffic_overtaken = 0

    def update(self, dt: float, player_y: float):
        # 1. Update dynamic traffic
        for veh in self.traffic:
            veh.update(dt, self.road)

        # 2. Procedural spawning ahead of player
        spawn_horizon = player_y - 1000

        # Spawn potholes
        while self.next_pothole_y > spawn_horizon:
            py = self.next_pothole_y
            left, right, cx, rw = self.road.get_road_edges(py)
            
            # Place pothole inside drivable asphalt
            margin = 35.0
            px = random.uniform(left + margin, right - margin)
            rx = random.uniform(14.0, 26.0)
            ry = random.uniform(12.0, 22.0)
            
            self.potholes.append(Pothole(px, py, rx, ry))
            self.next_pothole_y -= random.uniform(100.0, 240.0)

        # Spawn traffic
        while self.next_traffic_y > spawn_horizon:
            ty = self.next_traffic_y
            left, right, cx, rw = self.road.get_road_edges(ty)
            
            vtype = random.choice(TrafficVehicle.TYPES)
            # Auto rickshaws drive slower, cars faster
            if vtype == 'AUTO':
                speed = random.uniform(90.0, 130.0)
                # Auto often hugs left or wanders
                offset = random.choice([-rw * 0.28, rw * 0.25, -rw * 0.15])
            elif vtype == 'TRUCK':
                speed = random.uniform(85.0, 120.0)
                offset = random.choice([-rw * 0.22, rw * 0.22])
            elif vtype == 'SCOOTER':
                speed = random.uniform(110.0, 150.0)
                offset = random.uniform(-rw * 0.35, rw * 0.35)
            else: # CAR
                speed = random.uniform(130.0, 175.0)
                offset = random.choice([-rw * 0.25, 0.0, rw * 0.25])

            tx = cx + offset
            self.traffic.append(TrafficVehicle(tx, ty, vtype, speed, offset))
            self.next_traffic_y -= random.uniform(220.0, 420.0)

        # 3. Clean up objects behind player & update stats
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

    def get_obstacles_in_range(self, min_y: float, max_y: float):
        """Returns all potholes and traffic between [min_y, max_y]."""
        p_in_range = [p for p in self.potholes if min_y <= p.y <= max_y]
        t_in_range = [t for t in self.traffic if min_y <= t.y <= max_y]
        return p_in_range, t_in_range

    def draw(self, surface: pygame.Surface, camera_y: float):
        # Draw potholes on the road surface first
        for p in self.potholes:
            p.draw(surface, camera_y)
        # Draw traffic vehicles on top
        for t in self.traffic:
            t.draw(surface, camera_y)
