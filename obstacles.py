"""
Obstacles module: Potholes, Dynamic Traffic (with sudden lane cuts), and Random Pedestrians.
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


class Pedestrian:
    """
    Random pedestrian walking along roadside shoulder who may unpredictably
    cut across the asphalt road without any warning or indication.
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
        self.side = side  # -1 = left side, 1 = right side
        self.radius = 8.0
        self.hit = False

        # Behavior
        self.state = 'WALKING_SHOULDER'
        self.shirt_color = random.choice(self.SHIRT_COLORS)
        self.skin_color = (195, 145, 105)
        self.hair_color = (25, 20, 20)

        # Speed and walking direction
        self.walk_speed_y = random.uniform(18.0, 32.0) * random.choice([-1, 1])
        self.cross_speed_x = random.uniform(34.0, 48.0) * (-self.side) # Cross towards opposite side
        
        # Sudden crossing decision timer (unpredictable jaywalking)
        self.cross_timer = random.uniform(1.2, 5.0)
        self.walk_phase = random.uniform(0.0, 6.28)

    def update(self, dt: float, road):
        self.walk_phase += dt * 8.0
        left, right, cx, rw = road.get_road_edges(self.y)

        if self.state == 'WALKING_SHOULDER':
            self.cross_timer -= dt
            # Walk along dirt edge
            target_edge = (left - 14) if self.side == -1 else (right + 14)
            self.x += (target_edge - self.x) * min(1.0, 4.0 * dt)
            self.y += self.walk_speed_y * dt

            # Sudden crossing without indication!
            if self.cross_timer <= 0:
                self.state = 'CROSSING'

        elif self.state == 'CROSSING':
            # Cut directly across the road without waiting
            self.x += self.cross_speed_x * dt
            self.y += (self.walk_speed_y * 0.4) * dt

            # Once crossed to opposite shoulder, return to walking along shoulder
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

        # Walking leg stride animation
        leg_offset = math.sin(self.walk_phase) * 4.0
        pygame.draw.circle(surface, (40, 40, 45), (int(ix + leg_offset), isy + 4), 3)
        pygame.draw.circle(surface, (40, 40, 45), (int(ix - leg_offset), isy - 4), 3)

        # Shoulders / Torso (Shirt)
        torso_rect = pygame.Rect(ix - 6, isy - 4, 12, 8)
        pygame.draw.ellipse(surface, self.shirt_color, torso_rect)
        pygame.draw.ellipse(surface, (20, 20, 20), torso_rect, 1)

        # Head with hair
        pygame.draw.circle(surface, self.skin_color, (ix, isy), 4)
        pygame.draw.circle(surface, self.hair_color, (ix, isy - 1), 3)


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

        # Random lane cutting behavior (without indicator)
        self.cut_timer = random.uniform(1.8, 4.5)
        self.swerve_rate = 2.8 # Lateral responsiveness

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
        left, right, road_cx, rw = road.get_road_edges(self.y)

        # Unpredictable random lane cutting across road (no indicators!)
        self.cut_timer -= dt
        if self.cut_timer <= 0:
            # Pick a new unpredictable lateral position across road
            if self.vtype == 'AUTO':
                # Autos frequently dart across from edge to middle or vice versa
                self.lateral_offset = random.choice([-rw * 0.32, 0.0, rw * 0.28, -rw * 0.15])
                self.swerve_rate = random.uniform(3.5, 5.5)
                self.cut_timer = random.uniform(2.5, 5.0)
            elif self.vtype == 'SCOOTER':
                # Scooters weave rapidly through gaps
                self.lateral_offset = random.uniform(-rw * 0.35, rw * 0.35)
                self.swerve_rate = random.uniform(4.0, 6.5)
                self.cut_timer = random.uniform(1.8, 4.0)
            elif self.vtype == 'TRUCK':
                # Trucks make wide, sweeping drifts across lanes
                self.lateral_offset = random.choice([-rw * 0.20, rw * 0.18, 0.0])
                self.swerve_rate = random.uniform(1.8, 2.8)
                self.cut_timer = random.uniform(4.0, 8.0)
            else: # CAR
                # Cars swerve to overtake or change lines abruptly
                self.lateral_offset = random.choice([-rw * 0.26, rw * 0.26, 0.0])
                self.swerve_rate = random.uniform(3.0, 4.8)
                self.cut_timer = random.uniform(3.0, 6.0)

        # Smoothly track road curvature and execute lateral cuts
        target_x = road_cx + self.lateral_offset
        # Clamp within road surface
        target_x = max(left + self.width, min(right - self.width, target_x))

        # Lateral movement with variable swerve rate
        self.x += (target_x - self.x) * min(1.0, self.swerve_rate * dt)

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
            # Rear bumper painted yellow/red
            pygame.draw.line(veh_surf, (255, 230, 0), (cx - hw + 2, cy + hl - 2), (cx + hw - 2, cy + hl - 2), 2)

        elif self.vtype == 'SCOOTER':
            pygame.draw.circle(veh_surf, self.color_top, (cx, cy), 6) # Helmet
            pygame.draw.circle(veh_surf, (30, 30, 30), (cx, cy - 8), 3) # Headlight / handlebars

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
        
        # Generation trackers (in world Y coordinates, decreasing)
        self.next_pothole_y = -150
        self.next_traffic_y = -300
        self.next_pedestrian_y = -180

        # Stats
        self.potholes_avoided = 0
        self.traffic_overtaken = 0
        self.pedestrians_avoided = 0

    def update(self, dt: float, player_y: float):
        # 1. Update dynamic traffic
        for veh in self.traffic:
            veh.update(dt, self.road)

        # 2. Update pedestrians
        for ped in self.pedestrians:
            ped.update(dt, self.road)

        # 3. Procedural spawning ahead of player
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

        # Spawn dynamic traffic
        while self.next_traffic_y > spawn_horizon:
            ty = self.next_traffic_y
            left, right, cx, rw = self.road.get_road_edges(ty)
            vtype = random.choice(TrafficVehicle.TYPES)
            if vtype == 'AUTO':
                speed = random.uniform(90.0, 130.0)
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

        # Spawn pedestrians along shoulders
        while self.next_pedestrian_y > spawn_horizon:
            pedy = self.next_pedestrian_y
            left, right, cx, rw = self.road.get_road_edges(pedy)
            side = random.choice([-1, 1])
            pedx = (left - random.uniform(6, 22)) if side == -1 else (right + random.uniform(6, 22))
            self.pedestrians.append(Pedestrian(pedx, pedy, side))
            self.next_pedestrian_y -= random.uniform(150.0, 310.0)

        # 4. Clean up objects behind player & update stats
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
        """Returns all potholes, traffic, and pedestrians between [min_y, max_y]."""
        p_in_range = [p for p in self.potholes if min_y <= p.y <= max_y]
        t_in_range = [t for t in self.traffic if min_y <= t.y <= max_y]
        ped_in_range = [ped for ped in self.pedestrians if min_y <= ped.y <= max_y]
        return p_in_range, t_in_range, ped_in_range

    def draw(self, surface: pygame.Surface, camera_y: float):
        # Draw potholes on the road surface
        for p in self.potholes:
            p.draw(surface, camera_y)
        # Draw pedestrians walking/crossing
        for ped in self.pedestrians:
            ped.draw(surface, camera_y)
        # Draw traffic vehicles on top
        for t in self.traffic:
            t.draw(surface, camera_y)
