"""
Circular Disc Sensor & 360° Radar Perception Field.
Provides omnidirectional situational awareness, sector threat localization,
and a Red-Zone Safe Distance Priority Algorithm with predictive slow maneuvering,
applicable to all vehicles except bikes.
"""

import math
import pygame
from typing import Dict, List, Tuple, Optional

def get_oriented_box_corners(cx: float, cy: float, width: float, length: float, heading: float) -> List[Tuple[float, float]]:
    """
    Returns the 4 world/screen corners of a vehicle's oriented bounding box (OBB)
    rotated by its heading angle. Heading 0 points along -y (up).
    """
    hw = width * 0.5
    hl = length * 0.5
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)

    return [
        (cx + hw * cos_h + hl * sin_h, cy + hw * sin_h - hl * cos_h),  # Front-Right
        (cx + hw * cos_h - hl * sin_h, cy + hw * sin_h + hl * cos_h),  # Rear-Right
        (cx - hw * cos_h - hl * sin_h, cy - hw * sin_h + hl * cos_h),  # Rear-Left
        (cx - hw * cos_h + hl * sin_h, cy - hw * sin_h - hl * cos_h),  # Front-Left
    ]


class CircularDiscSensor:
    """
    Omnidirectional 360° Circular Disc Radar Sensor.
    Concentric Zones:
      - Critical Core (Red): Inner safety bubble. Safe distance priority keeps cars out of this zone.
        If an NPC vehicle enters the red zone, the vehicle slowly maneuvers past it based on
        predicted motion.
      - Caution Buffer (Yellow): Normal traffic zone where vehicles can follow and drive close naturally.
      - Perception Disc (Cyan): Scanning radar zone across 8 radial sectors.
    """
    SECTORS = ['FRONT', 'FR', 'RIGHT', 'RR', 'REAR', 'RL', 'LEFT', 'FL']

    def __init__(self, r_inner: float = 46.0, r_mid: float = 98.0, r_outer: float = 175.0,
                 is_player: bool = True, is_bike: bool = False):
        self.r_inner = r_inner  # Red zone boundary (primary safe distance threshold)
        self.r_mid = r_mid      # Yellow zone boundary (traffic buffer)
        self.r_outer = r_outer  # Cyan perception boundary

        self.is_player = is_player
        self.is_bike = is_bike

        self.sector_distances: Dict[str, float] = {s: r_outer for s in self.SECTORS}
        self.sector_threats: Dict[str, str] = {s: 'CLEAR' for s in self.SECTORS}
        self.sector_points: Dict[str, Optional[Tuple[float, float]]] = {s: None for s in self.SECTORS}
        self.sector_velocities: Dict[str, float] = {s: 0.0 for s in self.SECTORS}

        self.sweep_angle = 0.0
        self.pulse_phase = 0.0

        # Algorithmic state
        self.nearest_dist = r_outer
        self.nearest_threat = 'CLEAR'
        self.critical_breached = False

        # Red-Zone Priority Safe Distance & Predictive Maneuver state
        self.vehicle_in_red = False
        self.vehicle_in_caution = False
        self.vehicle_in_yellow = False
        self.closest_vehicle_dist = r_outer
        self.closest_vehicle_threat = 'CLEAR'
        self.closest_vehicle_sector = 'FRONT'

        self.red_zone_urgency = 0.0          # 0.0 to 1.0 (depth into red zone)
        self.red_zone_maneuver_steer = 0.0   # Predictive lateral evasion steer
        self.red_zone_slow_speed = 999.0     # Crawl bypass speed in red zone

        self.predicted_other_x = 0.0
        self.predicted_other_y = 0.0
        self.predicted_other_dir = "HOLDING LANE"
        self.predicted_maneuver_side = "RIGHT"

        # Compatibility aliases
        self.safe_distance_urgency = 0.0
        self.yellow_repulsion_steer = 0.0
        self.repulsion_steer = 0.0
        self.headway_safe_speed = 999.0

    def update(self, vehicle, road, all_traffic=None, player_car=None,
               pedestrians=None, cows=None, potholes=None, dt: float = 0.0):
        """
        Updates radar perception and executes the Red-Zone Safe Distance Priority Algorithm.
        """
        self.sweep_angle = (self.sweep_angle + dt * 4.6) % (2 * math.pi)
        self.pulse_phase = (self.pulse_phase + dt * 2.6) % 1.0

        for s in self.SECTORS:
            self.sector_distances[s] = self.r_outer
            self.sector_threats[s] = 'CLEAR'
            self.sector_points[s] = None
            self.sector_velocities[s] = 0.0

        contacts = []
        veh_contacts = []

        # 1. Other traffic vehicles
        if all_traffic:
            for other in all_traffic:
                if other is vehicle:
                    continue
                v_rad = max(other.width, other.length) * 0.38
                contacts.append((other.x, other.y, f"TRAFFIC_{other.vtype}", v_rad, other.speed, True))
                veh_contacts.append(other)

        # 2. Player car (for NPC vehicles)
        if player_car and player_car is not vehicle:
            p_rad = max(player_car.width, player_car.length) * 0.38
            contacts.append((player_car.x, player_car.y, "AUTONOMOUS_CAR", p_rad, player_car.speed, True))
            veh_contacts.append(player_car)

        # 3. Pedestrians
        if pedestrians:
            for ped in pedestrians:
                contacts.append((ped.x, ped.y, 'PEDESTRIAN', ped.radius, getattr(ped, 'speed', 15.0), False))

        # 4. Cows
        if cows:
            for cow in cows:
                contacts.append((cow.x, cow.y, 'COW', cow.radius, getattr(cow, 'speed', 10.0), False))

        # 5. Potholes
        if potholes:
            for p in potholes:
                contacts.append((p.x, p.y, 'POTHOLE', p.effective_radius, 0.0, False))

        # 6. Road boundaries
        if road:
            for sample_y in [vehicle.y - 70.0, vehicle.y, vehicle.y + 70.0]:
                left, right, _, _ = road.get_road_edges(sample_y)
                contacts.append((left, sample_y, 'ROAD_EDGE', 6.0, 0.0, False))
                contacts.append((right, sample_y, 'ROAD_EDGE', 6.0, 0.0, False))

        nearest_d = self.r_outer
        nearest_t = 'CLEAR'

        v_in_red = False
        v_in_caution = False
        min_v_dist = self.r_outer
        min_v_threat = 'CLEAR'
        min_v_sec = 'FRONT'
        min_v_obj = None

        veh_heading = getattr(vehicle, 'heading', 0.0)

        for idx, (ox, oy, otype, rad, ospd, is_veh) in enumerate(contacts):
            dx = ox - vehicle.x
            dy = oy - vehicle.y
            raw_dist = math.hypot(dx, dy)
            eff_dist = max(0.0, raw_dist - rad)

            if eff_dist < self.r_outer:
                abs_ang = math.atan2(dx, -dy)
                rel_ang = (abs_ang - veh_heading + math.pi) % (2 * math.pi) - math.pi
                deg = math.degrees(rel_ang)

                # Map to 8 radial sectors
                if -22.5 <= deg < 22.5:
                    sec = 'FRONT'
                elif 22.5 <= deg < 67.5:
                    sec = 'FR'
                elif 67.5 <= deg < 112.5:
                    sec = 'RIGHT'
                elif 112.5 <= deg < 157.5:
                    sec = 'RR'
                elif deg >= 157.5 or deg < -157.5:
                    sec = 'REAR'
                elif -157.5 <= deg < -112.5:
                    sec = 'RL'
                elif -112.5 <= deg < -67.5:
                    sec = 'LEFT'
                else:
                    sec = 'FL'

                if eff_dist < self.sector_distances[sec]:
                    self.sector_distances[sec] = eff_dist
                    self.sector_threats[sec] = otype
                    self.sector_points[sec] = (ox, oy)
                    self.sector_velocities[sec] = ospd

                if eff_dist < nearest_d:
                    nearest_d = eff_dist
                    nearest_t = otype

                # Vehicle tracking
                if is_veh:
                    if eff_dist < min_v_dist:
                        min_v_dist = eff_dist
                        min_v_threat = otype
                        min_v_sec = sec
                        if idx < len(veh_contacts):
                            min_v_obj = veh_contacts[idx]
                    if eff_dist < self.r_mid:
                        v_in_caution = True
                    if eff_dist < (self.r_inner + 8.0):
                        v_in_red = True

        self.nearest_dist = nearest_d
        self.nearest_threat = nearest_t
        self.vehicle_in_red = v_in_red
        self.vehicle_in_caution = v_in_caution
        self.vehicle_in_yellow = v_in_caution
        self.closest_vehicle_dist = min_v_dist
        self.closest_vehicle_threat = min_v_threat
        self.closest_vehicle_sector = min_v_sec

        # Immediate nose-to-tail / direct collision breach check (gap < 8 px)
        self.critical_breached = any(
            self.sector_distances[s] < (self.r_inner * 0.55) and self.sector_threats[s] not in ['CLEAR', 'POTHOLE', 'ROAD_EDGE']
            for s in ['FRONT', 'FL', 'FR']
        )

        # =========================================================================
        # 🛡️ RED-ZONE SAFE DISTANCE & PREDICTIVE SLOW MANEUVER ALGORITHM
        # Cars should stay OUT of the red zone mostly.
        # If an NPC vehicle enters the red zone, the car slowly maneuvers past it,
        # predicting where the other car will probably go!
        # (Applies to all vehicles except bikes).
        # =========================================================================
        if self.is_bike:
            self.red_zone_urgency = 0.0
            self.red_zone_maneuver_steer = 0.0
            self.red_zone_slow_speed = 999.0
            self.yellow_repulsion_steer = 0.0
            self.repulsion_steer = 0.0
            self.headway_safe_speed = 999.0
            return

        # Check if another vehicle is approaching or inside the RED zone
        red_threshold = self.r_inner + 10.0
        if min_v_dist < red_threshold and min_v_obj is not None:
            # Urgency scales from 0.0 at red threshold to 1.0 deep inside red zone
            urgency = (red_threshold - min_v_dist) / max(1.0, red_threshold * 0.45)
            self.red_zone_urgency = max(0.0, min(1.0, urgency))
            self.safe_distance_urgency = self.red_zone_urgency

            # 1. PREDICT WHERE THE OTHER CAR IN THE RED ZONE WILL PROBABLY GO:
            other = min_v_obj
            other_vx = getattr(other, 'vx', 0.0)
            other_spd = getattr(other, 'speed', 30.0)
            other_hdg = getattr(other, 'heading', 0.0)

            # Projected lateral movement over ~1 second lookahead
            pred_dx = other_vx * 1.0
            pred_x = other.x + pred_dx
            pred_y = other.y - other_spd * math.cos(other_hdg) * 0.8
            self.predicted_other_x = pred_x
            self.predicted_other_y = pred_y

            # Road boundaries to find the open bypass lane
            left_e, right_e, road_cx, rw = road.get_road_edges(vehicle.y)
            safe_l = left_e + vehicle.width * 0.65 + 4.0
            safe_r = right_e - vehicle.width * 0.65 - 4.0

            gap_left = max(0.0, other.x - safe_l)
            gap_right = max(0.0, safe_r - other.x)

            # Analyze trajectory intent of other car
            if other_vx > 2.5:
                self.predicted_other_dir = "DRIFTING RIGHT"
                # Other car moving right, so pass on the LEFT!
                maneuver_target_x = max(safe_l, other.x - (other.width + vehicle.width) * 0.5 - 16.0)
                self.predicted_maneuver_side = "LEFT"
            elif other_vx < -2.5:
                self.predicted_other_dir = "DRIFTING LEFT"
                # Other car moving left, so pass on the RIGHT!
                maneuver_target_x = min(safe_r, other.x + (other.width + vehicle.width) * 0.5 + 16.0)
                self.predicted_maneuver_side = "RIGHT"
            else:
                self.predicted_other_dir = "HOLDING LANE"
                # Pick the wider open side of the road
                if gap_right >= gap_left:
                    maneuver_target_x = min(safe_r, other.x + (other.width + vehicle.width) * 0.5 + 16.0)
                    self.predicted_maneuver_side = "RIGHT"
                else:
                    maneuver_target_x = max(safe_l, other.x - (other.width + vehicle.width) * 0.5 - 16.0)
                    self.predicted_maneuver_side = "LEFT"

            # Compute predictive evasive steering to maneuver past into the open gap
            lat_err = maneuver_target_x - vehicle.x
            steer_cmd = max(-1.0, min(1.0, lat_err / 28.0)) * 1.55
            self.red_zone_maneuver_steer = steer_cmd
            self.yellow_repulsion_steer = steer_cmd
            self.repulsion_steer = steer_cmd

            # 2. SLOW MANEUVER SPEED:
            # Crawl cautiously past the car in the red zone (~12-18 km/h / 38-55 px/s)
            # Never stop completely unless direct nose-to-tail physical overlap within 10 px!
            dy_headway = vehicle.y - other.y
            dx_lateral = abs(vehicle.x - other.x)

            if 0 < dy_headway < (vehicle.length + other.length) * 0.45 and dx_lateral < (vehicle.width + other.width) * 0.48:
                # Direct bumper touch proximity; pause briefly to let maneuver steering establish lateral offset
                self.red_zone_slow_speed = 0.0
            else:
                # Slowly crawl and maneuver past!
                self.red_zone_slow_speed = max(38.0, min(65.0, other_spd * 0.85 + 16.0))

            self.headway_safe_speed = self.red_zone_slow_speed
        else:
            # Outside the red zone: natural driving flow in yellow/cyan zones
            self.red_zone_urgency = 0.0
            self.red_zone_maneuver_steer = 0.0
            self.red_zone_slow_speed = 999.0
            self.yellow_repulsion_steer = 0.0
            self.repulsion_steer = 0.0
            self.headway_safe_speed = 999.0
            self.safe_distance_urgency = 0.0

        # Automated directional horn for pedestrians or cows
        if self.is_player:
            front_d = min(self.sector_distances['FRONT'], self.sector_distances['FL'], self.sector_distances['FR'])
            front_threat = self.sector_threats['FRONT']
            if ('COW' in front_threat or 'PEDESTRIAN' in front_threat) and front_d < 145.0:
                if getattr(vehicle, 'honk_timer', 0.0) <= 0.05:
                    vehicle.honk_timer = 0.45
                    if 'COW' in front_threat:
                        vehicle.add_thought("RADAR: Bovine obstacle in front sector; pulsing horn.", "WARN")
                    else:
                        vehicle.add_thought("RADAR: Pedestrian proximity breach; sounding horn warning.", "WARN")

    def draw_world(self, surface: pygame.Surface, vehicle, camera_y: float,
                   show_radar: bool = True, show_hitbox: bool = False):
        """
        Renders the clean circular disc sensor on the road and/or rotated hitbox.
        Rotating sweep line is REMOVED - only the clean circular disc is kept on view.
        """
        sy = vehicle.y - camera_y
        h = surface.get_height()
        if sy < -self.r_outer or sy > h + self.r_outer:
            return

        cx = int(vehicle.x)
        cy = int(sy)

        # 1. Clean Circular Disc Sensor (NO rotating line!)
        if show_radar:
            disc_size = int(self.r_outer * 2 + 10)
            disc_surf = pygame.Surface((disc_size, disc_size), pygame.SRCALPHA)
            dcx = disc_size // 2
            dcy = disc_size // 2

            # Outer Perception Disc (Cyan)
            pygame.draw.circle(disc_surf, (0, 225, 255, 15), (dcx, dcy), int(self.r_outer))
            pygame.draw.circle(disc_surf, (0, 220, 255, 45), (dcx, dcy), int(self.r_outer), width=1)

            # Caution Buffer Disc (Yellow Zone)
            y_alpha = 40 if self.vehicle_in_caution else 18
            y_border_alpha = 140 if self.vehicle_in_caution else 65
            pygame.draw.circle(disc_surf, (255, 210, 40, y_alpha), (dcx, dcy), int(self.r_mid))
            pygame.draw.circle(disc_surf, (255, 210, 40, y_border_alpha), (dcx, dcy), int(self.r_mid),
                               width=2 if self.vehicle_in_caution else 1)

            # Critical Core (Red Zone - glows on breach or red zone entry)
            core_alpha = 100 if self.vehicle_in_red else 22
            core_border_alpha = 180 if self.vehicle_in_red else 60
            pygame.draw.circle(disc_surf, (255, 50, 50, core_alpha), (dcx, dcy), int(self.r_inner))
            pygame.draw.circle(disc_surf, (255, 50, 50, core_border_alpha), (dcx, dcy),
                               int(self.r_inner), width=2 if self.vehicle_in_red else 1)

            # Pulsing Wave Ring
            pulse_r = int(self.r_inner + self.pulse_phase * (self.r_outer - self.r_inner))
            pulse_alpha = int(55 * (1.0 - self.pulse_phase))
            pygame.draw.circle(disc_surf, (0, 240, 255, pulse_alpha), (dcx, dcy), pulse_r, width=1)

            # Sector Contact Blip Dots
            for sec, pt in self.sector_points.items():
                if pt is not None and self.sector_distances[sec] < self.r_outer:
                    bdx = pt[0] - vehicle.x
                    bdy = pt[1] - vehicle.y
                    bx = int(dcx + bdx)
                    by = int(dcy + bdy)
                    if 0 <= bx < disc_size and 0 <= by < disc_size:
                        dist = self.sector_distances[sec]
                        b_col = (255, 60, 60) if dist < self.r_inner else (255, 210, 40) if dist < self.r_mid else (0, 235, 255)
                        pygame.draw.circle(disc_surf, b_col, (bx, by), 4)
                        pygame.draw.circle(disc_surf, (255, 255, 255), (bx, by), 6, width=1)

            surface.blit(disc_surf, (cx - dcx, cy - dcy))

        # 2. Rotated Oriented Bounding Box (OBB) Hitbox
        if show_hitbox:
            corners = get_oriented_box_corners(vehicle.x, sy, vehicle.width, vehicle.length, vehicle.heading)
            hb_color = (255, 60, 60) if self.vehicle_in_red else (255, 215, 30) if self.vehicle_in_caution else (80, 240, 120)
            pygame.draw.polygon(surface, hb_color, [(int(px), int(py)) for px, py in corners], width=2)
            pygame.draw.circle(surface, (255, 255, 255), (cx, cy), 3)

    def draw_radar_hud(self, surface: pygame.Surface, cx: int, cy: int, radius: int = 38):
        """Draws mini 360° radar screen for the AI dashboard."""
        pygame.draw.circle(surface, (15, 24, 34), (cx, cy), radius)
        pygame.draw.circle(surface, (0, 180, 230), (cx, cy), radius, width=2)
        pygame.draw.circle(surface, (255, 210, 40), (cx, cy), int(radius * 0.56), width=1)
        pygame.draw.circle(surface, (255, 60, 60), (cx, cy), int(radius * 0.26), width=1)

        # Crosshairs
        pygame.draw.line(surface, (40, 70, 95), (cx - radius, cy), (cx + radius, cy), 1)
        pygame.draw.line(surface, (40, 70, 95), (cx, cy - radius), (cx, cy + radius), 1)

        # Plot contact dots
        for sec, dist in self.sector_distances.items():
            if dist < self.r_outer and self.sector_threats[sec] != 'CLEAR':
                ratio = dist / self.r_outer
                r_dist = int(ratio * (radius - 4))
                sec_angles = {
                    'FRONT': 0.0, 'FR': 0.78, 'RIGHT': 1.57, 'RR': 2.36,
                    'REAR': 3.14, 'RL': -2.36, 'LEFT': -1.57, 'FL': -0.78
                }
                ang = sec_angles.get(sec, 0.0)
                px = cx + math.sin(ang) * r_dist
                py = cy - math.cos(ang) * r_dist
                p_col = (255, 60, 60) if dist < self.r_inner else (255, 210, 40) if dist < self.r_mid else (0, 220, 255)
                pygame.draw.circle(surface, p_col, (int(px), int(py)), 3)

        # Center car marker
        pygame.draw.rect(surface, (0, 220, 255), (cx - 3, cy - 4, 6, 8), border_radius=2)
