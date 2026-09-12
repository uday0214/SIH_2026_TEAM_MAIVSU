"""
Autonomous Player Car with pure pursuit trajectory tracking, realistic steering constraints,
dynamic speed control, and internal cognitive thought logging.
"""

import math
import pygame
from typing import List, Tuple
from config import (
    PLAYER_WIDTH, PLAYER_LENGTH,
    PLAYER_BASE_SPEED, PLAYER_MAX_SPEED, PLAYER_MIN_SPEED,
    PLAYER_ACCEL, PLAYER_DECEL, PLAYER_STEER_SPEED, MAX_STEER_DEVIATION,
    PLANNER_REPLAN_INTERVAL, POTHOLE_OBSTACLE_SPEED_THRESHOLD, REST_ACCEL_FACTOR
)
from obstacles import negotiate_deadlock_priority

class CircularDiscSensor:
    """
    Omnidirectional 360° Circular Disc Sensor Field.
    Provides immediate reflexive awareness, sector-based threat localization,
    reactive lateral repulsion away from flank hazards, and automated directional horn.
    """
    SECTORS = ['FRONT', 'FR', 'RIGHT', 'RR', 'REAR', 'RL', 'LEFT', 'FL']
    FORWARD_SECTORS = {'FRONT', 'FL', 'FR'}
    FLANK_SECTORS = {'LEFT', 'RIGHT'}
    REAR_SECTORS = {'REAR', 'RL', 'RR'}

    def __init__(self):
        self.r_inner = 46.0   # Critical Core (Emergency reflex stop)
        self.r_mid = 105.0    # Caution Buffer (Repulsive steering & easing)
        self.r_outer = 180.0  # Perception Disc (Scanning radar)

        self.sector_distances = {s: 180.0 for s in self.SECTORS}
        self.sector_threats = {s: 'CLEAR' for s in self.SECTORS}
        self.sector_points = {s: None for s in self.SECTORS}

        self.sweep_angle = 0.0
        self.pulse_phase = 0.0
        self.critical_breached = False
        self.repulsion_steer = 0.0
        self.nearest_dist = 180.0
        self.nearest_threat = 'CLEAR'

        # Directional Threat & Obstacle Tracking
        self.red_zone_tracks = []
        self.forward_hazard_present = False
        self.rear_hazard_present = False
        self.flank_hazard_present = False
        self.forward_blocker_info = None
        self.rear_blocker_info = None

    def update(self, car, road, obstacles, dt: float):
        self.sweep_angle = (self.sweep_angle + dt * 4.6) % (2 * math.pi)
        self.pulse_phase = (self.pulse_phase + dt * 2.6) % 1.0

        for s in self.SECTORS:
            self.sector_distances[s] = self.r_outer
            self.sector_threats[s] = 'CLEAR'
            self.sector_points[s] = None

        self.red_zone_tracks = []
        self.forward_hazard_present = False
        self.rear_hazard_present = False
        self.flank_hazard_present = False
        self.forward_blocker_info = None
        self.rear_blocker_info = None

        min_y = car.y - self.r_outer - 25.0
        max_y = car.y + self.r_outer + 25.0
        p_nearby, t_nearby, ped_nearby, cow_nearby = obstacles.get_obstacles_in_range(min_y, max_y)

        # Collect candidate proximity contact points with dynamic vehicle clearance & pothole obstacle status
        contacts = []

        for t in t_nearby:
            vtype = getattr(t, 'vtype', 'CAR')
            t_margin = 20.0 if vtype in ('TRUCK', 'BUS') else (13.0 if vtype == 'CAR' else (8.0 if vtype == 'AUTO' else 5.0))
            contacts.append((t.x, t.y, f"TRAFFIC_{vtype}", t.width * 0.5 + t_margin, t))
        for ped in ped_nearby:
            contacts.append((ped.x, ped.y, 'PEDESTRIAN', ped.radius, ped))
        for cow in cow_nearby:
            contacts.append((cow.x, cow.y, 'COW', cow.radius, cow))
        for p in p_nearby:
            # Potholes above crawl threshold (> 6-7 km/h, ~23.5 px/s) are pure obstacles in radar
            p_is_obstacle = (car.speed > POTHOLE_OBSTACLE_SPEED_THRESHOLD)
            p_type = 'POTHOLE_OBSTACLE' if p_is_obstacle else 'POTHOLE_CRAWL'
            contacts.append((p.x, p.y, p_type, p.effective_radius + 4.0, p))

        # Road boundaries
        for sample_y in [car.y - 75.0, car.y, car.y + 75.0]:
            left, right, _, _ = road.get_road_edges(sample_y)
            contacts.append((left, sample_y, 'ROAD_EDGE', 6.0, None))
            contacts.append((right, sample_y, 'ROAD_EDGE', 6.0, None))

        nearest_d = self.r_outer
        nearest_t = 'CLEAR'

        sin_h = math.sin(car.heading)
        cos_h = math.cos(car.heading)

        for ox, oy, otype, rad, obj_ref in contacts:
            dx = ox - car.x
            dy = oy - car.y
            raw_dist = math.hypot(dx, dy)
            eff_dist = max(0.0, raw_dist - rad)

            if eff_dist < self.r_outer:
                # Calculate relative bearing angle relative to car heading
                abs_ang = math.atan2(dx, -dy)
                rel_ang = (abs_ang - car.heading + math.pi) % (2 * math.pi) - math.pi
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

                if eff_dist < nearest_d:
                    nearest_d = eff_dist
                    nearest_t = otype

                # Directional Red Zone Analysis:
                # Projects obstacle location onto vehicle heading and movement axis.
                # Potholes above crawl speed are now included as critical obstacles!
                if eff_dist < self.r_inner and otype not in ['CLEAR', 'POTHOLE_CRAWL', 'ROAD_EDGE']:
                    # d_fwd: positive in front of vehicle heading, negative behind
                    # d_lat: positive to right of vehicle heading, negative to left
                    d_fwd = dx * sin_h - dy * cos_h
                    d_lat = dx * cos_h + dy * sin_h

                    # Forward trajectory corridor overlap check
                    in_forward_cone = sec in self.FORWARD_SECTORS
                    lateral_path_overlap = abs(d_lat) < (car.width * 0.5 + rad + 6.0)

                    # Determine how this obstacle affects our direction of travel
                    is_forward_hazard = (d_fwd > -2.0) and (in_forward_cone or lateral_path_overlap)
                    is_rear_hazard = (sec in self.REAR_SECTORS) or (d_fwd <= -2.0)
                    is_flank_hazard = (sec in self.FLANK_SECTORS) and not is_forward_hazard and not is_rear_hazard

                    track_info = {
                        'sector': sec,
                        'type': otype,
                        'dist': eff_dist,
                        'd_fwd': d_fwd,
                        'd_lat': d_lat,
                        'is_forward_hazard': is_forward_hazard,
                        'is_rear_hazard': is_rear_hazard,
                        'is_flank_hazard': is_flank_hazard,
                        'point': (ox, oy),
                        'ref': obj_ref
                    }
                    self.red_zone_tracks.append(track_info)

                    if is_forward_hazard:
                        self.forward_hazard_present = True
                        if self.forward_blocker_info is None or eff_dist < self.forward_blocker_info['dist']:
                            self.forward_blocker_info = track_info
                    elif is_rear_hazard:
                        self.rear_hazard_present = True
                        if self.rear_blocker_info is None or eff_dist < self.rear_blocker_info['dist']:
                            self.rear_blocker_info = track_info
                    elif is_flank_hazard:
                        self.flank_hazard_present = True

        self.nearest_dist = nearest_d
        self.nearest_threat = nearest_t

        # 1. Critical core breach check:
        # ONLY triggers emergency halt when an obstacle is an actual obstruction in our forward path!
        # Obstacles trailing behind us in the rear red zone NEVER trigger an emergency stop.
        self.critical_breached = self.forward_hazard_present

        # 2. Reactive lateral repulsion away from flank hazards
        d_left = min(self.sector_distances['LEFT'], self.sector_distances['FL'], self.sector_distances['RL'])
        d_right = min(self.sector_distances['RIGHT'], self.sector_distances['FR'], self.sector_distances['RR'])

        rep_left = max(0.0, (self.r_mid - d_left) / self.r_mid) if d_left < self.r_mid else 0.0
        rep_right = max(0.0, (self.r_mid - d_right) / self.r_mid) if d_right < self.r_mid else 0.0
        # Positive repulsion steers right; negative steers left
        self.repulsion_steer = (rep_left - rep_right) * 0.40

        # 3. Automated directional horn trigger (alerts cows and pedestrians to clear out of the way)
        front_d = min(self.sector_distances['FRONT'], self.sector_distances['FL'], self.sector_distances['FR'])
        front_threat = self.sector_threats['FRONT']
        if ('COW' in front_threat or 'PEDESTRIAN' in front_threat) and front_d < 145.0:
            if car.honk_timer <= 0.05:
                car.honk_timer = 0.45
                if 'COW' in front_threat:
                    car.add_thought("SENSOR DISC: Bovine obstacle in front sector; pulsing horn.", "WARN")
                else:
                    car.add_thought("SENSOR DISC: Pedestrian proximity breach; sounding horn warning.", "WARN")

    def draw_world(self, surface: pygame.Surface, car, camera_y: float):
        sy = car.y - camera_y
        h = surface.get_height()
        if sy < -self.r_outer or sy > h + self.r_outer:
            return

        cx = int(car.x)
        cy = int(sy)

        disc_size = int(self.r_outer * 2 + 10)
        disc_surf = pygame.Surface((disc_size, disc_size), pygame.SRCALPHA)
        dcx = disc_size // 2
        dcy = disc_size // 2

        # 1. Outer Perception Disc (Transparent Cyan Field)
        pygame.draw.circle(disc_surf, (0, 225, 255, 18), (dcx, dcy), int(self.r_outer))
        pygame.draw.circle(disc_surf, (0, 220, 255, 60), (dcx, dcy), int(self.r_outer), width=1)

        # 2. Caution Buffer Disc (Soft Amber)
        pygame.draw.circle(disc_surf, (255, 200, 40, 22), (dcx, dcy), int(self.r_mid))
        pygame.draw.circle(disc_surf, (255, 200, 40, 75), (dcx, dcy), int(self.r_mid), width=1)

        # 3. Critical Core Safety Bubble (Red alert if forward hazard, amber if rear follower)
        if self.forward_hazard_present:
            core_alpha = 110
            core_col = (255, 45, 45)
            core_width = 2
        elif self.rear_hazard_present:
            core_alpha = 65
            core_col = (255, 160, 30)
            core_width = 1
        elif self.flank_hazard_present:
            core_alpha = 55
            core_col = (255, 200, 40)
            core_width = 1
        else:
            core_alpha = 25
            core_col = (255, 50, 50)
            core_width = 1

        pygame.draw.circle(disc_surf, (*core_col, core_alpha), (dcx, dcy), int(self.r_inner))
        pygame.draw.circle(disc_surf, (*core_col, 140 if (self.forward_hazard_present or self.rear_hazard_present) else 60),
                           (dcx, dcy), int(self.r_inner), width=core_width)

        # 4. Pulsing Wave Ring
        pulse_r = int(self.r_inner + self.pulse_phase * (self.r_outer - self.r_inner))
        pulse_alpha = int(70 * (1.0 - self.pulse_phase))
        pygame.draw.circle(disc_surf, (0, 240, 255, pulse_alpha), (dcx, dcy), pulse_r, width=1)

        # 5. Rotating Radar Sweep Beam
        beam_len = self.r_outer - 4
        sw_x = dcx + math.sin(self.sweep_angle) * beam_len
        sw_y = dcy - math.cos(self.sweep_angle) * beam_len
        pygame.draw.line(disc_surf, (180, 255, 255, 130), (dcx, dcy), (int(sw_x), int(sw_y)), 2)

        # Trailing sweep glow
        trail_angle = self.sweep_angle - 0.25
        tr_x = dcx + math.sin(trail_angle) * beam_len * 0.95
        tr_y = dcy - math.cos(trail_angle) * beam_len * 0.95
        pygame.draw.polygon(disc_surf, (0, 230, 255, 30), [(dcx, dcy), (int(sw_x), int(sw_y)), (int(tr_x), int(tr_y))])

        # 6. Sector Contact Blips
        for sec, pt in self.sector_points.items():
            if pt is not None and self.sector_distances[sec] < self.r_outer:
                bdx = pt[0] - car.x
                bdy = pt[1] - car.y
                bx = int(dcx + bdx)
                by = int(dcy + bdy)
                if 0 <= bx < disc_size and 0 <= by < disc_size:
                    dist = self.sector_distances[sec]
                    b_col = (255, 60, 60) if dist < self.r_inner else (255, 200, 40) if dist < self.r_mid else (0, 235, 255)
                    pygame.draw.circle(disc_surf, b_col, (bx, by), 4)
                    pygame.draw.circle(disc_surf, (255, 255, 255), (bx, by), 6, width=1)

        surface.blit(disc_surf, (cx - dcx, cy - dcy))

    def draw_radar_hud(self, surface: pygame.Surface, cx: int, cy: int, radius: int = 54):
        """Draws a circular 360° radar display for the AI sidebar dashboard."""
        pygame.draw.circle(surface, (15, 24, 34), (cx, cy), radius)
        pygame.draw.circle(surface, (0, 180, 230), (cx, cy), radius, width=2)
        pygame.draw.circle(surface, (255, 200, 40), (cx, cy), int(radius * 0.60), width=1)
        pygame.draw.circle(surface, (255, 60, 60), (cx, cy), int(radius * 0.28), width=1)

        # Crosshairs
        pygame.draw.line(surface, (40, 70, 95), (cx - radius, cy), (cx + radius, cy), 1)
        pygame.draw.line(surface, (40, 70, 95), (cx, cy - radius), (cx, cy + radius), 1)

        # Sweeping radar line
        sw_x = cx + math.sin(self.sweep_angle) * (radius - 2)
        sw_y = cy - math.cos(self.sweep_angle) * (radius - 2)
        pygame.draw.line(surface, (0, 240, 255), (cx, cy), (int(sw_x), int(sw_y)), 2)

        # Plot contact dots
        for sec, dist in self.sector_distances.items():
            if dist < self.r_outer and self.sector_threats[sec] != 'CLEAR':
                ratio = dist / self.r_outer
                r_dist = int(ratio * (radius - 4))
                # Approximate bearing of sector
                sec_angles = {
                    'FRONT': 0.0, 'FR': 0.78, 'RIGHT': 1.57, 'RR': 2.36,
                    'REAR': 3.14, 'RL': -2.36, 'LEFT': -1.57, 'FL': -0.78
                }
                ang = sec_angles.get(sec, 0.0)
                px = cx + math.sin(ang) * r_dist
                py = cy - math.cos(ang) * r_dist
                p_col = (255, 60, 60) if dist < self.r_inner else (255, 200, 40) if dist < self.r_mid else (0, 220, 255)
                pygame.draw.circle(surface, p_col, (int(px), int(py)), 3)

        # Car icon in center
        pygame.draw.rect(surface, (0, 220, 255), (cx - 3, cy - 5, 6, 10), border_radius=2)


class AutonomousCar:
    def __init__(self, start_x: float, start_y: float, start_heading: float = 0.0):
        self.x = start_x
        self.y = start_y
        self.speed = 0.0
        self.target_speed = PLAYER_BASE_SPEED
        self.heading = start_heading # Aligned with road tangent
        self.steering_angle = 0.0
        
        # Dimensions
        self.width = PLAYER_WIDTH
        self.length = PLAYER_LENGTH

        # 360° Circular Disc Sensor Field
        self.sensor = CircularDiscSensor()

        # A* Path tracking
        self.path: List[Tuple[float, float]] = []
        self.target_waypoint: Tuple[float, float] = (start_x, start_y - 60)
        self.replan_timer = 0.0
        self.sim_time = 0.0

        # Auto Mode & Speed Adaptation
        self.auto_mode = True
        self.auto_speed_reason = "CRUISING"

        # AI Cognitive Dashboard: Internal Thoughts & Observations
        self.thoughts_log = [
            ("00:00", "Sensors calibrated. Circular Disc Sensor active.", "SYS"),
            ("00:01", "Cruising speed engaged; scanning road boundaries and hazards.", "INFO"),
        ]
        self.last_thought_time = 0.0
        self.last_thought_text = ""
        self.observations = {
            "road_width": 320,
            "road_curve": 0.0,
            "threat": "CLEAR",
            "steer_pct": 0,
            "throttle_pct": 0,
            "brake_pct": 0,
            "safety_margin": 98,
            "sensor_dist": 180,
            "sensor_threat": "CLEAR",
            "critical_breached": False,
            "traffic_regime": "EMPTY",
            "traffic_count": 0
        }

        # State & Feedback
        self.pothole_bumps = 0
        self.collisions = 0
        self.pedestrian_bumps = 0
        self.cow_bumps = 0
        self.bump_shake = 0.0
        self.is_braking = False

        # Visual styling
        self.body_color = (0, 150, 255)      # Electric blue
        self.cabin_color = (25, 30, 45)       # Tinted glass
        self.headlight_color = (255, 255, 210, 45)
        self.honk_timer = 0.0

    def add_thought(self, text: str, tag: str = "DECISION"):
        """Records an internal thought with timestamp, deduplicating consecutive thoughts."""
        if text == self.last_thought_text and (self.sim_time - self.last_thought_time) < 2.0:
            return
        self.last_thought_text = text
        self.last_thought_time = self.sim_time

        mins = int(self.sim_time // 60)
        secs = int(self.sim_time % 60)
        time_str = f"{mins:02d}:{secs:02d}"

        self.thoughts_log.append((time_str, text, tag))
        if len(self.thoughts_log) > 10:
            self.thoughts_log.pop(0)

    def update(self, dt: float, road, obstacles, planner):
        self.sim_time += dt

        # 1. Periodic A* Replanning
        self.replan_timer += dt
        min_y = self.y - 480
        max_y = self.y + 60
        p_nearby, t_nearby, ped_nearby, cow_nearby = obstacles.get_obstacles_in_range(min_y, max_y)

        # 360° Circular Disc Sensor update
        self.sensor.update(self, road, obstacles, dt)

        if self.replan_timer >= PLANNER_REPLAN_INTERVAL or not self.path:
            self.replan_timer = 0.0
            self.path = planner.plan_path(self.x, self.y, self.speed, p_nearby, t_nearby, ped_nearby, cow_nearby)

        # 2. Pure Pursuit Path Tracking with Realistic Steering Constraints
        # Lookahead distance scales smoothly to ensure gentle highway cornering
        pursuit_dist = max(75.0, min(145.0, 50.0 + self.speed * 0.45))
        target_pt = self._find_pursuit_target(pursuit_dist)
        self.target_waypoint = target_pt

        dx = target_pt[0] - self.x
        dy = target_pt[1] - self.y
        desired_heading = math.atan2(dx, -dy)

        # Reduced turning rate and steering inertia (Realistic heavy vehicle steering)
        # Apply circular disc lateral repulsion steering for reactive reflex avoidance
        angle_diff = (desired_heading - self.heading + math.pi) % (2 * math.pi) - math.pi
        target_steer = max(-PLAYER_STEER_SPEED, min(PLAYER_STEER_SPEED, angle_diff * 1.8 + self.sensor.repulsion_steer))
        self.steering_angle += (target_steer - self.steering_angle) * min(1.0, 3.8 * dt)

        # Vehicle kinematics constraint: No in-place turning when stationary!
        # Heading rate scales with forward linear velocity. When speed <= 1.0 px/s, heading cannot turn.
        if self.speed > 1.0:
            speed_factor = min(1.0, self.speed / 40.0)
            self.heading += self.steering_angle * speed_factor * dt
        else:
            self.steering_angle = 0.0

        # Realistic Steering Constraint:
        # Strictly clamp maximum off-axis heading relative to the road tangent
        road_tangent = road.get_tangent_angle(self.y)
        dev = (self.heading - road_tangent + math.pi) % (2 * math.pi) - math.pi
        if dev > MAX_STEER_DEVIATION:
            self.heading = road_tangent + MAX_STEER_DEVIATION
            self.steering_angle = min(0.0, self.steering_angle)
        elif dev < -MAX_STEER_DEVIATION:
            self.heading = road_tangent - MAX_STEER_DEVIATION
            self.steering_angle = max(0.0, self.steering_angle)

        # 3. Traffic Density Sensing & Adaptive Speed Allowance
        rw = road.get_road_width(self.y)
        active_threat = "CLEAR"

        # Sense surrounding and forward traffic around the vehicle
        traffic_ahead_count = 0
        traffic_surround_count = 0
        traffic_congestion_score = 0.0

        for t in t_nearby:
            t_dy = self.y - t.y   # positive = ahead, negative = behind
            t_dx = abs(self.x - t.x)
            t_dist = math.hypot(t_dx, t_dy)
            if -100.0 <= t_dy <= 340.0 and t_dx <= rw * 0.65:
                traffic_surround_count += 1
                if 0.0 < t_dy <= 300.0:
                    traffic_ahead_count += 1
                if t_dist < 280.0:
                    traffic_congestion_score += max(0.0, 1.0 - (t_dist / 280.0))

        # Classify Traffic Density Regime:
        # - HEAVY: Dense traffic -> Low speed priority (defensive headway 95 - 120 px/s)
        # - MODERATE: Medium traffic -> Paced speed (145 - 180 px/s)
        # - LIGHT: Sparse traffic -> Standard cruise (185 - 230 px/s)
        # - EMPTY: Empty road -> Higher speed allowance (up to PLAYER_MAX_SPEED 290 px/s / ~81 km/h)
        if traffic_congestion_score >= 1.6 or traffic_surround_count >= 3 or traffic_ahead_count >= 2:
            traffic_regime = "HEAVY"
            traffic_speed_allowance = 115.0  # ~32 km/h (Low speed priority)
            traffic_reason = "HEAVY TRAFFIC (LOW SPEED PRIORITY)"
        elif traffic_congestion_score >= 0.7 or traffic_surround_count >= 2:
            traffic_regime = "MODERATE"
            traffic_speed_allowance = 175.0  # ~49 km/h (Paced speed)
            traffic_reason = "MODERATE TRAFFIC (PACED SPEED)"
        elif traffic_congestion_score > 0.15 or traffic_surround_count == 1:
            traffic_regime = "LIGHT"
            traffic_speed_allowance = 230.0  # ~64 km/h (Cruising)
            traffic_reason = "LIGHT TRAFFIC (FLOWING)"
        else:
            traffic_regime = "EMPTY"
            traffic_speed_allowance = PLAYER_MAX_SPEED  # 290 px/s (~81 km/h Higher speed allowance!)
            traffic_reason = "EMPTY ROAD (HIGH SPEED ALLOWANCE)"

        if self.auto_mode:
            width_ratio = max(0.0, min(1.0, (rw - 210.0) / 190.0))
            if traffic_regime == "EMPTY":
                base_auto_speed = 220.0 + width_ratio * 70.0  # 220 - 290 px/s (~62 - 81 km/h)
                reason = traffic_reason
            elif traffic_regime == "HEAVY":
                base_auto_speed = min(traffic_speed_allowance, 95.0 + width_ratio * 25.0)  # 95 - 120 px/s (~27 - 34 km/h)
                reason = traffic_reason
                self.add_thought("Dense traffic detected; prioritizing low speed and defensive headway.", "DECISION")
            elif traffic_regime == "MODERATE":
                base_auto_speed = min(traffic_speed_allowance, 145.0 + width_ratio * 35.0)
                reason = traffic_reason
            else: # LIGHT
                base_auto_speed = min(traffic_speed_allowance, 185.0 + width_ratio * 45.0)
                reason = traffic_reason

            # Road bottleneck cognition
            if rw < 240:
                self.add_thought(f"Road pinched to {int(rw)}px bottleneck. Easing throttle.", "ALERT")

            # Curvature lookahead
            future_tangent = road.get_tangent_angle(self.y - 140)
            curr_tangent = road.get_tangent_angle(self.y)
            curve_severity = abs((future_tangent - curr_tangent + math.pi) % (2 * math.pi) - math.pi)
            if curve_severity > 0.30:
                curve_slow = max(0.62, 1.0 - (curve_severity - 0.30) * 1.5)
                base_auto_speed *= curve_slow
                reason = "SHARP CURVE AHEAD"
                active_threat = "CURVATURE"
                self.add_thought("Sharp curve detected ahead; pre-braking for smooth cornering.", "DECISION")

            # Pedestrian hazard cognition
            for ped in ped_nearby:
                p_dy = self.y - ped.y
                p_dx = abs(self.x - ped.x)
                if 0 < p_dy < 130 and p_dx < 50:
                    base_auto_speed = min(base_auto_speed, 65.0)
                    self.honk_timer = 0.3
                    reason = "BRAKING FOR PEDESTRIAN"
                    active_threat = "PEDESTRIAN"
                    self.add_thought(f"Pedestrian at {int(p_dy/8)}m stepping out! Braking and honking.", "WARN")
                    break

            # Traffic vehicle hazard cognition
            for t in t_nearby:
                t_dy = self.y - t.y
                t_dx = abs(self.x - t.x)
                if 0 < t_dy < 95 and t_dx < 36:
                    base_auto_speed = min(base_auto_speed, t.speed * 0.90)
                    self.honk_timer = 0.35
                    reason = f"FOLLOWING {t.vtype}"
                    active_threat = f"{t.vtype}_AHEAD"
                    self.add_thought(f"Behind slow {t.vtype} ({int(t.speed*0.28)} km/h). Seeking overtake lane.", "DECISION")
                    break

            # Cow hazard cognition (predict behavior and adjust trajectory early from far off)
            for cow in cow_nearby:
                c_dy = self.y - cow.y
                c_dx = abs(self.x - cow.x)
                if 0 < c_dy < 200:
                    if getattr(cow, 'is_resting', False):
                        if c_dx < (self.width * 0.5 + cow.width * 0.5 + 10.0):
                            base_auto_speed = min(base_auto_speed, 54.0)
                            self.honk_timer = 0.35
                            reason = "SLOWING (RESTING COW)"
                            active_threat = "RESTING_COW"
                            self.add_thought("Sacred cow resting in path; slowing to steer around.", "DECISION")
                        elif c_dx < (self.width * 0.5 + cow.width * 0.5 + 32.0):
                            base_auto_speed = min(base_auto_speed, 110.0)
                            reason = "PASSING COW (CAUTIOUS)"
                            active_threat = "RESTING_COW"
                            self.add_thought("Passing resting cow cautiously in open lane.", "INFO")
                    else:
                        if c_dx < (self.width * 0.5 + cow.width * 0.5 + 16.0):
                            base_auto_speed = min(base_auto_speed, 85.0)
                            reason = "CAUTION (COW HERD)"
                            active_threat = "COW_HERD"
                            self.add_thought("Bovine herd grazing on shoulder; holding safe clearance.", "INFO")
                    break

            self.target_speed = base_auto_speed
            self.auto_speed_reason = reason
        else:
            self.auto_speed_reason = "MANUAL SPEED"

        # 4. Dynamic Speed Control Execution (Capable of coming to complete rest)
        curvature_factor = max(0.50, 1.0 - abs(angle_diff) * 0.70)
        effective_desired = self.target_speed * curvature_factor

        # Close proximity collision override with complete stop at rest
        for t in t_nearby:
            dy = self.y - t.y
            dx = abs(self.x - t.x)
            if 0 < dy < 95 and dx < (self.width + t.width) * 0.62:
                min_gap = (self.length + t.length) * 0.5 + 24.0
                if dy < min_gap:
                    effective_desired = 0.0 # Full stop at rest!
                    self.add_thought(f"Blocked by {t.vtype} ahead. Vehicle coming to full stop.", "WARN")
                else:
                    gap_factor = max(0.0, min(1.0, (dy - min_gap) / 45.0))
                    effective_desired = min(effective_desired, t.speed * gap_factor)
                self.honk_timer = 0.35
                break

        for ped in ped_nearby:
            p_dy = self.y - ped.y
            p_dx = abs(self.x - ped.x)
            if 0 < p_dy < 85 and p_dx < (self.width * 0.5 + ped.radius + 18.0):
                is_ped_moving = getattr(ped, 'is_moving', True)
                has_row = getattr(ped, 'has_right_of_way', False)
                is_ped_standing = getattr(ped, 'is_standing', False)

                if is_ped_moving or has_row:
                    effective_desired = 0.0
                    self.add_thought("Yielding right-of-way to crossing pedestrian.", "DECISION")
                elif is_ped_standing and getattr(ped, 'stand_timer', 0.0) > 0.8:
                    effective_desired = min(effective_desired, 44.0)
                    self.add_thought("Pedestrian holding position; proceeding past cautiously.", "DECISION")
                else:
                    if p_dy < 42.0:
                        effective_desired = 0.0
                    else:
                        effective_desired = min(effective_desired, 25.0)
                self.honk_timer = 0.35
                break

        # Bovine close-proximity stop (only stop if car directly overlaps cow in lane)
        for cow in cow_nearby:
            c_dy = self.y - cow.y
            c_dx = abs(self.x - cow.x)
            if 0 < c_dy < 80 and c_dx < (self.width * 0.5 + cow.width * 0.5 + 6.0):
                if getattr(cow, 'is_resting', False):
                    if c_dy < 48.0:
                        effective_desired = 0.0 # Full stop before resting cow!
                        self.add_thought("Resting cow blocking lane ahead. Holding full stop.", "WARN")
                    else:
                        effective_desired = min(effective_desired, 32.0)
                else:
                    if c_dy < 38.0:
                        effective_desired = 0.0
                    else:
                        effective_desired = min(effective_desired, 45.0)
                break

        # Pothole crossing: At speeds > 6-7 km/h, unavoidable potholes require decelerating to crawl speed (<= 22 px/s)
        for p in p_nearby:
            p_dy = self.y - p.y
            p_dx = abs(self.x - p.x)
            if 0 < p_dy < 95 and p_dx < (p.effective_radius + self.width * 0.5 + 4.0):
                crawl_spd = 22.0  # <= 6-7 km/h (~6.2 km/h)
                if effective_desired > crawl_spd:
                    effective_desired = crawl_spd
                    if self.auto_mode:
                        self.auto_speed_reason = "POTHOLE CRAWL (<=7 km/h)"
                    self.add_thought("Approaching unavoidable pothole crater; reducing to <=7 km/h crawl.", "DECISION")
                break

        # Circular Disc Sensor Threat Assessment & Inter-Vehicle Deadlock Negotiation:
        # Analyzes obstacle location, threat type, and performs principled deadlock resolution.
        if self.sensor.critical_breached and self.sensor.forward_hazard_present:
            info = self.sensor.forward_blocker_info
            threat_type = info['type'] if info else "OBSTACLE"
            sec = info['sector'] if info else "FRONT"
            obj_ref = info.get('ref') if info else None

            if threat_type == 'POTHOLE_OBSTACLE':
                # Approaching a pothole obstacle at speed: decelerate to crawl threshold (<= 22 px/s)
                effective_desired = min(effective_desired, 22.0)
                if self.auto_mode:
                    self.auto_speed_reason = "POTHOLE CRAWL (<=7 km/h)"
                self.add_thought("SENSOR RED ZONE: Pothole obstacle in forward path; slowing to <=7 km/h crawl.", "WARN")
            elif threat_type.startswith("TRAFFIC_") and obj_ref is not None:
                # Inter-vehicle deadlock negotiation
                if self.speed < 25.0 and getattr(obj_ref, 'speed', 0.0) < 25.0:
                    has_priority, reason = negotiate_deadlock_priority(self, obj_ref, road)
                    if has_priority:
                        effective_desired = 26.0  # Creep forward out of deadlock
                        if self.auto_mode:
                            self.auto_speed_reason = f"DEADLOCK: {reason[:16]}"
                        self.add_thought(f"DEADLOCK RESOLUTION: Priority acquired ({reason}); creeping forward to clear jam.", "DECISION")
                    else:
                        effective_desired = 0.0  # Yield to smaller/higher priority vehicle
                        if self.auto_mode:
                            self.auto_speed_reason = f"YIELD: {reason[:16]}"
                        self.add_thought(f"DEADLOCK RESOLUTION: Yielding ({reason}); holding position for clearance.", "INFO")
                else:
                    effective_desired = 0.0
                    if self.auto_mode:
                        self.auto_speed_reason = f"SENSOR STOP ({threat_type} {sec})"
                    self.add_thought(f"SENSOR DISC: Forward path blocked by {threat_type} in {sec}! Emergency stop.", "ALERT")
            else:
                effective_desired = 0.0
                if self.auto_mode:
                    self.auto_speed_reason = f"SENSOR STOP ({threat_type} {sec})"
                self.add_thought(f"SENSOR DISC: Forward path blocked by {threat_type} in {sec}! Emergency stop.", "ALERT")
        elif self.sensor.rear_hazard_present:
            # Obstacle is BEHIND us in the rear red zone.
            # We must NOT stop, as stopping creates a deadlock or invites a rear-end collision!
            info = self.sensor.rear_blocker_info
            threat_type = info['type'] if info else "TRAFFIC"
            # If path ahead is clear (effective_desired > 0), maintain forward cruising momentum away from tailgater
            if effective_desired > 0.0:
                effective_desired = max(effective_desired, 95.0)
                if self.auto_mode and not self.auto_speed_reason.startswith("FOLLOWING"):
                    self.auto_speed_reason = f"EVADING REAR {threat_type}"
                self.add_thought(f"SENSOR DISC: {threat_type} trailing in rear red zone; holding forward motion to open gap.", "DECISION")
        elif self.sensor.flank_hazard_present:
            # Flank obstacle alongside; lateral repulsion steering already provides separation
            if effective_desired > 75.0:
                effective_desired = min(effective_desired, 75.0)
            self.add_thought("SENSOR DISC: Flank obstacle alongside; maintaining lateral clearance.", "INFO")

        if self.honk_timer > 0:
            self.honk_timer -= dt

        if effective_desired < self.speed:
            self.speed = max(effective_desired, self.speed - PLAYER_DECEL * dt)
            self.is_braking = True
        else:
            # Acceleration from rest reduced by 20% to prevent rapid launch
            rest_scale = REST_ACCEL_FACTOR if self.speed < 40.0 else (REST_ACCEL_FACTOR + (1.0 - REST_ACCEL_FACTOR) * min(1.0, (self.speed - 40.0) / 40.0))
            effective_accel = PLAYER_ACCEL * rest_scale
            self.speed = min(effective_desired, self.speed + effective_accel * dt)
            self.is_braking = False

        self.speed = max(0.0, self.speed) # Fully stop capable

        # 5. Integrate Motion
        vx = math.sin(self.heading) * self.speed
        vy = -math.cos(self.heading) * self.speed
        self.x += vx * dt
        self.y += vy * dt

        # Strict road boundary clamping for player vehicle
        left, right, _, rw = road.get_road_edges(self.y)
        safe_left = left + self.width * 0.65
        safe_right = right - self.width * 0.65
        self.x = max(safe_left, min(safe_right, self.x))

        # 6. Decay bump shake
        if self.bump_shake > 0:
            self.bump_shake = max(0.0, self.bump_shake - dt * 5.0)

        # 7. Collision Detection & Feedback Thoughts
        # Potholes
        for p in obstacles.potholes:
            if not getattr(p, 'hit', False) and abs(self.y - p.y) < 25 and abs(self.x - p.x) < 25:
                if p.contains_point(self.x, self.y):
                    p.hit = True
                    # Speed-scaled suspension impact: 40% harsher at high speed, 10% lower when crawling
                    if self.speed > 180.0:
                        self.bump_shake = 1.40  # +40% high speed penalty impact
                        self.speed = max(0.0, self.speed - 35.0)  # Momentum loss from rim strike
                        self.add_thought("High-speed pothole impact! Severe suspension shock (+40% penalty).", "ALERT")
                    elif self.speed <= 80.0:
                        self.bump_shake = 0.90  # 10% lower penalty for crawling
                        self.add_thought("Traversed pothole crater at reduced crawl speed (-10% penalty).", "INFO")
                    else:
                        self.bump_shake = 1.0
                        self.add_thought("Hit pothole crater! Suspension absorbing shock.", "WARN")
                    self.pothole_bumps += 1
                    break

        # Dynamic Traffic
        for t in obstacles.traffic:
            if not getattr(t, 'hit', False) and abs(self.y - t.y) < (self.length + t.length) / 2 and abs(self.x - t.x) < (self.width + t.width) / 2:
                t.hit = True
                self.collisions += 1
                self.add_thought(f"Impact with {t.vtype}! Recalibrating spatial margin.", "ALERT")

        # Pedestrians
        for ped in obstacles.pedestrians:
            if not getattr(ped, 'hit', False) and abs(self.y - ped.y) < 22 and abs(self.x - ped.x) < 18:
                ped.hit = True
                self.pedestrian_bumps += 1
                self.add_thought("Pedestrian contact warning!", "ALERT")

        # Indian Bovines (Cows)
        for cow in getattr(obstacles, 'cows', []):
            if not getattr(cow, 'hit', False) and abs(self.y - cow.y) < 26 and abs(self.x - cow.x) < 22:
                cow.hit = True
                self.cow_bumps += 1
                self.add_thought("Contact with bovine obstacle!", "ALERT")

        # 8. Update Observations Telemetry
        self.observations["road_width"] = int(rw)
        self.observations["road_curve"] = round(math.degrees(road_tangent), 1)
        self.observations["threat"] = active_threat
        self.observations["steer_pct"] = int((self.steering_angle / PLAYER_STEER_SPEED) * 100)
        self.observations["throttle_pct"] = int((self.speed / PLAYER_MAX_SPEED) * 100) if not self.is_braking else 15
        self.observations["brake_pct"] = 85 if self.is_braking else 0
        safety_calc = 100 - len(p_nearby) * 5 - len(t_nearby) * 5 - len(ped_nearby) * 7 - len(cow_nearby) * 6
        self.observations["safety_margin"] = max(40, min(99, safety_calc))
        self.observations["sensor_dist"] = int(self.sensor.nearest_dist)
        self.observations["sensor_threat"] = self.sensor.nearest_threat
        self.observations["sensor_status"] = "ALERT" if self.sensor.critical_breached else ("CAUTION" if self.sensor.nearest_dist < self.sensor.r_mid else "CLEAR")
        self.observations["critical_breached"] = self.sensor.critical_breached
        self.observations["traffic_regime"] = traffic_regime
        self.observations["traffic_count"] = traffic_surround_count

    def _find_pursuit_target(self, lookahead: float) -> Tuple[float, float]:
        """Finds point on A* path ahead of vehicle by lookahead distance."""
        if not self.path:
            return (self.x, self.y - lookahead)

        for i in range(len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i+1]
            if p2[1] < self.y:
                dist = math.hypot(p2[0] - self.x, p2[1] - self.y)
                if dist >= lookahead:
                    return p2

        return self.path[-1]

    def draw(self, surface: pygame.Surface, camera_y: float, show_radar: bool = True):
        sy = self.y - camera_y

        shake_x = 0
        shake_y = 0
        if self.bump_shake > 0:
            shake_x = math.sin(pygame.time.get_ticks() * 0.08) * (self.bump_shake * 4.0)
            shake_y = math.cos(pygame.time.get_ticks() * 0.08) * (self.bump_shake * 4.0)

        draw_x = self.x + shake_x
        draw_y = sy + shake_y

        # Draw 360° Circular Disc Sensor Field on Road if enabled
        if show_radar:
            self.sensor.draw_world(surface, self, camera_y)

        # Headlight beam projection
        beam_length = 190
        beam_width = 85
        beam_surf = pygame.Surface((surface.get_width(), surface.get_height()), pygame.SRCALPHA)
        
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)

        front_cx = draw_x + sin_h * (self.length / 2)
        front_cy = draw_y - cos_h * (self.length / 2)

        p_left = (
            front_cx + sin_h * beam_length - cos_h * beam_width,
            front_cy - cos_h * beam_length - sin_h * beam_width
        )
        p_right = (
            front_cx + sin_h * beam_length + cos_h * beam_width,
            front_cy - cos_h * beam_length + sin_h * beam_width
        )

        pygame.draw.polygon(beam_surf, self.headlight_color, [
            (front_cx - cos_h * 8, front_cy - sin_h * 8),
            p_left,
            p_right,
            (front_cx + cos_h * 8, front_cy + sin_h * 8)
        ])
        surface.blit(beam_surf, (0, 0))

        # Vehicle Sprite
        veh_size = int(self.length * 1.6)
        car_surf = pygame.Surface((veh_size, veh_size), pygame.SRCALPHA)
        cx = veh_size // 2
        cy = veh_size // 2
        hw = self.width // 2
        hl = self.length // 2

        # 1. Wheels
        wheel_color = (25, 25, 25)
        for wx in [-hw - 1, hw - 3]:
            for wy in [-hl + 6, hl - 14]:
                pygame.draw.rect(car_surf, wheel_color, (cx + wx, cy + wy, 4, 10), border_radius=2)

        # 2. Main Body Chassis
        body_rect = pygame.Rect(cx - hw, cy - hl, self.width, self.length)
        pygame.draw.rect(car_surf, (15, 20, 25), body_rect, border_radius=5)
        pygame.draw.rect(car_surf, self.body_color, body_rect.inflate(-2, -2), border_radius=4)

        # 3. Cabin & Windshield
        cabin_rect = pygame.Rect(cx - hw + 3, cy - hl + 14, self.width - 6, self.length - 28)
        pygame.draw.rect(car_surf, self.cabin_color, cabin_rect, border_radius=4)
        
        # Front windshield
        pygame.draw.line(car_surf, (160, 220, 255), (cx - hw + 4, cy - hl + 14), (cx + hw - 4, cy - hl + 14), 3)
        # Rear window
        pygame.draw.line(car_surf, (120, 180, 220), (cx - hw + 4, cy + hl - 14), (cx + hw - 4, cy + hl - 14), 2)

        # 4. Brake lights / Tail lights
        tail_color = (255, 30, 20) if self.is_braking else (160, 20, 15)
        pygame.draw.circle(car_surf, tail_color, (cx - hw + 4, cy + hl - 2), 3)
        pygame.draw.circle(car_surf, tail_color, (cx + hw - 4, cy + hl - 2), 3)

        # 5. Headlights
        pygame.draw.circle(car_surf, (255, 255, 200), (cx - hw + 4, cy - hl + 2), 3)
        pygame.draw.circle(car_surf, (255, 255, 200), (cx + hw - 4, cy - hl + 2), 3)

        # Rotate car to heading angle
        angle_deg = -math.degrees(self.heading)
        rotated = pygame.transform.rotate(car_surf, angle_deg)
        rot_rect = rotated.get_rect(center=(int(draw_x), int(draw_y)))
        surface.blit(rotated, rot_rect)

        # 6. Visual Honk Soundwaves
        if self.honk_timer > 0:
            honk_surf = pygame.Surface((surface.get_width(), surface.get_height()), pygame.SRCALPHA)
            horn_front_x = draw_x + sin_h * (self.length / 2 + 10)
            horn_front_y = draw_y - cos_h * (self.length / 2 + 10)
            for r in [12, 22, 32]:
                pygame.draw.circle(honk_surf, (255, 220, 60, 140), (int(horn_front_x), int(horn_front_y)), r, width=2)
            surface.blit(honk_surf, (0, 0))
