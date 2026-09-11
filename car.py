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
    PLANNER_REPLAN_INTERVAL
)

class AutonomousCar:
    def __init__(self, start_x: float, start_y: float):
        self.x = start_x
        self.y = start_y
        self.speed = 0.0
        self.target_speed = PLAYER_BASE_SPEED
        self.heading = 0.0 # radians (0 is pointing up along -Y)
        self.steering_angle = 0.0
        
        # Dimensions
        self.width = PLAYER_WIDTH
        self.length = PLAYER_LENGTH

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
            ("00:00", "Sensors calibrated. A* planning lattice active.", "SYS"),
            ("00:01", "Cruising speed engaged; scanning road boundaries.", "INFO"),
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
            "safety_margin": 98
        }

        # State & Feedback
        self.pothole_bumps = 0
        self.collisions = 0
        self.pedestrian_bumps = 0
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
        min_y = self.y - 450
        max_y = self.y + 60
        p_nearby, t_nearby, ped_nearby = obstacles.get_obstacles_in_range(min_y, max_y)

        if self.replan_timer >= PLANNER_REPLAN_INTERVAL or not self.path:
            self.replan_timer = 0.0
            self.path = planner.plan_path(self.x, self.y, self.speed, p_nearby, t_nearby, ped_nearby)

        # 2. Pure Pursuit Path Tracking with Realistic Steering Constraints
        # Longer pursuit lookahead ensures gentle, realistic highway steering
        pursuit_dist = max(65.0, min(125.0, self.speed * 0.52))
        target_pt = self._find_pursuit_target(pursuit_dist)
        self.target_waypoint = target_pt

        dx = target_pt[0] - self.x
        dy = target_pt[1] - self.y
        desired_heading = math.atan2(dx, -dy)

        # Reduced turning rate and steering inertia (Realistic heavy vehicle steering)
        angle_diff = (desired_heading - self.heading + math.pi) % (2 * math.pi) - math.pi
        target_steer = max(-PLAYER_STEER_SPEED, min(PLAYER_STEER_SPEED, angle_diff * 1.8))
        self.steering_angle += (target_steer - self.steering_angle) * min(1.0, 3.8 * dt)
        self.heading += self.steering_angle * dt

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

        # 3. Auto Speed Mode Adaptation & Deliberation
        rw = road.get_road_width(self.y)
        active_threat = "CLEAR"

        if self.auto_mode:
            width_ratio = max(0.0, min(1.0, (rw - 210.0) / 190.0))
            base_auto_speed = PLAYER_MIN_SPEED + 80.0 + width_ratio * 125.0
            reason = "CRUISING (WIDE ROAD)" if width_ratio > 0.55 else "CHOKE POINT (NARROW)"

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

            self.target_speed = base_auto_speed
            self.auto_speed_reason = reason
        else:
            self.auto_speed_reason = "MANUAL SPEED"

        # 4. Dynamic Speed Control Execution (Capable of coming to complete rest)
        curvature_factor = max(0.0, 1.0 - abs(angle_diff) * 1.4)
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
            if 0 < p_dy < 80 and p_dx < 36:
                if p_dy < 42.0:
                    effective_desired = 0.0 # Full stop for pedestrian!
                    self.add_thought("Pedestrian directly in path! Emergency stop at rest.", "ALERT")
                else:
                    effective_desired = min(effective_desired, 25.0)
                self.honk_timer = 0.35
                break

        if self.honk_timer > 0:
            self.honk_timer -= dt

        if effective_desired < self.speed:
            self.speed = max(effective_desired, self.speed - PLAYER_DECEL * dt)
            self.is_braking = True
        else:
            self.speed = min(effective_desired, self.speed + PLAYER_ACCEL * dt)
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
                    self.bump_shake = 1.0
                    self.pothole_bumps += 1
                    self.add_thought("Hit pothole crater! Suspension absorbing shock.", "WARN")
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

        # 8. Update Observations Telemetry
        self.observations["road_width"] = int(rw)
        self.observations["road_curve"] = round(math.degrees(road_tangent), 1)
        self.observations["threat"] = active_threat
        self.observations["steer_pct"] = int((self.steering_angle / PLAYER_STEER_SPEED) * 100)
        self.observations["throttle_pct"] = int((self.speed / PLAYER_MAX_SPEED) * 100) if not self.is_braking else 15
        self.observations["brake_pct"] = 85 if self.is_braking else 0
        safety_calc = 100 - len(p_nearby) * 5 - len(t_nearby) * 6 - len(ped_nearby) * 8
        self.observations["safety_margin"] = max(40, min(99, safety_calc))

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

    def draw(self, surface: pygame.Surface, camera_y: float):
        sy = self.y - camera_y

        shake_x = 0
        shake_y = 0
        if self.bump_shake > 0:
            shake_x = math.sin(pygame.time.get_ticks() * 0.08) * (self.bump_shake * 4.0)
            shake_y = math.cos(pygame.time.get_ticks() * 0.08) * (self.bump_shake * 4.0)

        draw_x = self.x + shake_x
        draw_y = sy + shake_y

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
