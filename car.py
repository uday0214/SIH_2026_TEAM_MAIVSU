"""
Autonomous Player Car with pure pursuit trajectory tracking, dynamic speed control,
and procedural vehicle rendering with headlights and brake lights.
"""

import math
import pygame
from typing import List, Tuple
from config import (
    PLAYER_WIDTH, PLAYER_LENGTH,
    PLAYER_BASE_SPEED, PLAYER_MAX_SPEED, PLAYER_MIN_SPEED,
    PLAYER_ACCEL, PLAYER_DECEL, PLAYER_STEER_SPEED,
    PLANNER_REPLAN_INTERVAL
)

class AutonomousCar:
    def __init__(self, start_x: float, start_y: float):
        self.x = start_x
        self.y = start_y
        self.speed = 0.0 # Starts stationary, ramps up smoothly
        self.target_speed = PLAYER_BASE_SPEED
        self.heading = 0.0 # radians (0 is pointing up along -Y)
        self.steering_angle = 0.0
        
        # Dimensions
        self.width = PLAYER_WIDTH
        self.length = PLAYER_LENGTH

        # A* Path tracking
        self.path: List[Tuple[float, float]] = []
        self.target_waypoint: Tuple[float, float] = (start_x, start_y - 50)
        self.replan_timer = 0.0

        # State & Feedback
        self.pothole_bumps = 0
        self.collisions = 0
        self.bump_shake = 0.0
        self.is_braking = False

        # Visual styling
        self.body_color = (0, 150, 255)      # Vibrant electric blue
        self.cabin_color = (25, 30, 45)       # Dark tinted glass
        self.headlight_color = (255, 255, 210, 45) # Warm headlight beam
        self.honk_timer = 0.0

    def update(self, dt: float, road, obstacles, planner):
        # 1. Periodic A* Replanning
        self.replan_timer += dt
        if self.replan_timer >= PLANNER_REPLAN_INTERVAL or not self.path:
            self.replan_timer = 0.0
            
            # Extract active obstacles in lookahead range
            min_y = self.y - 450
            max_y = self.y + 60
            p_nearby, t_nearby = obstacles.get_obstacles_in_range(min_y, max_y)
            
            # Compute new A* path
            self.path = planner.plan_path(self.x, self.y, self.speed, p_nearby, t_nearby)

        # 2. Pure Pursuit Path Tracking
        # Find a lookahead target point on the path
        pursuit_dist = max(35.0, min(80.0, self.speed * 0.35))
        target_pt = self._find_pursuit_target(pursuit_dist)
        self.target_waypoint = target_pt

        # Calculate desired steering heading towards target
        dx = target_pt[0] - self.x
        dy = target_pt[1] - self.y # negative when ahead
        desired_heading = math.atan2(dx, -dy)

        # Smooth steering with rate limit
        angle_diff = (desired_heading - self.heading + math.pi) % (2 * math.pi) - math.pi
        max_turn = PLAYER_STEER_SPEED * dt
        self.steering_angle = max(-max_turn, min(max_turn, angle_diff))
        self.heading += self.steering_angle

        # 3. Dynamic Speed Adjustment
        # Slow down during sharp steering or when congested ahead
        curvature_factor = max(0.0, 1.0 - abs(angle_diff) * 1.5)
        desired_speed = PLAYER_MIN_SPEED + (self.target_speed - PLAYER_MIN_SPEED) * curvature_factor
        
        # Check clearance immediately ahead (emergency slow down if obstacle is tight)
        ahead_box_y = self.y - 45
        for t in obstacles.traffic:
            if 0 < (self.y - t.y) < 75 and abs(self.x - t.x) < 32:
                desired_speed = min(desired_speed, t.speed * 0.9)
                self.honk_timer = 0.35 # Honk horn when close behind traffic
                break

        if self.honk_timer > 0:
            self.honk_timer -= dt

        if desired_speed < self.speed:
            self.speed = max(desired_speed, self.speed - PLAYER_DECEL * dt)
            self.is_braking = True
        else:
            self.speed = min(desired_speed, self.speed + PLAYER_ACCEL * dt)
            self.is_braking = False

        # 4. Integrate Motion
        vx = math.sin(self.heading) * self.speed
        vy = -math.cos(self.heading) * self.speed
        self.x += vx * dt
        self.y += vy * dt

        # 5. Decay bump shake
        if self.bump_shake > 0:
            self.bump_shake = max(0.0, self.bump_shake - dt * 5.0)

        # 6. Obstacle collision detection
        # Check potholes
        for p in obstacles.potholes:
            if not getattr(p, 'hit', False) and abs(self.y - p.y) < 25 and abs(self.x - p.x) < 25:
                if p.contains_point(self.x, self.y):
                    p.hit = True
                    self.bump_shake = 1.0
                    self.pothole_bumps += 1
                    break

        # Check traffic collision
        for t in obstacles.traffic:
            if not getattr(t, 'hit', False) and abs(self.y - t.y) < (self.length + t.length) / 2 and abs(self.x - t.x) < (self.width + t.width) / 2:
                t.hit = True
                self.collisions += 1

    def _find_pursuit_target(self, lookahead: float) -> Tuple[float, float]:
        """Finds point on A* path ahead of vehicle by lookahead distance."""
        if not self.path:
            return (self.x, self.y - lookahead)

        # Iterate forward through path points
        for i in range(len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i+1]
            # Only consider points ahead of car
            if p2[1] < self.y:
                dist = math.hypot(p2[0] - self.x, p2[1] - self.y)
                if dist >= lookahead:
                    return p2

        return self.path[-1]

    def draw(self, surface: pygame.Surface, camera_y: float):
        sy = self.y - camera_y

        # Chassis shake effect when hitting bump
        shake_x = 0
        shake_y = 0
        if self.bump_shake > 0:
            shake_x = math.sin(pygame.time.get_ticks() * 0.08) * (self.bump_shake * 4.0)
            shake_y = math.cos(pygame.time.get_ticks() * 0.08) * (self.bump_shake * 4.0)

        draw_x = self.x + shake_x
        draw_y = sy + shake_y

        # Draw Headlights Light Beam (projected on road ahead)
        beam_length = 190
        beam_width = 85
        beam_surf = pygame.Surface((surface.get_width(), surface.get_height()), pygame.SRCALPHA)
        
        # Calculate headlight source and cone corners
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

        # Render Rotated Vehicle Sprite
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
