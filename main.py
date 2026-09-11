"""
Main entry point for Indian Highway Autonomous Driving Simulation with A* Pathfinding.
Pure Pygame implementation with zero external dependencies or backend.
"""

import sys
import os
import math
import random
import pygame

from config import (
    SCREEN_WIDTH, SCREEN_HEIGHT, FPS, TITLE,
    COLOR_TEXT, COLOR_HUD_BG, COLOR_PATH_LINE, COLOR_PATH_WAYPOINT,
    COLOR_GRID_EXPLORED, COLOR_GRID_OBSTACLE
)
from road import InfiniteRoad
from obstacles import ObstacleManager
from planner import AStarPlanner
from car import AutonomousCar


class IndianHighwaySimulation:
    def __init__(self, headless: bool = False):
        pygame.init()
        pygame.font.init()

        self.headless = headless
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption(TITLE)

        self.clock = pygame.time.Clock()
        self.font_large = pygame.font.SysFont("monospace", 17, bold=True)
        self.font_medium = pygame.font.SysFont("monospace", 13, bold=True)
        self.font_small = pygame.font.SysFont("monospace", 11)

        # Simulation state
        self.paused = False
        self.show_debug = True
        self.show_dashboard = True  # Toggleable AI Thoughts & Decision Dashboard
        self.roadside_props = []

        self.reset()

    def reset(self):
        """Initializes or resets all simulation components."""
        self.road = InfiniteRoad()
        self.obstacles = ObstacleManager(self.road)
        self.planner = AStarPlanner(self.road)

        start_y = 0.0
        start_x = self.road.get_road_center(start_y)
        self.car = AutonomousCar(start_x, start_y)

        self.camera_y = start_y - SCREEN_HEIGHT * 0.72

        self.roadside_props = []
        for prop_y in range(-2000, 1000, 140):
            self._spawn_roadside_prop(prop_y)

    def _spawn_roadside_prop(self, wy: float):
        left, right, cx, _ = self.road.get_road_edges(wy)
        side = random.choice([-1, 1])
        if side == -1:
            px = left - random.uniform(50, 130)
        else:
            px = right + random.uniform(50, 130)

        ptype = random.choices(['BUSH', 'TREE', 'MILESTONE'], weights=[0.55, 0.35, 0.10])[0]
        size = random.uniform(14, 28)
        self.roadside_props.append({
            'x': px,
            'y': wy,
            'type': ptype,
            'size': size,
            'color': (random.randint(28, 45), random.randint(65, 95), random.randint(25, 40))
        })

    def handle_events(self) -> bool:
        """Processes user input events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_d:
                    self.show_debug = not self.show_debug
                elif event.key in (pygame.K_TAB, pygame.K_t):
                    self.show_dashboard = not self.show_dashboard
                elif event.key == pygame.K_a:
                    self.car.auto_mode = not self.car.auto_mode
                elif event.key == pygame.K_r:
                    self.reset()
                elif event.key == pygame.K_UP:
                    self.car.target_speed = min(350.0, self.car.target_speed + 25.0)
                elif event.key == pygame.K_DOWN:
                    self.car.target_speed = max(100.0, self.car.target_speed - 25.0)

        return True

    def update(self, dt: float):
        if self.paused:
            return

        dt = min(dt, 0.05)

        # 1. Update obstacles, traffic (Truck, Bus, Car, Auto, Bike), and pedestrians
        self.obstacles.update(dt, self.car.y)

        # 2. Update player car (A* tracking, realistic steering constraints, thoughts)
        self.car.update(dt, self.road, self.obstacles, self.planner)

        # 3. Smooth Camera Tracking
        target_cam_y = self.car.y - SCREEN_HEIGHT * 0.72
        self.camera_y += (target_cam_y - self.camera_y) * min(1.0, 8.0 * dt)

        # 4. Roadside Props update
        min_prop_y = min(p['y'] for p in self.roadside_props) if self.roadside_props else self.camera_y
        if min_prop_y > self.car.y - 1200:
            for new_y in range(int(min_prop_y) - 140, int(self.car.y - 1400), -140):
                self._spawn_roadside_prop(new_y)

        self.roadside_props = [p for p in self.roadside_props if p['y'] < self.camera_y + SCREEN_HEIGHT + 200]

    def draw_roadside_props(self):
        for prop in self.roadside_props:
            sy = prop['y'] - self.camera_y
            if -40 <= sy <= SCREEN_HEIGHT + 40:
                px = int(prop['x'])
                sy = int(sy)
                ptype = prop['type']
                size = int(prop['size'])

                if ptype == 'MILESTONE':
                    mw, mh = 14, 22
                    base_rect = pygame.Rect(px - mw // 2, sy - mh // 2, mw, mh)
                    pygame.draw.rect(self.screen, (240, 240, 235), base_rect, border_top_left_radius=7, border_top_right_radius=7)
                    top_rect = pygame.Rect(px - mw // 2, sy - mh // 2, mw, 8)
                    pygame.draw.rect(self.screen, (240, 190, 20), top_rect, border_top_left_radius=7, border_top_right_radius=7)
                    pygame.draw.rect(self.screen, (50, 50, 50), base_rect, width=1, border_top_left_radius=7, border_top_right_radius=7)
                elif ptype == 'TREE':
                    pygame.draw.circle(self.screen, (35, 60, 25), (px, sy), size)
                    pygame.draw.circle(self.screen, prop['color'], (px - 2, sy - 2), int(size * 0.85))
                else: # BUSH
                    pygame.draw.circle(self.screen, prop['color'], (px, sy), size)
                    pygame.draw.circle(self.screen, (40, 75, 30), (px + 3, sy - 2), int(size * 0.6))

    def draw_debug_overlay(self):
        if not self.show_debug:
            return

        debug_surf = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)

        for ex, ey in self.planner.last_explored_cells:
            esy = ey - self.camera_y
            if 0 <= esy <= SCREEN_HEIGHT:
                pygame.draw.circle(debug_surf, COLOR_GRID_EXPLORED, (int(ex), int(esy)), 3)

        cell = self.planner.cell_size
        for ox, oy in self.planner.last_obstacle_cells:
            osy = oy - self.camera_y
            if 0 <= osy <= SCREEN_HEIGHT:
                pygame.draw.rect(debug_surf, COLOR_GRID_OBSTACLE, (int(ox - cell/2), int(osy - cell/2), cell, cell))

        path = self.car.path
        if len(path) >= 2:
            screen_pts = [(int(px), int(py - self.camera_y)) for px, py in path]
            pygame.draw.lines(debug_surf, (0, 200, 255, 90), False, screen_pts, 6)
            pygame.draw.lines(debug_surf, (180, 255, 255, 220), False, screen_pts, 3)

            for idx, pt in enumerate(screen_pts):
                if idx % 3 == 0 or idx == len(screen_pts) - 1:
                    pygame.draw.circle(debug_surf, COLOR_PATH_WAYPOINT, pt, 4)

        tx, ty = self.car.target_waypoint
        tsy = ty - self.camera_y
        if 0 <= tsy <= SCREEN_HEIGHT:
            pygame.draw.circle(debug_surf, (255, 80, 80), (int(tx), int(tsy)), 6, width=2)
            pygame.draw.line(debug_surf, (255, 100, 100, 160),
                             (int(self.car.x), int(self.car.y - self.camera_y)),
                             (int(tx), int(tsy)), 1)

        self.screen.blit(debug_surf, (0, 0))

    def draw_hud(self):
        """Top-left Telemetry Card."""
        hud_w, hud_h = 390, 210
        hud_surf = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(hud_surf, (14, 18, 24, 220), (0, 0, hud_w, hud_h), border_radius=10)
        pygame.draw.rect(hud_surf, (0, 180, 230, 120), (0, 0, hud_w, hud_h), width=2, border_radius=10)

        speed_kmh = int(self.car.speed * 0.28)
        target_kmh = int(self.car.target_speed * 0.28)
        dist_m = int(-self.car.y / 8.0)
        fps = int(self.clock.get_fps())
        rw = int(self.road.get_road_width(self.car.y))
        rw_status = " (Bottleneck)" if rw < 235 else " (Wide)" if rw > 370 else ""

        auto_str = f"AUTO: {self.car.auto_speed_reason}" if self.car.auto_mode else "MANUAL SPEED"
        auto_col = (80, 245, 160) if self.car.auto_mode else (255, 180, 80)

        lines = [
            ("AUTONOMOUS NAVIGATOR (A*)", (0, 220, 255), self.font_large),
            (f"Speed: {speed_kmh} km/h (Target: {target_kmh})  |  FPS: {fps}", (255, 255, 255), self.font_medium),
            (f"Mode [A]: {auto_str}", auto_col, self.font_medium),
            (f"Road Width: {rw} px{rw_status}  |  Dist: {dist_m} m", (210, 225, 235), self.font_medium),
            (f"A* Nodes Explored: {self.planner.nodes_explored_count} cells", (255, 220, 80), self.font_medium),
            (f"Potholes: {self.obstacles.potholes_avoided} dodged | Hits: {self.car.pothole_bumps}",
             (120, 255, 140) if self.car.pothole_bumps == 0 else (255, 160, 80), self.font_medium),
            (f"Traffic: {self.obstacles.traffic_overtaken} overtaken | Hits: {self.car.collisions}",
             (160, 220, 255) if self.car.collisions == 0 else (255, 120, 100), self.font_medium),
            (f"Pedestrians: {self.obstacles.pedestrians_avoided} dodged | Hits: {self.car.pedestrian_bumps}",
             (255, 210, 140) if self.car.pedestrian_bumps == 0 else (255, 80, 80), self.font_medium),
        ]

        y_offset = 10
        for text, color, font in lines:
            t_surf = font.render(text, True, color)
            hud_surf.blit(t_surf, (14, y_offset))
            y_offset += t_surf.get_height() + 3

        self.screen.blit(hud_surf, (16, 16))

        # Bottom Controls Banner
        ctrl_w, ctrl_h = 740, 34
        ctrl_surf = pygame.Surface((ctrl_w, ctrl_h), pygame.SRCALPHA)
        pygame.draw.rect(ctrl_surf, (15, 20, 26, 215), (0, 0, ctrl_w, ctrl_h), border_radius=6)
        ctrl_text = "[TAB] AI Thoughts Dashboard  |  [A] Auto Speed  |  [D] A* Debug  |  [SPACE] Pause  |  [R] Reset"
        t_ctrl = self.font_small.render(ctrl_text, True, (210, 225, 240))
        ctrl_surf.blit(t_ctrl, (16, 10))
        self.screen.blit(ctrl_surf, (SCREEN_WIDTH // 2 - ctrl_w // 2, SCREEN_HEIGHT - 44))

        if self.paused:
            pause_surf = self.font_large.render("-- SIMULATION PAUSED --", True, (255, 220, 40))
            px = SCREEN_WIDTH // 2 - pause_surf.get_width() // 2
            py = 35
            self.screen.blit(pause_surf, (px, py))

    def draw_dashboard(self):
        """Draws the toggleable AI Thoughts & Observation Dashboard on the right."""
        if not self.show_dashboard:
            return

        dw, dh = 370, 520
        dx = SCREEN_WIDTH - dw - 16
        dy = 16

        dash_surf = pygame.Surface((dw, dh), pygame.SRCALPHA)
        pygame.draw.rect(dash_surf, (10, 15, 22, 235), (0, 0, dw, dh), border_radius=10)
        pygame.draw.rect(dash_surf, (0, 190, 240, 140), (0, 0, dw, dh), width=2, border_radius=10)

        # Header
        t_title = self.font_large.render("AI COGNITIVE DASHBOARD", True, (0, 235, 255))
        t_sub = self.font_small.render("[TAB / T] Toggle  |  A* Neural Observation Log", True, (160, 195, 220))
        dash_surf.blit(t_title, (14, 12))
        dash_surf.blit(t_sub, (14, 34))

        pygame.draw.line(dash_surf, (40, 70, 95), (14, 52), (dw - 14, 52), 1)

        # Perception & Telemetry
        obs = self.car.observations
        y_cur = 60

        sec_title = self.font_medium.render("PERCEPTION & ACTUATORS", True, (255, 215, 60))
        dash_surf.blit(sec_title, (14, y_cur))
        y_cur += 20

        threat_color = (80, 240, 120) if obs["threat"] == "CLEAR" else (255, 170, 50) if "AHEAD" in obs["threat"] else (255, 80, 80)
        t_threat = self.font_small.render(f"Focus Threat: {obs['threat']}", True, threat_color)
        dash_surf.blit(t_threat, (14, y_cur))
        
        t_rw = self.font_small.render(f"Road Width: {obs['road_width']}px", True, (210, 225, 240))
        dash_surf.blit(t_rw, (210, y_cur))
        y_cur += 20

        # Safety Margin Bar
        margin = obs["safety_margin"]
        t_margin = self.font_small.render(f"Safety Margin: {margin}%", True, (200, 220, 235))
        dash_surf.blit(t_margin, (14, y_cur))
        bar_w = 140
        bar_h = 9
        pygame.draw.rect(dash_surf, (30, 40, 50), (210, y_cur + 2, bar_w, bar_h), border_radius=3)
        fill_w = int((margin / 100.0) * bar_w)
        margin_col = (80, 230, 130) if margin > 70 else (240, 180, 40)
        pygame.draw.rect(dash_surf, margin_col, (210, y_cur + 2, fill_w, bar_h), border_radius=3)
        y_cur += 22

        # Steering Intent Bar
        steer_pct = obs["steer_pct"]
        steer_dir = "CENTER" if abs(steer_pct) < 5 else ("LEFT" if steer_pct < 0 else "RIGHT")
        t_steer = self.font_small.render(f"Steer: {steer_pct}% ({steer_dir})", True, (200, 220, 235))
        dash_surf.blit(t_steer, (14, y_cur))
        s_bar_w = 140
        pygame.draw.rect(dash_surf, (30, 40, 50), (210, y_cur + 2, s_bar_w, bar_h), border_radius=3)
        cx_bar = 210 + s_bar_w // 2
        pygame.draw.line(dash_surf, (150, 150, 160), (cx_bar, y_cur), (cx_bar, y_cur + bar_h + 2), 1)
        s_fill = int((steer_pct / 100.0) * (s_bar_w // 2))
        if s_fill < 0:
            pygame.draw.rect(dash_surf, (0, 220, 255), (cx_bar + s_fill, y_cur + 2, -s_fill, bar_h), border_radius=2)
        elif s_fill > 0:
            pygame.draw.rect(dash_surf, (0, 220, 255), (cx_bar, y_cur + 2, s_fill, bar_h), border_radius=2)
        y_cur += 22

        # Throttle & Brake
        th_pct = obs["throttle_pct"]
        brk_pct = obs["brake_pct"]
        t_pwr = self.font_small.render(f"Throttle/Brake:", True, (200, 220, 235))
        dash_surf.blit(t_pwr, (14, y_cur))
        # Throttle (Green)
        pygame.draw.rect(dash_surf, (30, 40, 50), (210, y_cur + 2, 70, bar_h), border_radius=3)
        pygame.draw.rect(dash_surf, (50, 220, 100), (210, y_cur + 2, int(70 * (th_pct/100.0)), bar_h), border_radius=3)
        # Brake (Red)
        pygame.draw.rect(dash_surf, (30, 40, 50), (286, y_cur + 2, 64, bar_h), border_radius=3)
        if brk_pct > 0:
            pygame.draw.rect(dash_surf, (255, 60, 50), (286, y_cur + 2, int(64 * (brk_pct/100.0)), bar_h), border_radius=3)
        y_cur += 28

        pygame.draw.line(dash_surf, (40, 70, 95), (14, y_cur), (dw - 14, y_cur), 1)
        y_cur += 10

        # Decision Making Stream
        dec_title = self.font_medium.render("INTERNAL DELIBERATION STREAM", True, (255, 215, 60))
        dash_surf.blit(dec_title, (14, y_cur))
        y_cur += 20

        for time_str, text, tag in self.car.thoughts_log[-5:]:
            if tag == "ALERT":
                tag_col = (255, 100, 80)
            elif tag == "WARN":
                tag_col = (255, 210, 50)
            elif tag == "DECISION":
                tag_col = (0, 225, 255)
            else:
                tag_col = (170, 185, 200)

            t_hdr = self.font_small.render(f"[{time_str}] [{tag}]", True, tag_col)
            dash_surf.blit(t_hdr, (14, y_cur))
            y_cur += 14

            if len(text) > 44:
                t1 = self.font_small.render(text[:42] + "-", True, (220, 230, 240))
                t2 = self.font_small.render("  " + text[42:], True, (220, 230, 240))
                dash_surf.blit(t1, (20, y_cur))
                y_cur += 13
                dash_surf.blit(t2, (20, y_cur))
                y_cur += 16
            else:
                t1 = self.font_small.render(text, True, (220, 230, 240))
                dash_surf.blit(t1, (20, y_cur))
                y_cur += 16

        self.screen.blit(dash_surf, (dx, dy))

    def run(self, max_frames: int = None):
        running = True
        frames = 0

        while running:
            dt = self.clock.tick(FPS) / 1000.0

            running = self.handle_events()
            self.update(dt)

            self.road.draw(self.screen, self.camera_y)
            self.draw_roadside_props()
            self.obstacles.draw(self.screen, self.camera_y)
            self.car.draw(self.screen, self.camera_y)
            self.draw_debug_overlay()
            self.draw_hud()
            self.draw_dashboard()

            pygame.display.flip()

            frames += 1
            if max_frames and frames >= max_frames:
                break

        pygame.quit()


if __name__ == "__main__":
    is_headless = "--headless" in sys.argv or os.environ.get("SDL_VIDEODRIVER") == "dummy"
    sim = IndianHighwaySimulation(headless=is_headless)
    
    max_f = None
    for arg in sys.argv:
        if arg.startswith("--frames="):
            max_f = int(arg.split("=")[1])

    sim.run(max_frames=max_f)
