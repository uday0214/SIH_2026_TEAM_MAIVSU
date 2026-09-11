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
        self.font_large = pygame.font.SysFont("monospace", 19, bold=True)
        self.font_medium = pygame.font.SysFont("monospace", 14, bold=True)
        self.font_small = pygame.font.SysFont("monospace", 12)

        # Simulation state
        self.paused = False
        self.show_debug = True
        self.roadside_props = []

        self.reset()

    def reset(self):
        """Initializes or resets all simulation components."""
        self.road = InfiniteRoad()
        self.obstacles = ObstacleManager(self.road)
        self.planner = AStarPlanner(self.road)

        # Place car at starting position on the road
        start_y = 0.0
        start_x = self.road.get_road_center(start_y)
        self.car = AutonomousCar(start_x, start_y)

        # Camera
        self.camera_y = start_y - SCREEN_HEIGHT * 0.72

        # Generate roadside props (milestones, bushes, dusty shrubs)
        self.roadside_props = []
        for prop_y in range(-2000, 1000, 140):
            self._spawn_roadside_prop(prop_y)

    def _spawn_roadside_prop(self, wy: float):
        """Creates roadside trees, bushes, and Indian highway milestone markers."""
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
        """Processes user input events. Returns False if simulation should exit."""
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

        # Cap dt to prevent tunneling on frame lag
        dt = min(dt, 0.05)

        # 1. Update obstacles and procedural generation
        self.obstacles.update(dt, self.car.y)

        # 2. Update player car (A* tracking, physics, steering)
        self.car.update(dt, self.road, self.obstacles, self.planner)

        # 3. Smooth Camera Tracking
        target_cam_y = self.car.y - SCREEN_HEIGHT * 0.72
        self.camera_y += (target_cam_y - self.camera_y) * min(1.0, 8.0 * dt)

        # 4. Roadside Props update
        min_prop_y = min(p['y'] for p in self.roadside_props) if self.roadside_props else self.camera_y
        if min_prop_y > self.car.y - 1200:
            for new_y in range(int(min_prop_y) - 140, int(self.car.y - 1400), -140):
                self._spawn_roadside_prop(new_y)

        # Prune old props
        self.roadside_props = [p for p in self.roadside_props if p['y'] < self.camera_y + SCREEN_HEIGHT + 200]

    def draw_roadside_props(self):
        """Renders roadside vegetation and yellow/white Indian NH milestones."""
        for prop in self.roadside_props:
            sy = prop['y'] - self.camera_y
            if -40 <= sy <= SCREEN_HEIGHT + 40:
                px = int(prop['x'])
                sy = int(sy)
                ptype = prop['type']
                size = int(prop['size'])

                if ptype == 'MILESTONE':
                    # Traditional Indian rounded milestone (Yellow top, White bottom)
                    mw, mh = 14, 22
                    base_rect = pygame.Rect(px - mw // 2, sy - mh // 2, mw, mh)
                    pygame.draw.rect(self.screen, (240, 240, 235), base_rect, border_top_left_radius=7, border_top_right_radius=7)
                    top_rect = pygame.Rect(px - mw // 2, sy - mh // 2, mw, 8)
                    pygame.draw.rect(self.screen, (240, 190, 20), top_rect, border_top_left_radius=7, border_top_right_radius=7)
                    pygame.draw.rect(self.screen, (50, 50, 50), base_rect, width=1, border_top_left_radius=7, border_top_right_radius=7)
                elif ptype == 'TREE':
                    # Layered leafy canopy
                    pygame.draw.circle(self.screen, (35, 60, 25), (px, sy), size)
                    pygame.draw.circle(self.screen, prop['color'], (px - 2, sy - 2), int(size * 0.85))
                else: # BUSH
                    pygame.draw.circle(self.screen, prop['color'], (px, sy), size)
                    pygame.draw.circle(self.screen, (40, 75, 30), (px + 3, sy - 2), int(size * 0.6))

    def draw_debug_overlay(self):
        """Draws the A* search tree, obstacle grid, and planned trajectory."""
        if not self.show_debug:
            return

        debug_surf = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)

        # 1. Draw Explored Nodes (Cyan faint dots)
        for ex, ey in self.planner.last_explored_cells:
            esy = ey - self.camera_y
            if 0 <= esy <= SCREEN_HEIGHT:
                pygame.draw.circle(debug_surf, COLOR_GRID_EXPLORED, (int(ex), int(esy)), 3)

        # 2. Draw Impassable / Hazard Grid Cells (Red faint squares)
        cell = self.planner.cell_size
        for ox, oy in self.planner.last_obstacle_cells:
            osy = oy - self.camera_y
            if 0 <= osy <= SCREEN_HEIGHT:
                pygame.draw.rect(debug_surf, COLOR_GRID_OBSTACLE, (int(ox - cell/2), int(osy - cell/2), cell, cell))

        # 3. Draw Active A* Planned Trajectory (Glowing Neon Ribbon)
        path = self.car.path
        if len(path) >= 2:
            screen_pts = [(int(px), int(py - self.camera_y)) for px, py in path]
            # Outer glow
            pygame.draw.lines(debug_surf, (0, 200, 255, 90), False, screen_pts, 6)
            # Core bright path
            pygame.draw.lines(debug_surf, (180, 255, 255, 220), False, screen_pts, 3)

            # Draw Waypoint Nodes
            for idx, pt in enumerate(screen_pts):
                if idx % 3 == 0 or idx == len(screen_pts) - 1:
                    pygame.draw.circle(debug_surf, COLOR_PATH_WAYPOINT, pt, 4)

        # 4. Draw Pure Pursuit Target Waypoint
        tx, ty = self.car.target_waypoint
        tsy = ty - self.camera_y
        if 0 <= tsy <= SCREEN_HEIGHT:
            pygame.draw.circle(debug_surf, (255, 80, 80), (int(tx), int(tsy)), 6, width=2)
            pygame.draw.line(debug_surf, (255, 100, 100, 160),
                             (int(self.car.x), int(self.car.y - self.camera_y)),
                             (int(tx), int(tsy)), 1)

        self.screen.blit(debug_surf, (0, 0))

    def draw_hud(self):
        """Draws telemetry and interactive controls HUD."""
        # Top-left HUD card
        hud_w, hud_h = 360, 175
        hud_surf = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(hud_surf, COLOR_HUD_BG, (0, 0, 0, 0)) # reset
        pygame.draw.rect(hud_surf, (15, 20, 26, 215), (0, 0, hud_w, hud_h), border_radius=10)
        pygame.draw.rect(hud_surf, (0, 180, 230, 120), (0, 0, hud_w, hud_h), width=2, border_radius=10)

        # Telemetry metrics
        speed_kmh = int(self.car.speed * 0.28) # scaled conversion
        dist_m = int(-self.car.y / 8.0)
        fps = int(self.clock.get_fps())

        lines = [
            ("AUTONOMOUS NAVIGATOR (A*)", (0, 220, 255), self.font_large),
            (f"Speed: {speed_kmh} km/h (Target: {int(self.car.target_speed * 0.28)})", (255, 255, 255), self.font_medium),
            (f"Distance: {dist_m} m  |  FPS: {fps}", (220, 220, 220), self.font_medium),
            (f"A* Nodes Explored: {self.planner.nodes_explored_count} cells", (255, 220, 80), self.font_medium),
            (f"Potholes Dodged: {self.obstacles.potholes_avoided}  | Hits: {self.car.pothole_bumps}",
             (100, 255, 120) if self.car.pothole_bumps == 0 else (255, 160, 80), self.font_medium),
            (f"Traffic Overtaken: {self.obstacles.traffic_overtaken}", (160, 220, 255), self.font_medium),
        ]

        y_offset = 12
        for text, color, font in lines:
            t_surf = font.render(text, True, color)
            hud_surf.blit(t_surf, (16, y_offset))
            y_offset += t_surf.get_height() + 4

        self.screen.blit(hud_surf, (16, 16))

        # Bottom Controls Banner
        ctrl_w, ctrl_h = 560, 34
        ctrl_surf = pygame.Surface((ctrl_w, ctrl_h), pygame.SRCALPHA)
        pygame.draw.rect(ctrl_surf, (15, 20, 26, 200), (0, 0, ctrl_w, ctrl_h), border_radius=6)
        ctrl_text = "[D] A* Overlay  |  [SPACE] Pause  |  [↑/↓] Speed  |  [R] Reset  |  [ESC] Exit"
        t_ctrl = self.font_small.render(ctrl_text, True, (200, 215, 230))
        ctrl_surf.blit(t_ctrl, (16, 10))
        self.screen.blit(ctrl_surf, (SCREEN_WIDTH // 2 - ctrl_w // 2, SCREEN_HEIGHT - 46))

        # Paused banner
        if self.paused:
            pause_surf = self.font_large.render("-- SIMULATION PAUSED --", True, (255, 220, 40))
            px = SCREEN_WIDTH // 2 - pause_surf.get_width() // 2
            py = 35
            self.screen.blit(pause_surf, (px, py))

    def run(self, max_frames: int = None):
        """Main game and simulation loop."""
        running = True
        frames = 0

        while running:
            dt = self.clock.tick(FPS) / 1000.0

            running = self.handle_events()
            self.update(dt)

            # Render pass
            self.road.draw(self.screen, self.camera_y)
            self.draw_roadside_props()
            self.obstacles.draw(self.screen, self.camera_y)
            self.car.draw(self.screen, self.camera_y)
            self.draw_debug_overlay()
            self.draw_hud()

            pygame.display.flip()

            frames += 1
            if max_frames and frames >= max_frames:
                break

        pygame.quit()


if __name__ == "__main__":
    is_headless = "--headless" in sys.argv or os.environ.get("SDL_VIDEODRIVER") == "dummy"
    sim = IndianHighwaySimulation(headless=is_headless)
    
    # If headless arg with frame count, run test
    max_f = None
    for arg in sys.argv:
        if arg.startswith("--frames="):
            max_f = int(arg.split("=")[1])

    sim.run(max_frames=max_f)
