"""
Procedural infinite road generator with unstructured, organic Indian road characteristics.
"""

import math
import pygame
from config import (
    SCREEN_WIDTH, SCREEN_HEIGHT,
    COLOR_BG_GRASS, COLOR_DIRT_SHOULDER, COLOR_ASPHALT,
    COLOR_ASPHALT_PATCH, COLOR_ROAD_MARKING,
    ROAD_BASE_WIDTH, SHOULDER_WIDTH
)

class InfiniteRoad:
    def __init__(self):
        # Base road parameters
        self.base_cx = SCREEN_WIDTH / 2.0
        
        # Procedural patches cache (indexed by chunk)
        self.chunk_size = 600
        self.patches = {}

    def get_road_center(self, world_y: float) -> float:
        """Returns the road centerline x-coordinate at world_y."""
        u = -world_y
        # Multi-frequency organic curves
        offset = (
            150.0 * math.sin(u * 0.0016) +
            75.0 * math.sin(u * 0.0038 + 1.3) +
            30.0 * math.sin(u * 0.0085 + 0.4)
        )
        return self.base_cx + offset

    def get_road_width(self, world_y: float) -> float:
        """Returns the road width at world_y (varies organically)."""
        u = -world_y
        variation = 45.0 * math.sin(u * 0.0028 + 2.1) + 20.0 * math.sin(u * 0.0069)
        return max(260.0, min(390.0, ROAD_BASE_WIDTH + variation))

    def get_road_edges(self, world_y: float):
        """Returns (left_edge, right_edge, center_x, width) at world_y."""
        cx = self.get_road_center(world_y)
        w = self.get_road_width(world_y)
        return cx - w / 2.0, cx + w / 2.0, cx, w

    def get_tangent_angle(self, world_y: float, step: float = 15.0) -> float:
        """
        Returns road heading angle in radians (0 = straight up, negative = left, positive = right).
        """
        cx1 = self.get_road_center(world_y)
        cx2 = self.get_road_center(world_y - step)
        dx = cx2 - cx1
        dy = -step # going forward
        return math.atan2(dx, -dy)

    def is_on_road(self, x: float, world_y: float, margin: float = 0.0) -> bool:
        """Checks if a coordinate is strictly within the drivable asphalt road."""
        left, right, _, _ = self.get_road_edges(world_y)
        return (left + margin) <= x <= (right - margin)

    def get_offroad_penalty(self, x: float, world_y: float) -> float:
        """
        Returns 0 if comfortably on road, increasing smoothly as car strays onto shoulder/grass.
        """
        left, right, _, _ = self.get_road_edges(world_y)
        if x < left:
            return (left - x)
        elif x > right:
            return (x - right)
        return 0.0

    def _generate_chunk_patches(self, chunk_id: int):
        """Generates deterministic asphalt tar patches and roadside details."""
        if chunk_id in self.patches:
            return self.patches[chunk_id]
        
        # Simple pseudo-random generator seeded by chunk_id
        seed = int((chunk_id * 9301 + 49297) % 233280)
        def rnd():
            nonlocal seed
            seed = (seed * 9301 + 49297) % 233280
            return seed / 233280.0

        chunk_y_start = chunk_id * self.chunk_size
        patches = []
        # 2 to 4 worn asphalt patches per chunk
        num_patches = int(2 + rnd() * 3)
        for _ in range(num_patches):
            py = chunk_y_start + rnd() * self.chunk_size
            cx = self.get_road_center(py)
            w = self.get_road_width(py)
            px = cx + (rnd() - 0.5) * (w * 0.7)
            pw = int(25 + rnd() * 50)
            ph = int(18 + rnd() * 35)
            angle = (rnd() - 0.5) * 40
            patches.append((px, py, pw, ph, angle))

        self.patches[chunk_id] = patches
        # Prune old chunks
        if len(self.patches) > 20:
            oldest = min(self.patches.keys())
            if oldest < chunk_id - 10:
                del self.patches[oldest]
        return patches

    def draw(self, surface: pygame.Surface, camera_y: float):
        """
        Renders the infinite road relative to the camera position.
        """
        h = surface.get_height()
        w = surface.get_width()

        # Fill with grass / countryside ground
        surface.fill(COLOR_BG_GRASS)

        # Draw road as a continuous polygon strip from top of screen to bottom
        step = 16
        start_y = int(camera_y - 60)
        end_y = int(camera_y + h + 60)

        shoulder_pts_left = []
        shoulder_pts_right = []
        road_pts_left = []
        road_pts_right = []
        center_pts = []

        for wy in range(start_y, end_y + step, step):
            screen_y = wy - camera_y
            left, right, cx, rw = self.get_road_edges(wy)

            # Shoulder edge points (unpaved dirt)
            shoulder_pts_left.append((left - SHOULDER_WIDTH, screen_y))
            shoulder_pts_right.append((right + SHOULDER_WIDTH, screen_y))

            # Asphalt edge points
            road_pts_left.append((left, screen_y))
            road_pts_right.append((right, screen_y))

            center_pts.append((cx, screen_y, wy))

        # 1. Draw Dirt Shoulder
        full_shoulder_poly = shoulder_pts_left + shoulder_pts_right[::-1]
        if len(full_shoulder_poly) >= 3:
            pygame.draw.polygon(surface, COLOR_DIRT_SHOULDER, full_shoulder_poly)

        # 2. Draw Main Asphalt Road
        full_road_poly = road_pts_left + road_pts_right[::-1]
        if len(full_road_poly) >= 3:
            pygame.draw.polygon(surface, COLOR_ASPHALT, full_road_poly)

        # 3. Draw Asphalt Patches (uneven road repairs)
        min_chunk = int(start_y // self.chunk_size)
        max_chunk = int(end_y // self.chunk_size)
        for cid in range(min_chunk, max_chunk + 1):
            patches = self._generate_chunk_patches(cid)
            for px, py, pw, ph, _ in patches:
                sy = py - camera_y
                if -50 <= sy <= h + 50:
                    patch_rect = pygame.Rect(int(px - pw / 2), int(sy - ph / 2), pw, ph)
                    pygame.draw.ellipse(surface, COLOR_ASPHALT_PATCH, patch_rect)

        # 4. Draw Broken / Faded Center Markings
        dash_len = 28
        gap_len = 34
        for i in range(len(center_pts) - 1):
            cx, sy, wy = center_pts[i]
            # Modulo on world_y for stable dashed lines as world moves
            if (-wy) % (dash_len + gap_len) < dash_len:
                nx, nsy, _ = center_pts[i+1]
                pygame.draw.line(surface, COLOR_ROAD_MARKING, (cx, sy), (nx, nsy), 3)

        # 5. Draw Rough Road Edge Lines
        if len(road_pts_left) > 1:
            pygame.draw.lines(surface, (50, 48, 45), False, road_pts_left, 2)
            pygame.draw.lines(surface, (50, 48, 45), False, road_pts_right, 2)
