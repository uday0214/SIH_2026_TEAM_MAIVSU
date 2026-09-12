"""
Procedural infinite road generator with unstructured, dynamic width variations
and sparse, weathered Indian road characteristics.
"""

import math
import pygame
from config import (
    SCREEN_WIDTH, SCREEN_HEIGHT,
    COLOR_BG_GRASS, COLOR_DIRT_SHOULDER, COLOR_ASPHALT,
    COLOR_ASPHALT_PATCH, COLOR_ROAD_MARKING,
    ROAD_BASE_WIDTH, ROAD_MIN_WIDTH, ROAD_MAX_WIDTH, SHOULDER_WIDTH
)

class InfiniteRoad:
    def __init__(self):
        # Base road parameters (centered in the driving viewport)
        self.base_cx = (SCREEN_WIDTH - 380.0) / 2.0
        
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
        """
        Returns the road width at world_y.
        Varies dynamically between wide open stretches (~420px)
        and narrow choke points / culvert pinches (~205px).
        """
        u = -world_y
        # Broad macro changes between wide highway and narrower village roads
        macro = 60.0 * math.sin(u * 0.0011) + 28.0 * math.sin(u * 0.0029 + 1.7)

        # Localized choke points (e.g. narrow bridges, roadside encroachment, culverts)
        choke = math.sin(u * 0.00065 + 0.8)
        choke_penalty = 0.0
        if choke > 0.65:
            choke_penalty = ((choke - 0.65) / 0.35) * 85.0
        
        # Micro unevenness
        micro = 14.0 * math.sin(u * 0.0075 + 0.3)
        
        w = ROAD_BASE_WIDTH + macro - choke_penalty + micro
        return max(ROAD_MIN_WIDTH, min(ROAD_MAX_WIDTH, w))

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

    def get_virtual_lanes(self, world_y: float):
        """
        Returns list of virtual lane center x-coordinates and lane width at world_y.
        Virtual lanes provide an underlying structural guidance grid for all vehicles.
        In Indian traffic (driving on the left), the left side is the primary travel lane.
        """
        left, right, cx, rw = self.get_road_edges(world_y)
        if rw >= 320.0:
            # 3-lane structure on wide highway: Left Lane (0), Center Lane (1), Right Overtaking Lane (2)
            lane_w = rw / 3.0
            lanes = [
                cx - rw * 0.28,  # Lane 0: Left primary lane
                cx,              # Lane 1: Center passing corridor
                cx + rw * 0.28   # Lane 2: Right overtaking lane
            ]
        else:
            # 2-lane structure on standard/narrow road: Left driving lane (0), Right overtaking lane (1)
            lane_w = rw / 2.0
            lanes = [
                cx - rw * 0.24,  # Lane 0: Left driving lane
                cx + rw * 0.24   # Lane 1: Right overtaking lane
            ]
        return lanes, lane_w

    def get_nearest_virtual_lane(self, x: float, world_y: float):
        """
        Returns (lane_index, lane_center_x, offset_from_road_center) for given position (x, world_y).
        """
        lanes, _ = self.get_virtual_lanes(world_y)
        cx = self.get_road_center(world_y)
        best_idx = 0
        best_dist = abs(x - lanes[0])
        for idx in range(1, len(lanes)):
            d = abs(x - lanes[idx])
            if d < best_dist:
                best_dist = d
                best_idx = idx
        lane_cx = lanes[best_idx]
        return best_idx, lane_cx, (lane_cx - cx)

    def get_road_side(self, x: float, world_y: float) -> str:
        """Returns 'LEFT' if vehicle is on left side of road, 'RIGHT' if on right side, 'CENTER' if near dividing line."""
        cx = self.get_road_center(world_y)
        delta = x - cx
        if delta < -14.0:
            return 'LEFT'
        elif delta > 14.0:
            return 'RIGHT'
        return 'CENTER'

    def is_cutting_to_other_side(self, from_x: float, to_x: float, world_y: float) -> bool:
        """
        Checks if a lateral transition crosses across the road centerline to the opposite side.
        """
        cx = self.get_road_center(world_y)
        d_from = from_x - cx
        d_to = to_x - cx
        return (d_from * d_to < 0) and abs(d_to) > 14.0

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

        # 4. Sparse Weathered Center Markings (only appear sparsely here and there)
        dash_len = 24
        gap_len = 36
        for i in range(len(center_pts) - 1):
            cx, sy, wy = center_pts[i]
            # Only active on isolated sparse segments (~15% of the highway)
            sparse_active = (math.sin((-wy) * 0.0013) > 0.72)
            if sparse_active and ((-wy) % (dash_len + gap_len) < dash_len):
                nx, nsy, _ = center_pts[i+1]
                # Faded worn paint
                pygame.draw.line(surface, COLOR_ROAD_MARKING, (cx, sy), (nx, nsy), 2)

        # 5. Draw Rough Road Edge Lines
        if len(road_pts_left) > 1:
            pygame.draw.lines(surface, (50, 48, 45), False, road_pts_left, 2)
            pygame.draw.lines(surface, (50, 48, 45), False, road_pts_right, 2)
