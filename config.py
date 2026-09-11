"""
Simulation configuration and constants.
"""

# Screen & Display
SCREEN_WIDTH = 960
SCREEN_HEIGHT = 720
FPS = 60
TITLE = "Indian Highway Autonomous Driving (A* Pathfinding)"

# Colors (RGB)
COLOR_BG_GRASS = (46, 89, 44)         # Dry lush roadside greenery / patchy ground
COLOR_DIRT_SHOULDER = (138, 110, 74)   # Earthy dirt / unpaved shoulder
COLOR_ASPHALT = (68, 68, 70)           # Weathered Indian road asphalt
COLOR_ASPHALT_PATCH = (55, 55, 58)     # Darker tar patch
COLOR_ROAD_MARKING = (210, 205, 175)   # Faded yellowish-white marking
COLOR_POTHOLE_INNER = (24, 20, 18)     # Deep dark hole
COLOR_POTHOLE_RIM = (100, 85, 70)      # Crumbling rim edge
COLOR_TEXT = (240, 240, 240)
COLOR_HUD_BG = (20, 24, 28, 200)

# A* Debug Colors
COLOR_PATH_LINE = (0, 240, 255)        # Neon Cyan for active planned path
COLOR_PATH_WAYPOINT = (255, 230, 50)   # Yellow waypoints
COLOR_GRID_EXPLORED = (80, 180, 220, 50) # Explored cells
COLOR_GRID_OBSTACLE = (255, 50, 50, 70)  # Obstacle cells

# Road Geometry
ROAD_BASE_WIDTH = 320                  # Base road width in pixels
ROAD_MIN_WIDTH = 205                   # Narrowest choke points / bridge bottlenecks
ROAD_MAX_WIDTH = 420                   # Wide open highway stretches
SHOULDER_WIDTH = 40                    # Dirt edge on each side
ROAD_SEGMENT_LENGTH = 10               # Resolution of road curve points

# Player Car
PLAYER_WIDTH = 26
PLAYER_LENGTH = 52
PLAYER_BASE_SPEED = 210                # pixels per second (~60 km/h scale)
PLAYER_MAX_SPEED = 290                 # Max speed on open clear roads
PLAYER_MIN_SPEED = 50                  # Minimum crawl speed
PLAYER_ACCEL = 160                     # Acceleration rate px/s^2
PLAYER_DECEL = 290                     # Braking rate px/s^2
PLAYER_STEER_SPEED = 2.5               # Controlled max turning radians/s (controlled steering constraint)
PLAYER_STEER_DAMPING = 0.85            # Inertial heading smoothing

# A* Planner Parameters
PLANNER_CELL_SIZE = 16                 # Grid resolution (pixels per cell)
PLANNER_LOOKAHEAD_DIST = 380           # Planning horizon ahead of car (px)
PLANNER_LATERAL_SPAN = 300             # Width of local search grid (px)
PLANNER_REPLAN_INTERVAL = 0.08         # Replan frequency in seconds (12.5 Hz)
SAFETY_MARGIN_CAR = 18                 # Buffer distance around vehicles
SAFETY_MARGIN_POTHOLE = 14             # Buffer distance around potholes
SAFETY_MARGIN_PEDESTRIAN = 18          # Buffer distance around pedestrians

# Obstacle Generation
POTHOLE_SPAWN_INTERVAL = (90, 220)     # Vertical distance between potholes
TRAFFIC_SPAWN_INTERVAL = (220, 480)    # Vertical distance between traffic cars
PEDESTRIAN_SPAWN_INTERVAL = (140, 310) # Vertical distance between pedestrians
MAX_TRAFFIC_AHEAD = 6
MAX_POTHOLES_AHEAD = 12
MAX_PEDESTRIANS_AHEAD = 8
