"""
Simulation configuration and constants.
"""

# Screen & Display
SCREEN_WIDTH = 1440
SCREEN_HEIGHT = 860
FPS = 60
TITLE = "Autonomous(A*) Driving on Indian Roads"

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
PLAYER_MIN_SPEED = 0.0                  # Full stop capable (speed can become 0)
PLAYER_ACCEL = 160                     # Acceleration rate px/s^2
PLAYER_DECEL = 320                     # Braking rate px/s^2

# Realistic Steering Constraints
PLAYER_STEER_SPEED = 1.45              # Max angular turning speed (rad/s) - realistic highway steering
MAX_STEER_DEVIATION = 0.38             # Max allowable heading angle deviation from road tangent (~21 deg)
PLAYER_STEER_DAMPING = 0.88            # Inertial heading smoothing

# A* Planner Parameters
PLANNER_CELL_SIZE = 16                 # Grid resolution (pixels per cell)
PLANNER_LOOKAHEAD_DIST = 380           # Planning horizon ahead of car (px)
PLANNER_LONG_LOOKAHEAD = 500.0         # Strategic long-range corridor horizon (px)
PLANNER_SHORT_LOOKAHEAD = 185.0        # Reactive short-range obstacle horizon (px)
PLANNER_LATERAL_SPAN = 300             # Width of local search grid (px)
PLANNER_REPLAN_INTERVAL = 0.08         # Replan frequency in seconds (12.5 Hz)
SAFETY_MARGIN_CAR = 18                 # Buffer distance around vehicles
SAFETY_MARGIN_POTHOLE = 14             # Buffer distance around potholes
SAFETY_MARGIN_PEDESTRIAN = 18          # Buffer distance around pedestrians

# Obstacle Generation
POTHOLE_SPAWN_INTERVAL = (250, 580)    # Reduced pothole frequency (by ~55%)
TRAFFIC_SPAWN_INTERVAL = (220, 460)    # Vertical distance between traffic cars
PEDESTRIAN_SPAWN_INTERVAL = (140, 310) # Vertical distance between pedestrians
MAX_TRAFFIC_AHEAD = 7
MAX_POTHOLES_AHEAD = 12
MAX_PEDESTRIANS_AHEAD = 8

# Pothole Obstacle Speed Threshold (px/s)
# At speeds > 6-7 km/h (~23.5 px/s), all potholes are treated as pure obstacles with infinite cost.
# Below this threshold, vehicles crawl across with finite penalty.
POTHOLE_OBSTACLE_SPEED_THRESHOLD = 23.5

# Rest Acceleration Reduction (20% reduction when accelerating from rest)
REST_ACCEL_FACTOR = 0.80

# Traffic Direction Vector & Rear Cut-In Avoidance Parameters
TRAFFIC_VECTOR_PRED_HORIZON = 2.0       # Projection horizon for traffic vectors (seconds)
REAR_CUTIN_MIN_HEADWAY_SEC = 0.70       # Min headway required to cut into adjacent lane (seconds)
REAR_CUTIN_SAFE_HEADWAY_SEC = 1.80      # Headway horizon where rear penalty applies (seconds)
REAR_CUTIN_MIN_GAP_PX = 60.0            # Min physical gap behind when cutting into lane (px)

# Vehicle Collision & Crowded Speed Penalties
VEHICLE_COLLISION_PENALTY_SCALE = 1.60   # +60% penalty increase for vehicle collisions across all vehicles
CROWDED_AREA_SPEED_PENALTY_MAX = 0.20    # Up to 20% additional penalty for high speeds in crowded areas
CROWDED_NEIGHBOR_RADIUS = 150.0          # Spatial radius to detect crowded vehicle clusters (px)
