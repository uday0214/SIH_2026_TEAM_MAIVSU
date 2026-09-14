#!/usr/bin/env python3
"""
Automated Multi-Modal Sensor Benchmark & Utility Optimizer for Pothole Detection.
Evaluates 5 sensors across 40 operational scenarios:
- 5 Distances: 3m, 6m, 9m, 12m, 15m (at optimal pitch angles theta*)
- 2 Lighting conditions: Day vs. Night (with vehicle headlights)
- 4 Water levels: 0% (Dry), 25% (Shallow), 50% (Half-filled), 100% (Brim-full)

Calculates Weighted Multi-Attribute Utility (MAUF) and Cost-Effectiveness Ratio (CER).
"""

import math
import json

# ── Simulation Physical Constants & Geometries ─────────────────────────────────
POTHOLE_WIDTH = 0.45    # meters
POTHOLE_DEPTH = 0.12    # meters
POTHOLE_SLOPE = 65.0    # degrees
VEHICLE_HEIGHT = 1.50   # meters
ROOF_MOUNT_Y = 1.75     # meters
BUMPER_MOUNT_Y = 0.35   # meters

DISTANCES = [3.0, 6.0, 9.0, 12.0, 15.0]
DIST_WEIGHTS = [0.15, 0.25, 0.25, 0.20, 0.15]

TIMES_OF_DAY = ["day", "night"]
TOD_WEIGHTS = [0.50, 0.50]

WATER_LEVELS = [0.0, 0.25, 0.50, 1.00]
WATER_WEIGHTS = [0.35, 0.25, 0.20, 0.20]

# ── 1. Sensor Cost Model (INR ₹) ───────────────────────────────────────────────
def get_sensor_costs():
    """Returns baseline INR costs for default automotive-grade configurations."""
    # LiDAR (32 channels, 30 deg FOV)
    ch = 32
    fov_deg = 30
    lidar_base = 150000 + int(((ch - 16) / 16) * (350000 - 150000))
    lidar_ch_prem = ch * 2800
    lidar_fov_prem = int(((fov_deg - 15) / 15) * 45000)
    lidar_total = lidar_base + lidar_ch_prem + lidar_fov_prem

    # RGB Camera (1920x1080 HDR, 60 deg FOV)
    cam_res = 1920
    cam_base = 18000 + int(((cam_res - 1280) / 640) * 17000)
    cam_res_prem = int(max(0, (cam_res - 1280) * 8))
    cam_total = cam_base + cam_res_prem

    # 4D Imaging Radar (768 virtual MIMO channels)
    rad_ch = 768
    radar_base = 50000 + int(((rad_ch - 192) / (768 - 192)) * 45000)
    radar_ch_prem = int(rad_ch * 45)
    radar_dsp = int(18000 + rad_ch * 18)
    radar_total = radar_base + radar_ch_prem + radar_dsp

    # Gated SWIR (1550 nm InGaAs, 250W peak laser, ns gate)
    swir_base = 185000
    swir_laser = 55000 + int((250 / 250) * 35000)
    swir_gate = 45000
    swir_total = swir_base + swir_laser + swir_gate

    # Thermal LWIR (VOx microbolometer 35 mK, Germanium optics)
    thermal_netd = 35
    thermal_base = 105000 + int(((65 - thermal_netd) / 50) * 45000)
    thermal_optics = 42000
    thermal_dsp = 28000
    thermal_total = thermal_base + thermal_optics + thermal_dsp

    return {
        "lidar": lidar_total,
        "camera": cam_total,
        "radar": radar_total,
        "swir": swir_total,
        "thermal": thermal_total,
    }

# ── 2. Optimal Look-Down Pitch Angle Finder ────────────────────────────────────
def get_optimal_pitch(sensor_name, dist_m, mount_y):
    """
    Computes optimal elevation angle theta* (radians) to center sensor FOV on pothole.
    theta* = -arctan(mount_y / dist_m) with sensor-specific boresight offset.
    """
    geom_angle = -math.atan2(mount_y, dist_m)
    # Refinements per sensor FOV coverage:
    if sensor_name == "lidar":
        # LiDAR 30 deg FOV: aim slightly lower so bottom channels penetrate hole cavity
        return geom_angle - math.radians(1.5)
    elif sensor_name == "camera":
        # Camera 60 deg FOV: center directly on rim
        return geom_angle
    elif sensor_name == "radar":
        # Radar 20 deg FOV: aim at cavity back-wall corner
        return geom_angle - math.radians(1.0)
    elif sensor_name == "swir":
        # SWIR 50 deg FOV
        return geom_angle
    elif sensor_name == "thermal":
        # Thermal 50 deg FOV: aim directly at cavity floor
        return geom_angle
    return geom_angle

# ── 3. Individual Sensor Effectiveness Models ──────────────────────────────────
def evaluate_lidar_scenario(dist_m, tod, water_lvl, mount_y):
    """
    Evaluates LiDAR beam penetration into cavity.
    Calculates number of beams landing inside pothole floor given pitch theta*.
    """
    theta_opt = get_optimal_pitch("lidar", dist_m, mount_y)
    fov = math.radians(30)
    channels = 32
    min_angle = theta_opt - fov / 2
    max_angle = theta_opt + fov / 2
    
    p_x1 = dist_m
    p_x2 = dist_m + POTHOLE_WIDTH
    
    hits_cavity = 0
    for i in range(channels):
        ang = min_angle if channels == 1 else min_angle + (i / (channels - 1)) * (max_angle - min_angle)
        # Ground hit position assuming y=0:
        if math.tan(ang) < -1e-4:
            hit_x = -mount_y / math.tan(ang)
            if p_x1 <= hit_x <= p_x2:
                hits_cavity += 1

    # Base hit score:
    if hits_cavity >= 3:
        raw_score = 1.0
    elif hits_cavity == 2:
        raw_score = 0.82
    elif hits_cavity == 1:
        raw_score = 0.55
    else:
        # Distance-dependent glancing proximity detection
        raw_score = max(0.10, 0.45 * math.exp(-dist_m / 8.0))

    # Water effect: Specular reflection scatters 905nm/1550nm pulses away at oblique incidence
    if water_lvl > 0.05:
        specular_loss = 0.15 + water_lvl * 0.25
        raw_score = max(0.08, raw_score * (1.0 - specular_loss))

    # Day/Night: LiDAR is an active emitter (905 nm), ambient daylight introduces negligible solar background noise
    if tod == "night":
        raw_score = min(1.0, raw_score * 1.02)  # Zero solar background noise at night
    else:
        raw_score *= 1.00

    return min(1.0, max(0.0, raw_score)), math.degrees(theta_opt)

def evaluate_camera_scenario(dist_m, tod, water_lvl, mount_y):
    """
    Evaluates RGB Camera edge detection, texture gradient, and depth shadow.
    Subject to headlight cone constraints at night.
    """
    theta_opt = get_optimal_pitch("camera", dist_m, mount_y)
    cam_range = 30.0
    res = 1920
    fov = math.radians(60)

    # Angular span across pothole
    ang1 = math.atan2(-mount_y, dist_m)
    ang2 = math.atan2(-mount_y, dist_m + POTHOLE_WIDTH)
    angular_span = abs(ang1 - ang2)
    pixel_coverage = (angular_span / fov) * res

    # Texture & Edge detection factors
    dist_factor = max(0.0, 1.0 - (dist_m / cam_range))
    res_factor = min(1.0, pixel_coverage / 45.0)
    size_factor = min(1.0, POTHOLE_WIDTH / 0.3)
    water_bonus = 0.25 if water_lvl > 0.1 else 0.0

    texture_score = min(1.0, dist_factor * 0.40 + res_factor * 0.35 + size_factor * 0.25 + water_bonus)
    
    depth_contrast = min(1.0, POTHOLE_DEPTH / 0.08)
    dist_penalty = max(0.0, 1.0 - math.sqrt(min(dist_m / 20.0, 1.0)))
    edge_score = min(1.0, depth_contrast * 0.45 + dist_penalty * 0.30 + res_factor * 0.25)
    
    depth_est_score = min(1.0, (POTHOLE_DEPTH / 0.06) * 0.5 + (1 - min(dist_m / 15.0, 1.0)) * 0.3 + (pixel_coverage / 40.0) * 0.2)

    composite = 0.40 * texture_score + 0.35 * edge_score + 0.25 * depth_est_score

    # Night-time headlight falloff
    if tod == "night":
        headlight_range = 22.0
        if dist_m <= headlight_range:
            night_factor = 0.55 + 0.35 * (1.0 - dist_m / headlight_range)
        else:
            dark_dist = dist_m - headlight_range
            night_factor = max(0.04, 0.55 * math.exp(-dark_dist / 3.0))
        composite *= night_factor

    return min(1.0, max(0.0, composite)), math.degrees(theta_opt)

def evaluate_radar_scenario(dist_m, tod, water_lvl, mount_y):
    """
    Evaluates 77 GHz 4D Imaging Radar MIMO point density, cavity dihedral reflection, and Doppler.
    """
    theta_opt = get_optimal_pitch("radar", dist_m, mount_y)
    rad_range = 35.0
    num_beams = 24
    
    in_range = dist_m < rad_range
    if not in_range:
        return 0.0, math.degrees(theta_opt)

    # Elevation resolution score
    dist_factor = max(0.2, 1.0 - (dist_m / rad_range) * 0.6)
    elevation_score = min(1.0, 0.45 + dist_factor * 0.55)

    # RCS: Back-wall corner reflector dihedral return + water dielectric boost
    # Dry asphalt RCS ~ -10 dBsm, Water dielectric (eps_r=81) reflection ~ -4 dBsm
    base_rcs = 6.0 if dist_m <= 10.0 else max(1.0, 6.0 - (dist_m - 10.0) * 0.8)
    if water_lvl > 0.1:
        water_boost = 3.5 * water_lvl
        rcs_score = min(1.0, (base_rcs + water_boost) / 12.0)
    else:
        rcs_score = min(1.0, base_rcs / 12.0)

    # Doppler ground speed validation (stationary obstacle zero-residual)
    doppler_score = 0.95

    composite = 0.40 * elevation_score + 0.35 * rcs_score + 0.25 * doppler_score
    # 77 GHz radar is completely immune to Day vs Night lighting conditions
    return min(1.0, max(0.0, composite)), math.degrees(theta_opt)

def evaluate_swir_scenario(dist_m, tod, water_lvl, mount_y):
    """
    Evaluates 1550 nm Gated SWIR.
    Active pulsed laser slice with strong water absorption (alpha ~ 30 cm^-1).
    """
    theta_opt = get_optimal_pitch("swir", dist_m, mount_y)
    # Range gate dynamically synchronized to target distance corridor [dist_m - 1.0, dist_m + 5.0]
    gate_start = max(1.0, dist_m - 1.0)
    gate_width = 6.0
    in_gate = (gate_start <= dist_m <= gate_start + gate_width)
    
    if not in_gate:
        return 0.05, math.degrees(theta_opt)

    power_ratio = min(1.5, 250.0 / 250.0)
    has_water = water_lvl > 0.05
    water_bonus = (0.35 * (water_lvl / 1.0)) if has_water else 0.10
    
    dist_attenuation = max(0.35, 1.0 - (dist_m / 35.0) * 0.45)
    gate_score = min(1.0, (0.65 * power_ratio + water_bonus) * dist_attenuation)
    
    # Active 1550nm illumination is completely day/night invariant
    return min(1.0, max(0.0, gate_score)), math.degrees(theta_opt)

def evaluate_thermal_scenario(dist_m, tod, water_lvl, mount_y):
    """
    Evaluates 8-14 um Thermal LWIR radiometric Delta T and NETD noise floor.
    """
    theta_opt = get_optimal_pitch("thermal", dist_m, mount_y)
    th_range = 35.0
    is_day = (tod == "day")
    road_temp = 41.5 if is_day else 16.2

    if water_lvl > 0.05:
        # Evaporative cooling by day, sky zenith reflection by night
        cavity_temp = 24.5 if is_day else 8.8
    else:
        cavity_temp = 29.8 if is_day else 23.4

    delta_t = cavity_temp - road_temp
    abs_delta_t = abs(delta_t)
    netd = 35.0  # mK

    snr = (abs_delta_t * 1000.0) / (netd * 2.5)
    dist_penalty = max(0.2, 1.0 - (dist_m / th_range) * 0.45)
    thermal_score = min(1.0, (snr / 15.0) * dist_penalty)

    return min(1.0, max(0.0, thermal_score)), math.degrees(theta_opt)

# ── 4. Main Matrix Runner ──────────────────────────────────────────────────────
def run_benchmark():
    eval_funcs = {
        "lidar": evaluate_lidar_scenario,
        "camera": evaluate_camera_scenario,
        "radar": evaluate_radar_scenario,
        "swir": evaluate_swir_scenario,
        "thermal": evaluate_thermal_scenario,
    }
    
    costs = get_sensor_costs()
    results = {}

    for sensor_name, func in eval_funcs.items():
        mount_y = ROOF_MOUNT_Y if sensor_name in ["lidar", "swir"] else BUMPER_MOUNT_Y
        
        scenario_scores = []
        weighted_sum = 0.0
        total_weight = 0.0

        dist_scores = {d: [] for d in DISTANCES}
        tod_scores = {t: [] for t in TIMES_OF_DAY}
        water_scores = {w: [] for w in WATER_LEVELS}
        optimal_angles = {}

        for i, d in enumerate(DISTANCES):
            w_d = DIST_WEIGHTS[i]
            for j, t in enumerate(TIMES_OF_DAY):
                w_t = TOD_WEIGHTS[j]
                for k, w in enumerate(WATER_LEVELS):
                    w_w = WATER_WEIGHTS[k]
                    
                    weight = w_d * w_t * w_w
                    score, opt_ang = func(d, t, w, mount_y)
                    optimal_angles[d] = opt_ang
                    
                    scenario_scores.append({
                        "distance": d,
                        "time_of_day": t,
                        "water_level": w,
                        "optimal_pitch_deg": round(opt_ang, 2),
                        "score": round(score, 4),
                        "weight": round(weight, 6)
                    })

                    weighted_sum += score * weight
                    total_weight += weight
                    dist_scores[d].append(score)
                    tod_scores[t].append(score)
                    water_scores[w].append(score)

        agg_effectiveness = weighted_sum / total_weight

        results[sensor_name] = {
            "name": sensor_name,
            "cost_inr": costs[sensor_name],
            "aggregated_effectiveness": agg_effectiveness,
            "optimal_angles_by_dist": {d: round(optimal_angles[d], 2) for d in DISTANCES},
            "avg_by_dist": {d: round(sum(dist_scores[d]) / len(dist_scores[d]), 4) for d in DISTANCES},
            "avg_by_tod": {t: round(sum(tod_scores[t]) / len(tod_scores[t]), 4) for t in TIMES_OF_DAY},
            "avg_by_water": {w: round(sum(water_scores[w]) / len(water_scores[w]), 4) for w in WATER_LEVELS},
            "scenarios": scenario_scores
        }

    # ── 5. Multi-Attribute Utility & CER Calculation ───────────────────────────
    all_costs = [r["cost_inr"] for r in results.values()]
    min_cost = min(all_costs)
    max_cost = max(all_costs)

    # Weighting profiles: Balanced (wE=0.65, wC=0.35), Safety-First (wE=0.85, wC=0.15), Budget (wE=0.45, wC=0.55)
    profiles = {
        "balanced": {"w_E": 0.65, "w_C": 0.35},
        "safety_first": {"w_E": 0.85, "w_C": 0.15},
        "budget": {"w_E": 0.45, "w_C": 0.55},
    }

    for s_name, data in results.items():
        cost = data["cost_inr"]
        norm_cost = (cost - min_cost) / (max_cost - min_cost) if max_cost > min_cost else 0.5
        data["norm_cost"] = round(norm_cost, 4)
        data["cost_savings_norm"] = round(1.0 - norm_cost, 4)
        
        # Cost-Effectiveness Ratio (% per 10k INR)
        eff_pct = data["aggregated_effectiveness"] * 100.0
        data["cer_pct_per_10k"] = round(eff_pct / (cost / 10000.0), 3)

        data["utility_scores"] = {}
        for p_name, p_weights in profiles.items():
            u = p_weights["w_E"] * data["aggregated_effectiveness"] + p_weights["w_C"] * (1.0 - norm_cost)
            data["utility_scores"][p_name] = round(u * 100.0, 2)

    return results

if __name__ == "__main__":
    benchmark_data = run_benchmark()
    print(json.dumps(benchmark_data, indent=2))
