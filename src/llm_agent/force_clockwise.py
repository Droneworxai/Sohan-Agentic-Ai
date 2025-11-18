#!/usr/bin/env python3
"""Force CLOCKWISE for demo comparison"""
import json
import math

# Load original GeoJSON
with open('loop_path.geojson', 'r') as f:
    geojson = json.load(f)

coords = geojson['features'][0]['geometry']['coordinates']
origin_lon, origin_lat = coords[0]

def gps_to_xy(lon, lat):
    R = 6371000
    lat1, lon1 = math.radians(origin_lat), math.radians(origin_lon)
    lat2, lon2 = math.radians(lat), math.radians(lon)
    x = R * (lon2 - lon1) * math.cos((lat1 + lat2) / 2)
    y = R * (lat2 - lat1)
    return (x, y)

# Convert to local coordinates (ORIGINAL ORDER = CLOCKWISE)
waypoints = [[round(x, 2), round(y, 2)] for x, y in [gps_to_xy(lon, lat) for lon, lat in coords]]

# Create CLOCKWISE mission
mission = {
    "strategy": "FORCED CLOCKWISE for demo",
    "llm_reasoning": "Morning conditions: Sun from east favors clockwise for optimal camera positioning",
    "llm_confidence": "high",
    "local_coords": waypoints  # ORIGINAL ORDER
}

with open('mission_plan.json', 'w') as f:
    json.dump(mission, f, indent=2)

print("=" * 70)
print("✅ MISSION SET TO CLOCKWISE")
print("=" * 70)
print(f"Total waypoints: {len(waypoints)}")
print(f"First waypoint: {waypoints[0]}")
print(f"Last waypoint: {waypoints[-1]}")
print()
print("Robot will now go CLOCKWISE (opposite of counter-clockwise)")
print("=" * 70)
