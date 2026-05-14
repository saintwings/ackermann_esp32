"""
Coverage Path Planning Service
Integrates coverage_path_planning.py with lat/lon coordinate conversion
"""

import numpy as np
import math
from coverage_path_planning import (
    MapEnvironment, CoveragePlanner, Geometry
)

# ============================================================================
# Coordinate Conversion Utilities
# ============================================================================

class CoordinateConverter:
    """Convert between lat/lon and local x/y coordinates"""
    
    # Earth radius in meters
    EARTH_RADIUS = 6371000
    
    def __init__(self, origin_lat: float, origin_lon: float):
        """
        Initialize converter with origin point (lat/lon)
        
        Args:
            origin_lat: Reference latitude in degrees
            origin_lon: Reference longitude in degrees
        """
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_lat_rad = math.radians(origin_lat)
        self.origin_lon_rad = math.radians(origin_lon)
    
    def latlon_to_xy(self, lat: float, lon: float) -> tuple:
        """
        Convert lat/lon to local x/y (meters from origin)
        
        Uses equirectangular approximation (accurate for small areas < 10km)
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            
        Returns:
            (x, y) in meters
        """
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        # Equirectangular approximation
        x = (lon_rad - self.origin_lon_rad) * self.EARTH_RADIUS * math.cos(self.origin_lat_rad)
        y = (lat_rad - self.origin_lat_rad) * self.EARTH_RADIUS
        
        return x, y
    
    def xy_to_latlon(self, x: float, y: float) -> tuple:
        """
        Convert local x/y (meters) to lat/lon
        
        Args:
            x: X coordinate in meters from origin
            y: Y coordinate in meters from origin
            
        Returns:
            (lat, lon) in degrees
        """
        lat = self.origin_lat + (y / self.EARTH_RADIUS) * (180 / math.pi)
        lon = self.origin_lon + (x / (self.EARTH_RADIUS * math.cos(self.origin_lat_rad))) * (180 / math.pi)
        
        return lat, lon
    
    def latlon_list_to_xy(self, waypoints: list) -> list:
        """Convert list of [lat, lon] to list of [x, y]"""
        return [self.latlon_to_xy(wp[0], wp[1]) for wp in waypoints]
    
    def xy_list_to_latlon(self, waypoints: list) -> list:
        """Convert list of [x, y] to list of [lat, lon]"""
        return [self.xy_to_latlon(wp[0], wp[1]) for wp in waypoints]


class CoveragePathService:
    """Service to generate coverage paths from boundary and obstacles"""
    
    def __init__(self):
        self.converter = None
        self.planner = None
    
    def plan_coverage(self, 
                     boundary_latlon: list,
                     obstacles_latlon: list,
                     row_spacing: float = 0.5,
                     safety_margin: float = 0.3,
                     sweep_angle: float = 0,
                     robot_speed: float = 1.0,
                     turn_threshold_deg: float = 8.0,
                     max_straight_spacing_m: float = 2.0,
                     max_waypoints: int = 200) -> dict:
        """
        Generate coverage path from boundary and obstacles (lat/lon)
        
        Args:
            boundary_latlon: List of [lat, lon] waypoints defining work area boundary
            obstacles_latlon: List of obstacles, each is a list of [lat, lon] waypoints
            row_spacing: Distance between coverage sweep rows (meters)
            safety_margin: Safety clearance from boundary/obstacles (meters)
            sweep_angle: Preferred sweep direction in degrees (0=N-S, 90=E-W)
            robot_speed: Robot forward speed in m/s (for time estimation)
            turn_threshold_deg: Keep points where heading changes by at least this amount
            max_straight_spacing_m: Maximum spacing between kept points on straight runs
            max_waypoints: Hard cap for returned waypoint count (preserves turns/endpoints)
            
        Returns:
            {
                'success': bool,
                'error': str or None,
                'waypoints': [{'lat': float, 'lon': float, 'heading': float}, ...],
                'cells_count': int,
                'total_distance': float,  # meters
                'estimated_time': float,   # seconds
                'statistics': {
                    'boundary_area': float,
                    'coverage_strips': int,
                    'transition_distance': float
                }
            }
        """
        try:
            # Validate inputs
            if not boundary_latlon or len(boundary_latlon) < 3:
                return {
                    'success': False,
                    'error': 'Boundary must have at least 3 points',
                    'waypoints': [],
                    'cells_count': 0,
                    'total_distance': 0,
                    'estimated_time': 0,
                    'statistics': {}
                }
            
            # Initialize coordinate converter with first boundary point as origin
            origin_lat, origin_lon = boundary_latlon[0]
            self.converter = CoordinateConverter(origin_lat, origin_lon)
            
            # Convert lat/lon to x/y
            boundary_xy = self.converter.latlon_list_to_xy(boundary_latlon)
            obstacles_xy = [
                self.converter.latlon_list_to_xy(obs) 
                for obs in obstacles_latlon
            ]
            
            # Create environment (in local x/y coordinates)
            env = MapEnvironment(
                boundary=boundary_xy,
                obstacles=obstacles_xy,
                margin=safety_margin,
                sweep_angle=sweep_angle,
                origin=(boundary_xy[0][0], boundary_xy[0][1])  # Use first point as reference origin
            )
            
            # Plan coverage path
            self.planner = CoveragePlanner(env)
            path_xy = self.planner.plan_path(row_spacing=row_spacing)
            
            if not path_xy:
                return {
                    'success': False,
                    'error': 'No valid coverage path found',
                    'waypoints': [],
                    'cells_count': 0,
                    'total_distance': 0,
                    'estimated_time': 0,
                    'statistics': {}
                }

            # Reduce excessive points on long straight segments while preserving
            # turning points and segment endpoints.
            simplified_path_xy = self._simplify_path(
                path_xy,
                turn_threshold_deg=max(0.5, turn_threshold_deg),
                max_straight_spacing_m=max(0.2, max_straight_spacing_m)
            )
            minimum_waypoints_required = self._count_required_waypoints(
                simplified_path_xy,
                turn_threshold_deg=max(0.5, turn_threshold_deg)
            )
            pre_limit_waypoint_count = len(simplified_path_xy)
            simplified_path_xy = self._enforce_waypoint_limit(
                simplified_path_xy,
                max_waypoints=max(minimum_waypoints_required, max(3, int(max_waypoints))),
                turn_threshold_deg=max(0.5, turn_threshold_deg)
            )
            
            # Convert path back to lat/lon and compute headings
            waypoints_result = []
            total_distance = 0
            
            for i, (x, y) in enumerate(simplified_path_xy):
                lat, lon = self.converter.xy_to_latlon(x, y)
                
                # Compute heading to next waypoint
                heading = 0
                if i < len(simplified_path_xy) - 1:
                    x_next, y_next = simplified_path_xy[i + 1]
                    heading = self._compute_heading(x, y, x_next, y_next)
                    distance = math.hypot(x_next - x, y_next - y)
                    total_distance += distance
                
                waypoints_result.append({
                    'lat': round(lat, 8),
                    'lon': round(lon, 8),
                    'heading': round(heading, 1)
                })
            
            # Estimated time
            estimated_time = total_distance / max(robot_speed, 0.1) if robot_speed > 0 else 0
            
            # Statistics
            stats = {
                'boundary_area': self._compute_polygon_area(boundary_xy),
                'coverage_strips': len(self.planner.cells),
                'transition_distance': self._compute_transition_distance(simplified_path_xy),
                'raw_waypoints': len(path_xy),
                'simplified_waypoints': len(simplified_path_xy),
                'minimum_waypoints_required': minimum_waypoints_required,
                'pre_limit_waypoints': pre_limit_waypoint_count
            }
            
            return {
                'success': True,
                'error': None,
                'waypoints': waypoints_result,
                'cells_count': len(self.planner.cells),
                'total_distance': round(total_distance, 2),
                'estimated_time': round(estimated_time, 1),
                'statistics': stats
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'waypoints': [],
                'cells_count': 0,
                'total_distance': 0,
                'estimated_time': 0,
                'statistics': {}
            }

    @staticmethod
    def _heading_delta_deg(h1: float, h2: float) -> float:
        """Smallest absolute heading delta in degrees (0..180)."""
        d = abs(h2 - h1) % 360.0
        return min(d, 360.0 - d)

    def _simplify_path(self,
                       path_xy: list,
                       turn_threshold_deg: float,
                       max_straight_spacing_m: float) -> list:
        """
        Keep key points only:
        - Start/end always
        - Points near heading changes (turns)
        - Occasional points on long straight runs (spacing cap)
        """
        if len(path_xy) <= 2:
            return list(path_xy)

        simplified = [path_xy[0]]
        last_kept = 0

        for i in range(1, len(path_xy) - 1):
            p_prev = path_xy[i - 1]
            p_cur = path_xy[i]
            p_next = path_xy[i + 1]

            h1 = self._compute_heading(p_prev[0], p_prev[1], p_cur[0], p_cur[1])
            h2 = self._compute_heading(p_cur[0], p_cur[1], p_next[0], p_next[1])
            turn_delta = self._heading_delta_deg(h1, h2)

            p_last = path_xy[last_kept]
            distance_from_last = math.hypot(p_cur[0] - p_last[0], p_cur[1] - p_last[1])

            keep_for_turn = turn_delta >= turn_threshold_deg
            keep_for_spacing = distance_from_last >= max_straight_spacing_m

            if keep_for_turn or keep_for_spacing:
                simplified.append(p_cur)
                last_kept = i

        if simplified[-1] != path_xy[-1]:
            simplified.append(path_xy[-1])

        return simplified

    def _enforce_waypoint_limit(self,
                                path_xy: list,
                                max_waypoints: int,
                                turn_threshold_deg: float) -> list:
        """Limit waypoint count while preserving endpoints and heading-change keypoints."""
        n = len(path_xy)
        if n <= max_waypoints:
            return path_xy

        key_indices = {0, n - 1}
        for i in range(1, n - 1):
            h1 = self._compute_heading(
                path_xy[i - 1][0], path_xy[i - 1][1],
                path_xy[i][0], path_xy[i][1]
            )
            h2 = self._compute_heading(
                path_xy[i][0], path_xy[i][1],
                path_xy[i + 1][0], path_xy[i + 1][1]
            )
            if self._heading_delta_deg(h1, h2) >= turn_threshold_deg:
                key_indices.add(i)

        key_sorted = sorted(key_indices)

        if len(key_sorted) > max_waypoints:
            selected = [0]
            interior = key_sorted[1:-1]
            slots = max_waypoints - 2
            if slots > 0 and interior:
                stride = len(interior) / float(slots)
                for k in range(slots):
                    idx = int(round(k * stride))
                    idx = min(max(idx, 0), len(interior) - 1)
                    selected.append(interior[idx])
            selected.append(n - 1)
            selected = sorted(set(selected))
            return [path_xy[i] for i in selected]

        remaining_slots = max_waypoints - len(key_sorted)
        candidates = [i for i in range(n) if i not in key_indices]
        chosen_extra = []
        if remaining_slots > 0 and candidates:
            stride = len(candidates) / float(remaining_slots + 1)
            for k in range(1, remaining_slots + 1):
                idx = int(round(k * stride)) - 1
                idx = min(max(idx, 0), len(candidates) - 1)
                chosen_extra.append(candidates[idx])

        final_indices = sorted(set(key_sorted + chosen_extra))
        return [path_xy[i] for i in final_indices]

    def _count_required_waypoints(self,
                                  path_xy: list,
                                  turn_threshold_deg: float) -> int:
        """Count start/end plus heading-change keypoints needed to preserve path shape."""
        n = len(path_xy)
        if n <= 2:
            return n

        key_indices = {0, n - 1}
        for i in range(1, n - 1):
            h1 = self._compute_heading(
                path_xy[i - 1][0], path_xy[i - 1][1],
                path_xy[i][0], path_xy[i][1]
            )
            h2 = self._compute_heading(
                path_xy[i][0], path_xy[i][1],
                path_xy[i + 1][0], path_xy[i + 1][1]
            )
            if self._heading_delta_deg(h1, h2) >= turn_threshold_deg:
                key_indices.add(i)

        return len(key_indices)
    
    @staticmethod
    def _compute_heading(x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Compute heading (bearing) from (x1,y1) to (x2,y2)
        
        Returns heading in degrees (0° = North, 90° = East, 180° = South, 270° = West)
        """
        dx = x2 - x1
        dy = y2 - y1
        
        # atan2(dy, dx) gives angle from East; heading uses North as reference
        # Convert from math angle to compass heading
        heading = math.degrees(math.atan2(dx, dy))  # atan2(East, North)
        
        # Normalize to 0-360 range
        heading = heading % 360
        return heading
    
    @staticmethod
    def _compute_polygon_area(polygon: list) -> float:
        """Compute area of polygon using Shoelace formula"""
        if len(polygon) < 3:
            return 0
        
        area = 0
        for i in range(len(polygon)):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % len(polygon)]
            area += x1 * y2 - x2 * y1
        
        return abs(area) / 2
    
    @staticmethod
    def _compute_transition_distance(path: list) -> float:
        """Estimate distance spent in transitions (non-sweep segments)"""
        # This is approximate - counts segments where heading changes > 45 degrees
        if len(path) < 2:
            return 0
        
        transition_dist = 0
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            distance = math.hypot(x2 - x1, y2 - y1)
            
            # Simple heuristic: if next segment exists, check heading change
            if i < len(path) - 2:
                x3, y3 = path[i + 2]
                h1 = math.degrees(math.atan2(x2 - x1, y2 - y1))
                h2 = math.degrees(math.atan2(x3 - x2, y3 - y2))
                heading_change = abs(h2 - h1) % 360
                
                if heading_change > 45 and heading_change < 315:
                    transition_dist += distance
        
        return transition_dist


# ============================================================================
# Example Usage
# ============================================================================

if __name__ == '__main__':
    # Test with simple boundary
    boundary = [
        [0, 0],
        [0, 10],
        [10, 10],
        [10, 0]
    ]
    
    obstacles = [
        [
            [3, 3],
            [3, 7],
            [7, 7],
            [7, 3]
        ]
    ]
    
    service = CoveragePathService()
    result = service.plan_coverage(
        boundary_latlon=boundary,
        obstacles_latlon=obstacles,
        row_spacing=0.5,
        safety_margin=0.3,
        sweep_angle=0,
        robot_speed=1.0
    )
    
    print(f"Success: {result['success']}")
    if result['success']:
        print(f"Waypoints: {len(result['waypoints'])}")
        print(f"Total Distance: {result['total_distance']} m")
        print(f"Estimated Time: {result['estimated_time']} s")
        print(f"Cells: {result['cells_count']}")
        print(f"Statistics: {result['statistics']}")
