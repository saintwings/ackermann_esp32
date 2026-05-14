import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import matplotlib.cm as cm
import heapq
import itertools
from matplotlib.path import Path

# ==========================================
# 1. GEOMETRY UTILITIES
# ==========================================
class Geometry:
    @staticmethod
    def get_line_intersection(p1, p2, p3, p4):
        x1, y1 = p1; x2, y2 = p2
        x3, y3 = p3; x4, y4 = p4
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-8: return None
        px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom
        py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom
        return px, py

    @staticmethod
    def rotate_point(x, y, angle_deg, origin=(5, 5)):
        angle_rad = np.radians(angle_deg)
        cx, cy = origin
        cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
        nx = cos_a * (x - cx) - sin_a * (y - cy) + cx
        ny = sin_a * (x - cx) + cos_a * (y - cy) + cy
        return nx, ny

    @staticmethod
    def offset_polygon(poly, margin):
        if margin == 0: return poly
        shifted_edges = []
        n = len(poly)
        for i in range(n):
            p1, p2 = poly[i], poly[(i + 1) % n]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            length = np.hypot(dx, dy)
            if length == 0: continue
            nx, ny = dy / length, -dx / length
            sp1 = (p1[0] + nx * margin, p1[1] + ny * margin)
            sp2 = (p2[0] + nx * margin, p2[1] + ny * margin)
            shifted_edges.append((sp1, sp2))
            
        new_poly = []
        for i in range(len(shifted_edges)):
            e1 = shifted_edges[i - 1]; e2 = shifted_edges[i]     
            intersect = Geometry.get_line_intersection(e1[0], e1[1], e2[0], e2[1])
            new_poly.append(intersect if intersect else e2[0])
        return new_poly

    @staticmethod
    def get_ray_intersections(x, poly):
        ints = []
        for i in range(len(poly)):
            p1, p2 = poly[i], poly[(i+1) % len(poly)]
            if min(p1[0], p2[0]) <= x <= max(p1[0], p2[0]):
                if abs(p2[0] - p1[0]) > 1e-6:
                    y = p1[1] + (x - p1[0]) * (p2[1] - p1[1]) / (p2[0] - p1[0])
                    ints.append(y)
                else:
                    ints.extend([p1[1], p2[1]])
        return sorted(list(set(np.round(ints, 4))))

# ==========================================
# 2. ENVIRONMENT & CELL DEFINITION
# ==========================================
class MapEnvironment:
    def __init__(self, boundary, obstacles, margin, sweep_angle, origin=(5, 5)):
        self.margin = margin
        self.sweep_angle = sweep_angle
        self.origin = origin
        
        self.raw_boundary = boundary
        self.raw_obstacles = obstacles
        self.safe_boundary = Geometry.offset_polygon(boundary, -margin)
        self.safe_obstacles = [Geometry.offset_polygon(obs, margin) for obs in obstacles]
        
        self.rot_boundary = [Geometry.rotate_point(x, y, -sweep_angle, origin) for x, y in self.safe_boundary]
        self.rot_obstacles = [[Geometry.rotate_point(x, y, -sweep_angle, origin) for x, y in obs] for obs in self.safe_obstacles]
        
        self.b_path = Path(self.rot_boundary)
        self.o_paths = [Path(o) for o in self.rot_obstacles]
        
        # SPEED OPTIMIZATION: Pre-calculate Axis-Aligned Bounding Boxes (AABB) for obstacles
        self.o_bboxes = []
        for obs in self.rot_obstacles:
            xs = [p[0] for p in obs]; ys = [p[1] for p in obs]
            self.o_bboxes.append((min(xs), max(xs), min(ys), max(ys)))

class BoustrophedonCell:
    def __init__(self, cell_id, x_min, x_max, k_index):
        self.id = cell_id
        self.x_min = x_min
        self.x_max = x_max
        self.k_index = k_index
        self.paths = [] 
        self.real_polygon = [] 

# ==========================================
# 3. COVERAGE PLANNER (A*, True TSP)
# ==========================================
class CoveragePlanner:
    def __init__(self, env):
        self.env = env
        self.cells = []

    def _get_free_intervals(self, x):
        bound_ints = Geometry.get_ray_intersections(x, self.env.rot_boundary)
        if len(bound_ints) < 2: return []
        y_min, y_max = bound_ints[0], bound_ints[-1]
        
        obs_ints = []
        for obs in self.env.rot_obstacles:
            obs_ints.extend(Geometry.get_ray_intersections(x, obs))
            
        valid_obs = sorted(list(set([y for y in obs_ints if y_min < y < y_max])))
        intervals = []
        curr_y = y_min
        for i in range(0, len(valid_obs)-1, 2):
            if valid_obs[i] > curr_y: intervals.append((curr_y, valid_obs[i]))
            curr_y = valid_obs[i+1]
        if curr_y < y_max: intervals.append((curr_y, y_max))
        return intervals

    def decompose_environment(self):
        xs = []
        for geom in [self.env.rot_boundary] + self.env.rot_obstacles:
            for p in geom: xs.append(p[0])
        critical_points = sorted(list(set(np.round(xs, 4))))

        cell_id = 0
        for i in range(len(critical_points) - 1):
            x_min, x_max = critical_points[i], critical_points[i+1]
            if x_max - x_min < 0.01: continue 
            
            intervals = self._get_free_intervals((x_min + x_max) / 2)
            for k in range(len(intervals)):
                cell = BoustrophedonCell(cell_id, x_min, x_max, k)
                
                int_l = self._get_free_intervals(x_min + 1e-4)
                int_r = self._get_free_intervals(x_max - 1e-4)
                if k < len(int_l) and k < len(int_r):
                    y_bl, y_tl = int_l[k]
                    y_br, y_tr = int_r[k]
                    poly_points = [(x_min, y_bl), (x_max, y_br), (x_max, y_tr), (x_min, y_tl)]
                    cell.real_polygon = [Geometry.rotate_point(px, py, self.env.sweep_angle, self.env.origin) for px, py in poly_points]
                
                self.cells.append(cell)
                cell_id += 1

    def generate_cell_sweeps(self, row_spacing):
        for cell in self.cells:
            x_sweeps = np.arange(cell.x_min + row_spacing/2, cell.x_max, row_spacing)
            columns = []
            
            for x in x_sweeps:
                intervals = self._get_free_intervals(x)
                if cell.k_index < len(intervals):
                    columns.append((x, intervals[cell.k_index][0], intervals[cell.k_index][1]))
            
            cell.paths = []
            if not columns: continue
            
            p1 = []; d = 1
            for x, yb, yt in columns:
                if d == 1: p1.extend([(x, yb), (x, yt)])
                else: p1.extend([(x, yt), (x, yb)])
                d *= -1
            cell.paths.append(p1)
            
            p2 = []; d = -1
            for x, yb, yt in columns:
                if d == 1: p2.extend([(x, yb), (x, yt)])
                else: p2.extend([(x, yt), (x, yb)])
                d *= -1
            cell.paths.append(p2)
            
            p3 = []; d = 1
            for x, yb, yt in reversed(columns):
                if d == 1: p3.extend([(x, yb), (x, yt)])
                else: p3.extend([(x, yt), (x, yb)])
                d *= -1
            cell.paths.append(p3)
            
            p4 = []; d = -1
            for x, yb, yt in reversed(columns):
                if d == 1: p4.extend([(x, yb), (x, yt)])
                else: p4.extend([(x, yt), (x, yb)])
                d *= -1
            cell.paths.append(p4)

    def _is_edge_safe(self, p1, p2, steps=4):
        # SPEED OPTIMIZATION: Reduced steps and implemented AABB fast-rejection
        for t in np.linspace(0, 1, steps):
            pt = (p1[0] + t*(p2[0]-p1[0]), p1[1] + t*(p2[1]-p1[1]))
            if not self.env.b_path.contains_point(pt, radius=0.05): return False
            
            for i, o_path in enumerate(self.env.o_paths):
                minx, maxx, miny, maxy = self.env.o_bboxes[i]
                # Fast AABB check: Only do heavy polygon math if inside the bounding box
                if minx - 0.1 <= pt[0] <= maxx + 0.1 and miny - 0.1 <= pt[1] <= maxy + 0.1:
                    if o_path.contains_point(pt, radius=-0.05): return False
        return True

    def _a_star_transit(self, start, goal, grid_res=0.25):
        start_node = (round(start[0], 4), round(start[1], 4))
        goal_node = (round(goal[0], 4), round(goal[1], 4))
        open_set = []; heapq.heappush(open_set, (0, start_node))
        came_from = {}; g_score = {start_node: 0}
        moves = [(0, grid_res), (0, -grid_res), (grid_res, 0), (-grid_res, 0),
                 (grid_res, grid_res), (-grid_res, grid_res), (grid_res, -grid_res), (-grid_res, -grid_res)]
                 
        nodes = 0
        while open_set and nodes < 2000: # SPEED OPTIMIZATION: Lowered safety cap
            _, current = heapq.heappop(open_set)
            nodes += 1
            if np.hypot(current[0]-goal_node[0], current[1]-goal_node[1]) <= grid_res * 1.5:
                if self._is_edge_safe(current, goal_node, steps=8):
                    path = [goal]
                    while current in came_from:
                        path.append(current)
                        current = came_from[current]
                    path.append(start)
                    raw_path = path[::-1]
                    
                    if len(raw_path) <= 2: return raw_path
                    smooth = [raw_path[0]]; curr_idx = 0
                    while curr_idx < len(raw_path) - 1:
                        furthest = curr_idx + 1
                        for check_idx in range(len(raw_path) - 1, curr_idx, -1):
                            dist = np.hypot(raw_path[check_idx][0]-raw_path[curr_idx][0], raw_path[check_idx][1]-raw_path[curr_idx][1])
                            if self._is_edge_safe(raw_path[curr_idx], raw_path[check_idx], steps=max(int(dist/0.2), 3)):
                                furthest = check_idx; break
                        smooth.append(raw_path[furthest])
                        curr_idx = furthest
                    return smooth
                    
            for dx, dy in moves:
                nb = (round(current[0] + dx, 4), round(current[1] + dy, 4))
                if not self._is_edge_safe(current, nb): continue
                tentative_g = g_score[current] + np.hypot(dx, dy)
                if nb not in g_score or tentative_g < g_score[nb]:
                    came_from[nb] = current; g_score[nb] = tentative_g
                    heapq.heappush(open_set, (tentative_g + np.hypot(nb[0]-goal_node[0], nb[1]-goal_node[1]), nb))
        return [start, goal]

    def plan_path(self, row_spacing=0.25):
        self.decompose_environment()
        self.generate_cell_sweeps(row_spacing)
        
        valid_cells = [c for c in self.cells if len(c.paths) > 0 and len(c.paths[0]) > 0]
        if not valid_cells: return []

        transit_cache = {}
        # SPEED OPTIMIZATION: Coarse grid for fast TSP estimates
        coarse_res = row_spacing * 2.0 
        
        def get_transit_cost(p1, p2):
            k = (round(p1[0],2), round(p1[1],2), round(p2[0],2), round(p2[1],2))
            k_rev = (round(p2[0],2), round(p2[1],2), round(p1[0],2), round(p1[1],2))
            if k in transit_cache: return transit_cache[k]
            if k_rev in transit_cache: return transit_cache[k_rev]
            
            t_path = self._a_star_transit(p1, p2, grid_res=coarse_res)
            cost = sum(np.hypot(t_path[i+1][0]-t_path[i][0], t_path[i+1][1]-t_path[i][1]) for i in range(len(t_path)-1))
            transit_cache[k] = cost
            return cost

        best_cost = float('inf')
        best_sequence = []
        
        # SPEED OPTIMIZATION: Lowered Exact TSP cap to 6 (720 permutations)
        if len(valid_cells) <= 6:
            for perm in itertools.permutations(valid_cells):
                for first_path_idx in range(4):
                    curr_cost = 0
                    curr_seq = [(perm[0], first_path_idx)]
                    curr_pos = perm[0].paths[first_path_idx][-1] 
                    
                    for cell in perm[1:]:
                        best_path_cost = float('inf')
                        best_path_idx = 0
                        
                        for p_idx in range(4):
                            cost = get_transit_cost(curr_pos, cell.paths[p_idx][0])
                            if cost < best_path_cost:
                                best_path_cost = cost
                                best_path_idx = p_idx
                                
                        curr_cost += best_path_cost
                        curr_seq.append((cell, best_path_idx))
                        curr_pos = cell.paths[best_path_idx][-1]
                        
                    if curr_cost < best_cost:
                        best_cost = curr_cost; best_sequence = curr_seq
        else:
            # Greedy Fallback
            unvisited = valid_cells.copy()
            current_cell = unvisited.pop(0)
            best_sequence = [(current_cell, 0)] 
            curr_pos = current_cell.paths[0][-1]
            
            while unvisited:
                best_next_cell = None
                best_path_idx = 0
                best_path_cost = float('inf')
                
                for cell in unvisited:
                    for p_idx in range(4):
                        cost = get_transit_cost(curr_pos, cell.paths[p_idx][0])
                        if cost < best_path_cost:
                            best_path_cost = cost
                            best_path_idx = p_idx
                            best_next_cell = cell
                            
                best_sequence.append((best_next_cell, best_path_idx))
                curr_pos = best_next_cell.paths[best_path_idx][-1]
                unvisited.remove(best_next_cell)

        # Assemble Global Path (Using fine-grained row_spacing for actual final drawing)
        flat_rotated_path = []
        for cell, path_idx in best_sequence:
            c_path = cell.paths[path_idx]
            if flat_rotated_path:
                transit = self._a_star_transit(flat_rotated_path[-1], c_path[0], grid_res=row_spacing)
                flat_rotated_path.extend(transit)
            flat_rotated_path.extend(c_path)

        real_world_path = [Geometry.rotate_point(px, py, self.env.sweep_angle, self.env.origin) for px, py in flat_rotated_path]

        if not real_world_path: return []

        smooth_path = []
        step_distance = 0.15
        for i in range(len(real_world_path)-1):
            p1, p2 = np.array(real_world_path[i]), np.array(real_world_path[i+1])
            num_steps = max(int(np.linalg.norm(p2 - p1) / step_distance), 1)
            for t in np.linspace(0, 1, num_steps, endpoint=False):
                smooth_path.append((p1 + t * (p2 - p1)).tolist())
        smooth_path.append(real_world_path[-1])
        
        return smooth_path

# ==========================================
# 4. VISUALIZER
# ==========================================
class Visualizer:
    def __init__(self, env, planner, drive_path):
        self.env = env
        self.planner = planner
        self.drive_path = drive_path

    def animate(self):
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_title(f"Coverage Planner\nMargin: {self.env.margin}", fontweight='bold')
        ax.set_aspect('equal')
        ax.set_xlim(-1, 11); ax.set_ylim(-1, 11)

        ax.add_patch(patches.Polygon(self.env.raw_boundary, closed=True, fill=False, edgecolor='green', linewidth=3, linestyle='--'))
        ax.add_patch(patches.Polygon(self.env.safe_boundary, closed=True, fill=False, edgecolor='black', linewidth=1.5))
        for raw, safe in zip(self.env.raw_obstacles, self.env.safe_obstacles):
            ax.add_patch(patches.Polygon(safe, closed=True, color='darkgray', alpha=0.5))
            ax.add_patch(patches.Polygon(raw, closed=True, color='red', zorder=3))

        colors = cm.Pastel1(np.linspace(0, 1, max(len(self.planner.cells), 1)))
        for idx, cell in enumerate(self.planner.cells):
            if not cell.real_polygon: continue
            ax.add_patch(patches.Polygon(cell.real_polygon, closed=True, fill=True, color=colors[idx % len(colors)], alpha=0.5, edgecolor='gray', zorder=1))
            xs = [p[0] for p in cell.real_polygon]; ys = [p[1] for p in cell.real_polygon]
            ax.text(np.mean(xs), np.mean(ys), f"Cell {cell.id}", fontsize=11, ha='center', va='center', fontweight='bold', zorder=6)

        line, = ax.plot([], [], color='blue', linewidth=1.5, zorder=4, alpha=0.6)
        robot = patches.Circle((0, 0), radius=self.env.margin, color='orange', zorder=5, alpha=0.8, ec='black')
        ax.add_patch(robot)

        if self.drive_path:
            ax.plot(self.drive_path[0][0], self.drive_path[0][1], 'o', color='lime', markersize=8, markeredgecolor='black', zorder=2)
            ax.plot(self.drive_path[-1][0], self.drive_path[-1][1], 'X', color='red', markersize=8, markeredgecolor='black', zorder=2)

        def update(frame):
            if not self.drive_path: return line, robot
            line.set_data([p[0] for p in self.drive_path[:frame+1]], [p[1] for p in self.drive_path[:frame+1]])
            robot.center = self.drive_path[frame]
            return line, robot

        ani = animation.FuncAnimation(fig, update, frames=len(self.drive_path), blit=True, interval=15, repeat=False)
        plt.tight_layout()
        plt.show()

# ==========================================
# 5. MAIN EXECUTION
# ==========================================
if __name__ == '__main__':
    raw_boundary = [(0, 0), (10, 0), (10, 8), (5, 10), (0, 8)]
    # raw_obstacles = [[(3, 3), (6, 3), (6, 5), (3, 5)],
    #                  [(7, 6), (8, 5), (9, 7)]]
    raw_obstacles = []
    
    env = MapEnvironment(raw_boundary, raw_obstacles, margin=0.3, sweep_angle=15)
    planner = CoveragePlanner(env)
    
    path = planner.plan_path(row_spacing=1)
    
    vis = Visualizer(env, planner, path)
    vis.animate()