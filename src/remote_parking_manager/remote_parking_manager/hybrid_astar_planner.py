"""
hybrid_astar_planner.py
Hybrid A* with Reeds-Shepp curves for LIMO ackermann parking.
"""

import math
import heapq
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

TWO_PI = 2.0 * math.pi


def _mod2pi(x: float) -> float:
    return x - TWO_PI * math.floor(x / TWO_PI)


def _normalize(a: float) -> float:
    while a > math.pi:
        a -= TWO_PI
    while a < -math.pi:
        a += TWO_PI
    return a


# ── Reeds-Shepp: 4-transformation approach ─────────────────────
# 6 base formulas × 4 transformations = 24 distinct RS word types.
# Base formulas compute segment lengths assuming all-forward.
# Transformations:
#   original:          (x,  y,  phi)  → all forward
#   timeflip:          (-x, y, -phi)  → all backward
#   reflect:           (x, -y, -phi)  → swap L↔R
#   timeflip+reflect:  (-x,-y,  phi)  → swap L↔R + all backward

def _LSL(x, y, phi):
    u, t = math.hypot(x - math.sin(phi), y - 1 + math.cos(phi)), \
           math.atan2(y - 1 + math.cos(phi), x - math.sin(phi))
    v = _mod2pi(phi - t)
    t = _mod2pi(t)
    return t, u, v

def _LSR(x, y, phi):
    x1 = x + math.sin(phi)
    y1 = y - 1 - math.cos(phi)
    d2 = x1*x1 + y1*y1
    if d2 < 4:
        return None
    u = math.sqrt(d2 - 4)
    theta = math.atan2(2, u)
    t = _mod2pi(math.atan2(y1, x1) + theta)
    v = _mod2pi(t - phi)
    return t, u, v

def _LRL(x, y, phi):
    x1 = x - math.sin(phi)
    y1 = y - 1 + math.cos(phi)
    d2 = x1*x1 + y1*y1
    if d2 > 16 or d2 < 1e-10:
        return None
    cval = (6 - d2) / 8
    if abs(cval) > 1:
        return None
    u = _mod2pi(TWO_PI - math.acos(cval))
    a_val = (d2 + 2) / (4 * math.sqrt(d2))
    if abs(a_val) > 1:
        return None
    t = _mod2pi(math.atan2(y1, x1) - math.acos(a_val) + math.pi/2)
    v = _mod2pi(phi - t + u)
    return t, u, v


def _simulate_word(word, lengths, sx, sy, syaw, r, directions, step=0.02):
    """Simulate path for a given RS word. Returns [(x,y,yaw,dir), ...]."""
    path = []
    x, y, yaw = sx, sy, syaw

    for ch, length, d in zip(word, lengths, directions):
        if length < 1e-10:
            continue
        if ch == 'S':
            dist = length * r * d
            n = max(1, int(abs(dist) / step))
            for i in range(1, n + 1):
                f = i / n
                path.append((x + dist*f*math.cos(yaw),
                              y + dist*f*math.sin(yaw),
                              yaw, d))
            x += dist * math.cos(yaw)
            y += dist * math.sin(yaw)
        else:  # L or R
            sign = 1.0 if ch == 'L' else -1.0
            angle = length * d  # signed angle
            n = max(1, int(length * r / step))
            ocx = x - sign * r * math.sin(yaw)
            ocy = y + sign * r * math.cos(yaw)
            a0 = math.atan2(y - ocy, x - ocx)
            for i in range(1, n + 1):
                f = i / n
                a = a0 + sign * angle * f
                px = ocx + r * math.cos(a)
                py = ocy + r * math.sin(a)
                pyaw = yaw + sign * angle * f
                path.append((px, py, pyaw, d))
            yaw += sign * angle
            x = ocx + r * math.cos(a0 + sign * angle)
            y = ocy + r * math.sin(a0 + sign * angle)
    return path


def reeds_shepp_path(sx, sy, syaw, gx, gy, gyaw, r, step=0.02):
    """Find shortest collision-free RS path. Returns [(x,y,yaw,dir),...] or None."""
    cos_s, sin_s = math.cos(syaw), math.sin(syaw)
    dx, dy = gx - sx, gy - sy
    lx = (cos_s * dx + sin_s * dy) / r
    ly = (-sin_s * dx + cos_s * dy) / r
    lphi = _normalize(gyaw - syaw)

    # base_formula → (word, formula_func)
    bases = [
        ('LSL', _LSL), ('LSR', _LSR), ('LRL', _LRL),
    ]

    # 4 transformations × input transform, direction, L↔R swap
    transforms = [
        # (x_sign, y_sign, phi_sign, dir, swap_lr)
        ( 1,  1,  1,  1, False),  # original
        (-1,  1, -1, -1, False),  # timeflip
        ( 1, -1, -1,  1, True),   # reflect
        (-1, -1,  1, -1, True),   # timeflip + reflect
    ]

    best = None
    best_len = float('inf')

    for word, formula in bases:
        for xs, ys, ps, d, swap in transforms:
            res = formula(lx * xs, ly * ys, lphi * ps)
            if res is None:
                continue
            t, u, v = res

            # Build actual word and lengths
            actual_word = word
            if swap:
                actual_word = actual_word.replace('L', 'X').replace('R', 'L').replace('X', 'R')

            total = (t + u + v) * r
            if total >= best_len:
                continue

            directions = []
            for ch in actual_word:
                directions.append(d)

            path = _simulate_word(actual_word, (t, u, v),
                                  sx, sy, syaw, r, directions, step)
            if not path:
                continue

            last = path[-1]
            err = math.hypot(last[0] - gx, last[1] - gy)
            yerr = abs(_normalize(last[2] - gyaw))
            if err < 0.05 and yerr < 0.1:
                best = path
                best_len = total

    return best


# ── Occupancy Grid ──────────────────────────────────────────────

class OccupancyGrid:
    def __init__(self, x_min, x_max, y_min, y_max, resolution):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.resolution = resolution
        self.nx = int(math.ceil((x_max - x_min) / resolution))
        self.ny = int(math.ceil((y_max - y_min) / resolution))
        self.grid = np.zeros((self.nx, self.ny), dtype=np.uint8)

    def world_to_grid(self, x, y):
        return int((x - self.x_min) / self.resolution), \
               int((y - self.y_min) / self.resolution)

    def set_obstacle_rect(self, cx, cy, sx, sy, yaw=0.0):
        cos_a, sin_a = math.cos(yaw), math.sin(yaw)
        hs, hw = sx / 2, sy / 2
        corners = [(cx + cos_a*dx - sin_a*dy, cy + sin_a*dx + cos_a*dy)
                    for dx, dy in [(-hs,-hw),(hs,-hw),(hs,hw),(-hs,hw)]]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        r = self.resolution
        for x in np.arange(max(self.x_min, min(xs)-r), min(self.x_max, max(xs)+r), r):
            for y in np.arange(max(self.y_min, min(ys)-r), min(self.y_max, max(ys)+r), r):
                lx = cos_a*(x-cx) + sin_a*(y-cy)
                ly = -sin_a*(x-cx) + cos_a*(y-cy)
                if abs(lx) <= hs and abs(ly) <= hw:
                    ix, iy = self.world_to_grid(x, y)
                    if 0 <= ix < self.nx and 0 <= iy < self.ny:
                        self.grid[ix, iy] = 1

    def is_occupied(self, x, y):
        if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
            return True
        ix, iy = self.world_to_grid(x, y)
        if ix < 0 or ix >= self.nx or iy < 0 or iy >= self.ny:
            return True
        return self.grid[ix, iy] > 0

    def check_footprint(self, cx, cy, yaw, length, width):
        cos_a, sin_a = math.cos(yaw), math.sin(yaw)
        hl, hw = length / 2, width / 2
        step = self.resolution
        for dl in np.arange(-hl, hl + step, step):
            for dw in np.arange(-hw, hw + step, step):
                if self.is_occupied(cx + cos_a*dl - sin_a*dw,
                                    cy + sin_a*dl + cos_a*dw):
                    return False
        return True


def build_parking_grid(empty_slot_x=0.225):
    """Build occupancy grid for the remote parking lot."""
    grid = OccupancyGrid(x_min=-1.5, x_max=1.2, y_min=-0.6, y_max=1.7,
                         resolution=0.02)
    cl, cw = 0.40, 0.26  # car + margin

    for cx in [-1.125, -0.675, -0.225, 0.225, 0.675]:  # A0, A1, A2, A3, A4
        if abs(cx - empty_slot_x) > 0.05:
            grid.set_obstacle_rect(cx, 0.985, cl, cw, -math.pi/2)

    grid.set_obstacle_rect(0.05, 1.585, 2.2, 0.10)  # north wall
    grid.set_obstacle_rect(-1.90, 0.0, 0.10, 3.17)   # west wall (actual position)

    return grid


# ── Hybrid A* ───────────────────────────────────────────────────

@dataclass(order=True)
class HANode:
    f: float
    g: float = field(compare=False)
    x: float = field(compare=False)
    y: float = field(compare=False)
    yaw: float = field(compare=False)
    direction: int = field(compare=False)
    parent_idx: int = field(compare=False)
    idx: int = field(compare=False)


class HybridAStarPlanner:
    def __init__(self, grid, wheelbase=0.24, min_turn_radius=0.42,
                 vehicle_length=0.31, vehicle_width=0.19,
                 xy_resolution=0.03, yaw_resolution_deg=5.0,
                 step_size=0.06, steer_angles_deg=None,
                 rs_interval=5):
        self.grid = grid
        self.wheelbase = wheelbase
        self.min_r = min_turn_radius
        self.veh_l = vehicle_length
        self.veh_w = vehicle_width
        self.xy_res = xy_resolution
        self.yaw_res = math.radians(yaw_resolution_deg)
        self.n_yaw = int(round(TWO_PI / self.yaw_res))
        self.step = step_size
        self.rs_interval = rs_interval
        self.collision_margin = 0.02

        if steer_angles_deg is None:
            steer_angles_deg = [-30, -15, 0, 15, 30]
        self.steer_angles = [math.radians(a) for a in steer_angles_deg]

    def _grid_index(self, x, y, yaw):
        ix = int(round((x - self.grid.x_min) / self.xy_res))
        iy = int(round((y - self.grid.y_min) / self.xy_res))
        iyaw = int(round(_mod2pi(yaw) / self.yaw_res)) % self.n_yaw
        return (ix, iy, iyaw)

    def _heuristic(self, x, y, yaw, gx, gy, gyaw):
        d = math.hypot(gx - x, gy - y)
        yaw_diff = abs(_normalize(gyaw - yaw))
        return d + 0.5 * self.min_r * yaw_diff

    def _is_free(self, x, y, yaw):
        return self.grid.check_footprint(
            x, y, yaw,
            self.veh_l + self.collision_margin * 2,
            self.veh_w + self.collision_margin * 2)

    def _simulate_step(self, x, y, yaw, steer, direction):
        d = self.step * direction
        if abs(steer) < 1e-6:
            return x + d*math.cos(yaw), y + d*math.sin(yaw), _mod2pi(yaw)
        beta = d * math.tan(steer) / self.wheelbase
        return (x + d*math.cos(yaw + beta/2),
                y + d*math.sin(yaw + beta/2),
                _mod2pi(yaw + beta))

    def _try_rs(self, x, y, yaw, gx, gy, gyaw):
        path = reeds_shepp_path(x, y, yaw, gx, gy, gyaw, self.min_r)
        if path is None:
            return None
        for px, py, pyaw, _ in path:
            if not self._is_free(px, py, pyaw):
                return None
        return path

    def plan(self, sx, sy, syaw, gx, gy, gyaw, max_iterations=80000):
        syaw, gyaw = _mod2pi(syaw), _mod2pi(gyaw)

        if not self._is_free(sx, sy, syaw):
            return None
        if not self._is_free(gx, gy, gyaw):
            return None

        rs_direct = self._try_rs(sx, sy, syaw, gx, gy, gyaw)
        if rs_direct is not None:
            return rs_direct

        start_h = self._heuristic(sx, sy, syaw, gx, gy, gyaw)
        start_node = HANode(f=start_h, g=0, x=sx, y=sy, yaw=syaw,
                            direction=1, parent_idx=-1, idx=0)
        open_heap = [start_node]
        all_nodes = [start_node]
        closed = {self._grid_index(sx, sy, syaw)}

        goal_node_idx = -1
        rs_suffix = None

        for count in range(1, max_iterations + 1):
            if not open_heap:
                break
            current = heapq.heappop(open_heap)

            if count % self.rs_interval == 0:
                rs = self._try_rs(current.x, current.y, current.yaw, gx, gy, gyaw)
                if rs is not None:
                    goal_node_idx = current.idx
                    rs_suffix = rs
                    break

            if (math.hypot(gx - current.x, gy - current.y) < 0.04 and
                    abs(_normalize(gyaw - current.yaw)) < math.radians(8)):
                goal_node_idx = current.idx
                break

            for steer in self.steer_angles:
                for direction in (1, -1):
                    nx, ny, nyaw = self._simulate_step(
                        current.x, current.y, current.yaw, steer, direction)
                    gi = self._grid_index(nx, ny, nyaw)
                    if gi in closed:
                        continue
                    if not self._is_free(nx, ny, nyaw):
                        continue

                    cost = self.step
                    if direction == -1:
                        cost *= 2.5
                    if abs(steer) > 1e-6:
                        cost *= 1.1
                    if direction != current.direction:
                        cost += 0.3

                    ng = current.g + cost
                    nf = ng + self._heuristic(nx, ny, nyaw, gx, gy, gyaw)
                    new_idx = len(all_nodes)
                    node = HANode(f=nf, g=ng, x=nx, y=ny, yaw=nyaw,
                                  direction=direction, parent_idx=current.idx,
                                  idx=new_idx)
                    all_nodes.append(node)
                    closed.add(gi)
                    heapq.heappush(open_heap, node)

        if goal_node_idx < 0:
            return None

        path = []
        idx = goal_node_idx
        while idx >= 0:
            n = all_nodes[idx]
            path.append((n.x, n.y, n.yaw, n.direction))
            idx = n.parent_idx
        path.reverse()

        if rs_suffix:
            path.extend(rs_suffix)
        return path


def plan_repark_path(start_x, start_y, start_yaw,
                     goal_x=0.225, goal_y=0.985,
                     goal_yaw=math.pi / 2):
    grid = build_parking_grid(empty_slot_x=goal_x)
    planner = HybridAStarPlanner(grid)
    return planner.plan(start_x, start_y, start_yaw,
                        goal_x, goal_y, goal_yaw)
