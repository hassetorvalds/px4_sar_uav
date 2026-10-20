"""观测覆盖网格（纯逻辑）。

相机点云（ENU）落到地面网格上，标记"看过的格子"；覆盖率和漏扫数只根据
真实点云计算，不用相机 FOV 估计。
"""

import math
import os
import struct
import zlib

import numpy as np


def point_in_polygon(east: float, north: float, polygon) -> bool:
    """射线法；polygon 为 [(east, north), ...]，首尾不必重复。"""
    if len(polygon) < 3:
        return False
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > north) != (yj > north)) and (
                east < (xj - xi) * (north - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def polygon_area(polygon) -> float:
    if len(polygon) < 3:
        return 0.0
    total = 0.0
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        total += xj * yi - xi * yj
        j = i
    return abs(total) / 2.0


def polygon_bounds(polygon):
    xs = [float(p[0]) for p in polygon]
    ys = [float(p[1]) for p in polygon]
    return min(xs), min(ys), max(xs), max(ys)


class CoverageGrid:
    def __init__(self, polygon, cell_size_m: float = 1.0,
                 ground_tol_m: float = 0.6):
        self.cell_size = float(cell_size_m)
        if self.cell_size <= 0:
            raise ValueError('cell_size_m 必须 > 0')
        self.ground_tol = float(ground_tol_m)
        self.polygon = []
        self.cells = set()
        self.observed = set()
        self.points_total = 0
        self.ground_points = 0
        self.points_in_polygon = 0
        self.reset(polygon)

    def reset(self, polygon):
        polygon = [(float(e), float(n)) for e, n in polygon]
        if len(polygon) < 3:
            raise ValueError('polygon 至少 3 个顶点')
        self.polygon = polygon
        self.cells = set()
        self.observed = set()
        self.points_total = 0
        self.ground_points = 0
        self.points_in_polygon = 0
        min_e, min_n, max_e, max_n = polygon_bounds(polygon)
        size = self.cell_size
        ix0 = math.floor(min_e / size)
        ix1 = math.ceil(max_e / size)
        iy0 = math.floor(min_n / size)
        iy1 = math.ceil(max_n / size)
        for ix in range(ix0, ix1 + 1):
            east = (ix + 0.5) * size
            for iy in range(iy0, iy1 + 1):
                north = (iy + 0.5) * size
                if point_in_polygon(east, north, polygon):
                    self.cells.add((ix, iy))

    def observe(self, points_enu) -> None:
        """points_enu: (N,3) ENU 点；只把近地点落在多边形格子里。"""
        points = np.asarray(points_enu, dtype=float)
        if points.size == 0:
            return
        if points.ndim != 2 or points.shape[1] < 3:
            return
        self.points_total += int(points.shape[0])
        ground = points[np.abs(points[:, 2]) <= self.ground_tol]
        if ground.size == 0:
            return
        self.ground_points += int(ground.shape[0])
        ix = np.floor(ground[:, 0] / self.cell_size).astype(int)
        iy = np.floor(ground[:, 1] / self.cell_size).astype(int)
        for cell in zip(ix.tolist(), iy.tolist()):
            if cell in self.cells:
                self.points_in_polygon += 1
                self.observed.add(cell)

    def _cell_bounds(self):
        if not self.cells:
            return 0, 0, 0, 0
        xs = [cell[0] for cell in self.cells]
        ys = [cell[1] for cell in self.cells]
        return min(xs), min(ys), max(xs), max(ys)

    def render_ascii(self) -> str:
        """北向上的 ASCII 热力图：# 已观测，. 漏扫。"""
        ix0, iy0, ix1, iy1 = self._cell_bounds()
        lines = []
        for iy in range(iy1, iy0 - 1, -1):
            row = []
            for ix in range(ix0, ix1 + 1):
                if (ix, iy) not in self.cells:
                    row.append(' ')
                else:
                    row.append('#' if (ix, iy) in self.observed else '.')
            lines.append(''.join(row))
        return '\n'.join(lines) + ('\n' if lines else '')

    def render_png(self, path: str, scale: int = 12) -> str:
        """纯标准库 PNG（不依赖 PIL/cv2）：绿=已观测，灰=漏扫，白=多边形外。"""
        scale = max(int(scale), 2)
        ix0, iy0, ix1, iy1 = self._cell_bounds()
        width = (ix1 - ix0 + 1) * scale
        height = (iy1 - iy0 + 1) * scale
        rows = []
        for iy in range(iy1, iy0 - 1, -1):
            row = bytearray()
            for ix in range(ix0, ix1 + 1):
                if (ix, iy) not in self.cells:
                    color = b'\xff\xff\xff'
                elif (ix, iy) in self.observed:
                    color = b'\x00\x6e\x2e'
                else:
                    color = b'\xe6\xe6\xe6'
                row.extend(color * scale)
            rows.append(bytes(row))
        raw = b''.join(b'\x00' + row for row in rows)

        def chunk(tag, data):
            return (struct.pack('>I', len(data)) + tag + data
                    + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))

        png = (b'\x89PNG\r\n\x1a\n'
               + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
               + chunk(b'IDAT', zlib.compress(raw))
               + chunk(b'IEND', b''))
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, 'wb') as handle:
            handle.write(png)
        return path

    def status(self) -> dict:
        total = len(self.cells)
        observed = len(self.observed)
        ratio = (observed / total) if total else 0.0
        return {
            'cell_size_m': self.cell_size,
            'total_cells': total,
            'observed_cells': observed,
            'missed_cells': max(total - observed, 0),
            'coverage_ratio': round(ratio, 4),
            'observed_area_m2': round(observed * self.cell_size ** 2, 1),
            'polygon_area_m2': round(polygon_area(self.polygon), 1),
            'points_total': self.points_total,
            'ground_points': self.ground_points,
            'points_in_polygon': self.points_in_polygon,
            'polygon': [[round(e, 2), round(n, 2)] for e, n in self.polygon],
        }
