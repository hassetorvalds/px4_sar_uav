#!/usr/bin/env python3
"""真实局部占据图 → A* 路径预览（不飞、不发布速度）。

订阅 /local_map/occupied 与 PX4 状态，按当前高度带过滤障碍，规划到相对目标；
输出 JSON 路径和摘要。
"""

import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
from px4_interface.frames import ned_to_enu
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from planner.astar import plan_path
from planner.visual_goal import goal_from_task


def cloud_xyz(msg):
    if msg.point_step != 12 or msg.width == 0:
        return np.empty((0, 3))
    return np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)


class Preview(Node):
    def __init__(self, args):
        super().__init__('astar_preview')
        self.args = args
        self.points = None
        self.monitor = Px4StateMonitor(self)
        self.create_subscription(PointCloud2, args.occupied_topic, self._on_cloud, 5)

    def _on_cloud(self, msg):
        self.points = cloud_xyz(msg).astype(float)

    def position_enu(self):
        state = self.monitor.snapshot()
        if state.position_ned is None:
            return None
        return ned_to_enu(*state.position_ned)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--occupied-topic', default='/local_map/occupied')
    parser.add_argument('--goal-east', type=float)
    parser.add_argument('--goal-north', type=float)
    parser.add_argument('--goal-task', default='', help='VLM TaskSpec JSON；读取第一个 goto')
    parser.add_argument('--height-min-above-m', type=float, default=0.5)
    parser.add_argument('--height-max-above-m', type=float, default=1.5)
    parser.add_argument('--resolution-m', type=float, default=0.25)
    parser.add_argument('--inflate-m', type=float, default=0.4)
    parser.add_argument('--max-radius-m', type=float, default=12.0)
    parser.add_argument('--timeout-s', type=float, default=10.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    rclpy.init()
    node = Preview(args)
    deadline = time.monotonic() + args.timeout_s
    position = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        position = node.position_enu()
        if position is not None and node.points is not None:
            break
    if position is None or node.points is None:
        raise SystemExit('没有收到 PX4 位置或 occupied 点云')
    if args.goal_task:
        import json
        with open(args.goal_task, encoding='utf-8') as handle:
            task = json.load(handle)
        east, north = goal_from_task(task, position)
    elif args.goal_east is not None and args.goal_north is not None:
        east = position[0] + args.goal_east
        north = position[1] + args.goal_north
    else:
        raise SystemExit('需要 --goal-task 或 --goal-east/--goal-north')
    pts = node.points
    band = ((pts[:, 2] >= position[2] + args.height_min_above_m)
            & (pts[:, 2] <= position[2] + args.height_max_above_m))
    obstacles = [(float(p[0]), float(p[1])) for p in pts[band]]
    goal = (east, north)
    path = plan_path(obstacles, position[:2], goal,
                     resolution=args.resolution_m, inflate_m=args.inflate_m,
                     max_radius_m=args.max_radius_m)
    length = (sum(math.hypot(path[i + 1][0] - path[i][0],
                             path[i + 1][1] - path[i][1])
                  for i in range(len(path) - 1)) if path else None)
    out = args.out or f'logs/planner/astar_preview_{time.strftime("%Y%m%d-%H%M%S")}.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    result = {
        'position_enu': [round(float(v), 2) for v in position],
        'goal_enu': [round(float(v), 2) for v in goal],
        'occupied_points': len(pts),
        'height_filtered_obstacles': len(obstacles),
        'path_found': path is not None,
        'path_length_m': None if length is None else round(length, 2),
        'path': [[round(float(x), 2), round(float(y), 2)] for x, y in (path or [])],
        'resolution_m': args.resolution_m, 'inflate_m': args.inflate_m,
    }
    with open(out, 'w', encoding='utf-8') as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2)
    print(json.dumps({k: v for k, v in result.items() if k != 'path'},
                     ensure_ascii=False, indent=2))
    print('path points:', len(path) if path else 0, 'out:', out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
