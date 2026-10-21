#!/usr/bin/env python3
"""阶段 1：注入簇验高度门 C（两档 top + 漂移/尺度）。

不飞、不依赖场景；把 voxel 簇 top 与三路高度注入 gate，落 JSON 取证。
真实 ROS 注入下一步用 /inject/height_status 回放到旧驱动。
"""

import json
import os
import time

from mission.height_gate import evaluate_height_gate

FLIGHT = 2.0
CLEARANCE = 0.5
TOPS = {'A_low_top_1m': 1.0, 'B_high_top_5m': 5.0}
OFFSETS = (0.0, 0.5, -0.5)


def main():
    rows = []
    for name, top in TOPS.items():
        for offset in OFFSETS:
            common = evaluate_height_gate(
                mode='c_map_rel', phase='DESCEND',
                px4_z=FLIGHT + offset, airsim_z=FLIGHT + offset,
                map_z=FLIGHT + offset, cluster_top_z=top + offset,
                clearance_m=CLEARANCE)
            px4_only = evaluate_height_gate(
                mode='c_map_rel', phase='DESCEND',
                px4_z=FLIGHT + offset, airsim_z=FLIGHT + offset,
                map_z=FLIGHT, cluster_top_z=top,
                clearance_m=CLEARANCE)
            legacy = evaluate_height_gate(
                mode='legacy', phase='DESCEND',
                px4_z=FLIGHT + offset, airsim_z=FLIGHT + offset,
                map_z=FLIGHT + offset, cluster_top_z=top + offset,
                clearance_m=CLEARANCE)
            rows.append({
                'case': name, 'offset_m': offset,
                'common_drift': common.to_dict(),
                'px4_only_drift': px4_only.to_dict(),
                'legacy': legacy.to_dict(),
            })
    base = {name: evaluate_height_gate(
        mode='c_map_rel', phase='DESCEND', px4_z=FLIGHT, airsim_z=FLIGHT,
        map_z=FLIGHT, cluster_top_z=top, clearance_m=CLEARANCE).overflyable
        for name, top in TOPS.items()}
    common_ok = all(row['common_drift']['overflyable'] == base[row['case']]
                    for row in rows)
    px4_only_ok = all(row['px4_only_drift']['overflyable'] == base[row['case']]
                      for row in rows)
    out = {
        'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'mode': 'c_map_rel', 'flight_alt_m': FLIGHT,
        'clearance_m': CLEARANCE, 'expected': base,
        'common_drift_invariant': common_ok,
        'px4_only_drift_invariant': px4_only_ok,
        'rows': rows,
    }
    path = os.path.join('logs', 'height_gate',
                        f'c_validation_{time.strftime("%Y%m%d-%H%M%S")}.json')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(out, handle, ensure_ascii=False, indent=2)
    print(json.dumps({k: out[k] for k in out if k != 'rows'},
                     ensure_ascii=False, indent=2))
    for row in rows:
        print(row['case'], f'offset={row["offset_m"]:+.1f}',
              'common=', row['common_drift']['overflyable'],
              'px4_only=', row['px4_only_drift']['overflyable'],
              'legacy=', row['legacy']['overflyable'])
    print('path:', path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
