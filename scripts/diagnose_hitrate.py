#!/usr/bin/env python3
"""命中率归因：从驱动日志判断每轮"卡在哪一步"（不需要飞行/ROS）。

服务器侧 2026-10-16 建议：命中率低时先看"未交权点是否集中在某些净空/朝向"。
本脚本把每轮归到以下阶段（互斥，按发生顺序取第一个失败点）：

```text
A 未发现候选   飞到点位后整圈扫描都没有"前向≤scan_found 且一侧开阔"的扇区
B 复验被拒     发现了候选，但 ±微转复验不通过（世界点漂移/一转就消失）
C 摆位未到     复验通过，但逼近没到停止阈值（摆位超时）
D 已交权       走到 RUN（这时 episode 由 analyze_detour.py 打分）
E 交权后无保持 RUN 里从未进 obstacle 保持态（只有慢速/正常）——由决策记录判定
```

用法：
    python3 scripts/diagnose_hitrate.py logs/2026-10-16/k24_*_driver.log ...
    python3 scripts/diagnose_hitrate.py --dir logs/2026-10-16
"""

import glob
import json
import os
import re
import sys

RE_ARRIVE = re.compile(r'转场到位（ENU 东 ([+-][0-9.]+) 北 ([+-][0-9.]+) 上 ([+-][0-9.]+)')
RE_CAND = re.compile(r'扫描发现候选障碍（前向 ([0-9.]+) m，左 ([^ ]+) / 右 ([^ ]+)）')
RE_VERIFY_FAIL = re.compile(r'微转复验不通过：(.+)')
RE_VERIFY_PASS = re.compile(r'微转复验通过：(.+)')
RE_APPROACH_TIMEOUT = re.compile(r'摆位超时（([0-9.]+) s，前向净空 ([0-9.]+)）')
RE_HANDOVER = re.compile(r'交权点记录：ENU 东 ([+-][0-9.]+) 北 ([+-][0-9.]+) 上 ([+-][0-9.]+)，'
                         r'航向\(ENU\) ([+-]?[0-9.]+)°')
RE_RUN = re.compile(r'已向 VLM 节点发送 enable')
RE_NO_SCAN = re.compile(r'全部候选点扫描完毕仍未找到可复验障碍')


def classify(driver_log, decisions_dir=None):
    text = open(driver_log, encoding='utf-8', errors='replace').read()
    arrivals = RE_ARRIVE.findall(text)
    cands = RE_CAND.findall(text)
    fails = RE_VERIFY_FAIL.findall(text)
    passes = RE_VERIFY_PASS.findall(text)
    approach_to = RE_APPROACH_TIMEOUT.findall(text)
    handover = RE_HANDOVER.search(text)
    ran = bool(RE_RUN.search(text))

    if ran:
        stage = 'D 已交权'
        if decisions_dir:
            guards = set()
            for p in glob.glob(os.path.join(decisions_dir, 'decision_*.json')):
                try:
                    guards.add(json.load(open(p, encoding='utf-8')).get('guard'))
                except Exception:      # noqa: BLE001
                    pass
            if 'obstacle' not in guards:
                stage = 'E 交权后无保持'
    elif approach_to:
        stage = 'C 摆位未到'
    elif fails and not passes:
        stage = 'B 复验被拒'
    elif RE_NO_SCAN.search(text) or not cands:
        stage = 'A 未发现候选'
    else:
        stage = '? 其它'

    return {
        'log': os.path.basename(driver_log),
        'stage': stage,
        'arrivals': arrivals[-2:],
        'candidates': len(cands),
        'cand_detail': cands[:3],
        'verify_fail': len(fails), 'verify_pass': len(passes),
        'approach_timeout': approach_to[:2],
        'handover': handover.groups() if handover else None,
    }


def main(argv):
    if len(argv) > 1 and argv[1] == '--dir':
        logs = sorted(glob.glob(os.path.join(argv[2], '*_driver.log')))
    else:
        logs = []
        for a in argv[1:]:
            logs.extend(sorted(glob.glob(a)))
    if not logs:
        print('用法：diagnose_hitrate.py logs/<date>/*_driver.log | --dir logs/<date>')
        return 1

    counts = {}
    print('| 轮次 | 归类 | 候选数 | 复验 通过/拒 | 交权点(ENU东,北,上,航向) | 最后转场到位 |')
    print('|---|---|---|---|---|---|')
    for log in logs:
        label = os.path.basename(log).split('_s')[0]
        sid = os.path.basename(log).split('_s')[1].split('_driver')[0] if '_s' in log else ''
        # 决策目录在**仓库根**（脚本位于 scripts/），不在 logs/<date>/ 下
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        decisions = os.path.join(root, f'vlm_decisions_s{sid}') if sid else None
        if decisions and not os.path.isdir(decisions):
            decisions = None
        info = classify(log, decisions)
        counts[info['stage']] = counts.get(info['stage'], 0) + 1
        ho = info['handover']
        print(f"| {label} | {info['stage']} | {info['candidates']} | "
              f"{info['verify_pass']}/{info['verify_fail']} | "
              f"{'—' if not ho else (ho[0]+','+ho[1]+','+ho[2]+','+ho[3])} | "
              f"{'—' if not info['arrivals'] else str(info['arrivals'][-1])} |")

    print('\n阶段分布：')
    for k in sorted(counts):
        print(f'  {k}: {counts[k]}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
