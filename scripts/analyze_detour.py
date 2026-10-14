#!/usr/bin/env python3
"""绕行成功率打分（服务器侧 2026-10-14 建议 ③ 的判据落地）。

判据（写死，保证可比）：

```text
障碍 episode   guard 连续处于 obstacle / obstacle_slow 的一段
闭合           episode 结束后 N=20 帧内，出现 ≥5 帧 post_filter.forward > 0.05
              （即"绕开后前向真的恢复了"）
成功率        闭合 episode 数 / 总 episode 数；要求 100% 才算这一趟成功
```

用法：
    python3 scripts/analyze_detour.py vlm_decisions_m4
"""

import glob
import json
import os
import sys

OBSTACLE_GUARDS = ('obstacle', 'obstacle_slow')
RECOVERY_WINDOW = 20
RECOVERY_FRAMES_NEEDED = 5
FORWARD_THRESHOLD = 0.05
MIN_EPISODE_FRAMES = 3           # 单帧闪现不算 episode（服务器侧 2026-10-14 定稿）
HOLD_GUARD = 'obstacle'          # episode 必须**至少含 1 帧保持态**，否则只是"减速"不算绕行


def main(argv):
    directory = argv[1] if len(argv) > 1 else 'vlm_decisions'
    min_episode_frames = int(argv[2]) if len(argv) > 2 else MIN_EPISODE_FRAMES
    paths = sorted(glob.glob(os.path.join(directory, 'decision_*.json')))
    if not paths:
        print(f'{directory} 下没有 decision_*.json')
        return 1
    records = [json.load(open(p, encoding='utf-8')) for p in paths]
    records.sort(key=lambda r: r.get('frame_seq') or 0)

    episodes = []
    current = None
    for index, record in enumerate(records):
        guard = record.get('guard')
        if guard in OBSTACLE_GUARDS:
            if current is None:
                current = {'start': index, 'end': index}
            else:
                current['end'] = index
        elif current is not None:
            episodes.append(current)
            current = None
    if current is not None:
        episodes.append(current)

    raw_episodes = len(episodes)
    qualified = []
    slow_only = 0
    for episode in episodes:
        frames = records[episode['start']:episode['end'] + 1]
        length = len(frames)
        has_hold = any(f.get('guard') == HOLD_GUARD for f in frames)
        if length < min_episode_frames:
            continue
        if not has_hold:
            slow_only += 1          # 只减速、从不保持/转向 → 不算绕行 episode，单独计数
            continue
        qualified.append(episode)
    print(f'目录 {directory}：{len(records)} 帧，识别到 {raw_episodes} 个障碍区间；'
          f'主口径 episode（≥{min_episode_frames} 帧且含保持态）{len(qualified)} 个，'
          f'仅慢速区间 {slow_only} 个（不计分）')
    episodes = qualified
    closed = 0
    for order, episode in enumerate(episodes, 1):
        window = records[episode['end'] + 1:episode['end'] + 1 + RECOVERY_WINDOW]
        forward_frames = sum(
            1 for r in window
            if ((r.get('post_filter') or {}).get('forward') or 0.0) > FORWARD_THRESHOLD)
        yaws = {(r.get('guard_detail') or {}).get('avoid_side') for r in
                records[episode['start']:episode['end'] + 1]}
        sides = {s for s in yaws if s}
        ok = forward_frames >= RECOVERY_FRAMES_NEEDED
        closed += int(ok)
        first = records[episode['start']]
        clearance = first.get('clearance') or [float('nan')]
        print(f'  episode {order}: seq {first.get("frame_seq")}–'
              f'{records[episode["end"]].get("frame_seq")}'
              f'（{episode["end"] - episode["start"] + 1} 帧，起始净空 {clearance[0]:.2f} m，'
              f'避让侧 {sides or "—"}）→ 恢复窗口内前向非零 {forward_frames} 帧 '
              f'{"✅ 闭合" if ok else "❌ 未闭合"}')

    transients = raw_episodes - len(episodes) - slow_only
    print(f'另有 {transients} 个单帧闪现区间（探测瞬态，不计分）')
    if episodes:
        rate = closed / len(episodes)
        print(f'\n闭合率 {closed}/{len(episodes)} = {rate * 100:.0f}% '
              f'→ {"PASS（本轮算成功）" if closed == len(episodes) else "FAIL"}')
    else:
        print('\n主口径没有 episode（本轮没遇到"保持态"障碍）→ 不计分')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
