#!/usr/bin/env python3
"""绕行成功率打分（服务器侧 2026-10-14 建议 ③ 的判据落地）。

判据（写死，保证可比）：

```text
障碍 episode   guard 连续处于 obstacle / obstacle_slow / obstacle_escape 的一段
闭合           episode 结束后 N=20 帧内，出现 ≥5 帧 post_filter.forward > 0.05
              （即"绕开后前向真的恢复了"）
主指标        绕行闭合率 = 闭合 episode 数 / **有 episode 的轮次**；要求 100%
副指标        寻障命中率 = 有 episode 的轮次 / 总轮次
另报          "无 episode"的轮次单独计数（既不隐藏、也不算 FAIL）
```

双指标口径来自服务器侧 2026-10-14 回信 ②：**"没找到可绕障碍"与"绕行逻辑是否
正确"是两种能力**，混进同一个分母会把"没找到障碍"错判成"绕行不会"。

用法：
    python3 scripts/analyze_detour.py vlm_decisions_m4                 # 单轮（含双指标）
    python3 scripts/analyze_detour.py vlm_decisions_t6 vlm_decisions_t7 ...  # 多轮聚合
    python3 scripts/analyze_detour.py --min-frames 3 vlm_decisions_a vlm_decisions_b
"""

import glob
import json
import os
import sys

OBSTACLE_GUARDS = ('obstacle', 'obstacle_slow', 'obstacle_escape')
# 快速闭合窗口（原判据）：episode 结束后 20 帧（≈9 s）内 ≥5 帧前向非零。
RECOVERY_WINDOW = 20
# **最终闭合窗口**（服务器侧 2026-10-16 指出）：恢复窗口必须 ≥"保持+脱困"的时间预算——
# `hold_max_s=20 s`（≈44 帧）+ 脱困 `escape_max_s=10 s`（≈22 帧）→ 取 50 帧（≈23 s）。
# 否则"合法但较慢"的绕行会被误判成未闭合（k7 的 episode 2 很可能就是被截断的）。
LONG_WINDOW = 50
RECOVERY_FRAMES_NEEDED = 5
FORWARD_THRESHOLD = 0.05
# "可绕"判据（服务器侧 2026-10-16）：episode 期间**至少有一侧净空 ≥ 2.0 m**（有空间可蹭过去）。
# 两侧都紧 / 盲区持续的 episode 归为"**不可绕**"——"保持住不硬推"是正确行为，**不进口径**。
AMENABLE_SIDE_M = 2.0
MIN_EPISODE_FRAMES = 3           # 单帧闪现不算 episode（服务器侧 2026-10-14 定稿）
# episode 必须**至少含 1 帧保持态**，否则只是"减速"不算绕行；脱困帧也算保持态。
HOLD_GUARDS = ('obstacle', 'obstacle_escape')


def analyze_round(directory, min_episode_frames):
    """分析单轮，返回 (合格 episode 列表, 闭合数, 记录, 统计字符串列表)。"""
    paths = sorted(glob.glob(os.path.join(directory, 'decision_*.json')))
    if not paths:
        return None
    records = [json.load(open(p, encoding='utf-8')) for p in paths]
    records.sort(key=lambda r: r.get('frame_seq') or 0)

    episodes = []
    current = None
    for position, record in enumerate(records):
        guard = record.get('guard')
        if guard in OBSTACLE_GUARDS:
            if current is None:
                current = {'start': position, 'end': position}
            else:
                current['end'] = position
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
        if len(frames) < min_episode_frames:
            continue
        if not any(f.get('guard') in HOLD_GUARDS for f in frames):
            slow_only += 1          # 只减速、从不保持/转向 → 不算绕行 episode，单独计数
            continue
        qualified.append(episode)

    lines = [f'目录 {directory}：{len(records)} 帧，识别到 {raw_episodes} 个障碍区间；'
             f'主口径 episode（≥{min_episode_frames} 帧且含保持态）{len(qualified)} 个，'
             f'仅慢速区间 {slow_only} 个（不计分）']
    closed = 0
    fast_closed = 0
    amenable_total = 0
    impassable = 0
    for order, episode in enumerate(qualified, 1):
        def _forward_frames(n):
            w = records[episode['end'] + 1:episode['end'] + 1 + n]
            return sum(1 for r in w
                       if ((r.get('post_filter') or {}).get('forward') or 0.0) > FORWARD_THRESHOLD)

        forward_frames = _forward_frames(RECOVERY_WINDOW)      # 快速（20 帧）
        forward_long = _forward_frames(LONG_WINDOW)            # 最终（50 帧）
        ep_frames = records[episode['start']:episode['end'] + 1]
        sides = {(r.get('guard_detail') or {}).get('avoid_side') for r in ep_frames}
        sides = {s for s in sides if s}
        escapes = [r for r in ep_frames if r.get('guard') == 'obstacle_escape']
        # "可绕"只看**保持帧**（guard=obstacle）的侧向净空：episode 末尾可能包含"已放开"的帧
        # （例如注入场景最后回到 3.0 m），把它们算进来会把"不可绕"误判成"可绕"（2026-10-16 esc5 实测）
        hold_frames = [r for r in ep_frames if r.get('guard') == 'obstacle']
        side_vals = [(r.get('guard_detail') or {}).get('side_clear_m')
                     for r in (hold_frames or ep_frames)]
        # 用**中位数**而不是最大值：保持段末尾常有 1–2 帧"障碍已放开、但释放确认还没走完"的帧
        # （2026-10-16 esc5：123 帧 0.4 + 2 帧 3.0）——取最大会把"不可绕"误判成"可绕"。
        nums = sorted(float(v) for v in side_vals if isinstance(v, (int, float)))
        if nums:
            side_max = nums[len(nums) // 2]           # 中位数
        else:
            side_max = float('inf')                   # 拿不到就按"可绕"计入（宁可算我们的账）
        amenable = side_max >= AMENABLE_SIDE_M
        ok = forward_long >= RECOVERY_FRAMES_NEEDED            # 主判据：最终闭合
        fast = forward_frames >= RECOVERY_FRAMES_NEEDED        # 子判据：快速闭合
        if amenable:
            amenable_total += 1
            closed += int(ok)
            fast_closed += int(fast)
        else:
            impassable += 1
        first = records[episode['start']]
        clearance = first.get('clearance') or [float('nan')]
        lines.append(
            f'  episode {order}: seq {first.get("frame_seq")}–'
            f'{records[episode["end"]].get("frame_seq")}'
            f'（{episode["end"] - episode["start"] + 1} 帧，起始净空 {clearance[0]:.2f} m，'
            f'避让侧 {sides or "—"}，脱困帧 {len(escapes)}）→ '
            f'{"可绕" if amenable else "**不可绕**"}（两侧最大 {side_max:.2f} m） '
            f'最终闭合 {forward_long}/50 帧 与 快速闭合 {forward_frames}/20 帧 → '
            f'{"✅ 最终闭合" if ok else "❌ 未闭合"}'
            + ('（快速）' if fast else ('（慢速恢复）' if ok else '')))
    transients = raw_episodes - len(qualified) - slow_only
    lines.append(f'另有 {transients} 个单帧闪现区间（探测瞬态，不计分）')
    return {'qualified': len(qualified), 'closed': closed, 'fast_closed': fast_closed,
            'amenable_total': amenable_total, 'impassable': impassable, 'lines': lines}


def main(argv):
    min_episode_frames = MIN_EPISODE_FRAMES
    directories = []
    index = 1
    while index < len(argv):
        arg = argv[index]
        if arg in ('-n', '--min-frames'):
            index += 1
            min_episode_frames = int(argv[index])
        elif arg.isdigit():          # 兼容旧用法：位置参数里的数字当 min_episode_frames
            min_episode_frames = int(arg)
        else:
            directories.append(arg)
        index += 1
    if not directories:
        directories = ['vlm_decisions']

    rounds_total = 0
    rounds_with_episode = 0
    total_episodes = 0
    total_closed = 0
    total_fast = 0
    rounds_amenable = 0
    total_impassable = 0
    for directory in directories:
        result = analyze_round(directory, min_episode_frames)
        if result is None:
            print(f'{directory} 下没有 decision_*.json')
            continue
        print('\n'.join(result['lines']))
        rounds_total += 1
        if result['qualified']:
            rounds_with_episode += 1
            total_episodes += result['qualified']
            total_closed += result['closed']
            total_fast += result['fast_closed']
            rounds_amenable += result['amenable_total']
            total_impassable += result['impassable']
            rate = result['closed'] / result['qualified']
            print(f'本轮闭合率 {result["closed"]}/{result["qualified"]} = {rate * 100:.0f}% '
                  f'→ {"PASS（本轮算成功）" if result["closed"] == result["qualified"] else "FAIL"}')
        else:
            print('本轮没有主口径 episode（没遇到"保持态"障碍）→ 计入无 episode 轮次')

    print('\n==== 双指标汇总（服务器侧 2026-10-14 ②） ====')
    if rounds_total <= 0:
        print('没有可分析的轮次')
        return 1
    if rounds_amenable:
        closure = total_closed / rounds_amenable
        print(f'主指标 · **可绕闭合率** = {total_closed}/{rounds_amenable} = {closure * 100:.0f}% '
              f'→ {"PASS（要求 100%）" if total_closed == rounds_amenable else "FAIL"}')
        print(f'副指标 · 快速闭合（20 帧）= {total_fast}/{rounds_amenable}')
    else:
        print('主指标 · 可绕闭合率 = 无可绕 episode，无法评估')
    print(f'另报   · **不可绕 episode {total_impassable} 个**（两侧均紧/盲区持续，保持住不硬推=正确行为，不进口径）')
    hit = rounds_with_episode / rounds_total
    print(f'副指标 · 寻障命中率 = 有 episode 轮次 {rounds_with_episode}/{rounds_total} '
          f'= {hit * 100:.0f}%')
    print(f'另报   · 无 episode 轮次 {rounds_total - rounds_with_episode}/{rounds_total}'
          f'（既不隐藏、也不算 FAIL）')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
