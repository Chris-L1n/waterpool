#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
异常行为检测模块

检测三类异常行为：
  1) 船-船搭靠 (SC)   — 两船接近 → 并靠 → 持续 ≥15秒
  2) 快速通过 (KS)     — 速度突增 / 加速度异常，穿越特定区域
  3) 船靠岸送人上岸 (KA) — 偏航→入禁区→减速停留，或运动特征判断

输入：radar_track_parser_v4 中 SimpleTracker.update() 输出的 targets 列表
     每个目标含：track_id, x_m, y_m, speed_m_s

输出：anomaly_events.csv + 终端打印 + on_anomaly(event) 回调

参考原型：test_3 - 副本/recognize/detect_anomalies.py（detect_berthing / detect_high_speed / detect_shore）

运行：
  实时模式：由 radar_live.py 调用 feed()
"""

import argparse
import csv
import json
import math
import time
from collections import OrderedDict, defaultdict, deque
from datetime import datetime
from pathlib import Path


# ============================================================
# 工具函数
# ============================================================

def point_in_polygon(x, y, polygon):
    """射线法判断点 (x, y) 是否在多边形内。polygon 为 [[x1,y1], [x2,y2], ...]"""
    n = len(polygon)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def point_in_circle(x, y, cx, cy, radius):
    return math.hypot(x - cx, y - cy) <= radius


def point_to_segment_dist(px, py, ax, ay, bx, by):
    """点到线段 AB 的最短距离"""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / denom))
    cx, cy = ax + t * abx, ay + t * aby
    return math.hypot(px - cx, py - cy)


def point_to_polygon_dist(x, y, polygon):
    """点到多边形边界的最短距离；点在内部时距离为 0"""
    if point_in_polygon(x, y, polygon):
        return 0.0
    n = len(polygon)
    return min(
        point_to_segment_dist(x, y, *polygon[i], *polygon[(i + 1) % n])
        for i in range(n)
    )


def distance_to_channel_center(x, y, channel_polygon):
    """点到航道中心线的近似距离（航道多边形中心 X 坐标的偏差）"""
    if not channel_polygon or len(channel_polygon) < 3:
        return 0.0
    xs = [p[0] for p in channel_polygon]
    center_x = (min(xs) + max(xs)) / 2.0
    return abs(x - center_x)


def dist_to_shore(x, y, shore_polygon):
    """
    计算点到岸边的最短距离。
    岸边 = unauthorized_docking_zone 多边形的边界。
    点在区域外时，距离为到多边形最近边的距离；
    点在区域内时，距离为到最近边的距离（≥0）。
    """
    if not shore_polygon or len(shore_polygon) < 3:
        return float("inf")
    n = len(shore_polygon)
    return min(
        point_to_segment_dist(x, y, *shore_polygon[i], *shore_polygon[(i + 1) % n])
        for i in range(n)
    )


# ============================================================
# 异常检测器
# ============================================================

class AnomalyDetector:
    """三类异常行为实时检测器"""

    def __init__(self, config, on_anomaly=None):
        """
        config 字典需包含 "zones" 和 "anomaly_detection" 两个 key。
        on_anomaly: 可选回调 (event_dict) -> None
        """
        # ── 区域配置（仅保留 KA 检测需要的部分）──
        z = config.get("zones", {})
        self.normal_channel = z.get("normal_channel", {}).get("points", [])
        self.docking_zone = z.get("unauthorized_docking_zone", {}).get("points", [])
        ap = z.get("authorized_docking_point") or {}
        self.auth_dock_pos = ap.get("position", None) if ap else None
        self.auth_dock_radius = float(ap.get("radius_m", 0.3)) if ap else 0.3

        # ── 检测参数 ──
        ad = config.get("anomaly_detection", {})
        self.enabled = bool(ad.get("enabled", True))

        sc = ad.get("ship_rendezvous", {})
        self.sc_dist_threshold = float(sc.get("distance_threshold_m", 0.5))
        self.sc_duration_s = float(sc.get("duration_threshold_s", 15.0))
        self.sc_rel_speed_max = float(sc.get("relative_speed_threshold_m_s", 0.3))
        self.sc_approach_window_s = float(sc.get("approach_window_s", 10.0))
        self.sc_approach_dist = float(sc.get("approach_distance_threshold_m", 2.0))
        self.sc_min_dist_drop = float(sc.get("min_distance_drop_m", 0.5))

        ks = ad.get("fast_passage", {})
        self.ks_speed_min = float(ks.get("speed_threshold_m_s", 1.5))
        self.ks_speed_ratio = float(ks.get("speed_ratio_threshold", 2.0))
        self.ks_accel_min = float(ks.get("acceleration_threshold_m_s2", 0.3))
        self.ks_min_duration_s = float(ks.get("min_duration_s", 2.0))

        ka = ad.get("shore_docking", {})
        self.ka_deviation_m = float(ka.get("deviation_threshold_m", 0.3))
        self.ka_speed_low = float(ka.get("speed_threshold_m_s", 0.2))
        self.ka_duration_s = float(ka.get("duration_threshold_s", 10.0))
        # 路径B（运动特征）
        self.ka_shore_dist = float(ka.get("shore_arrival_distance_m", 0.5))
        self.ka_approach_window_s = float(ka.get("shore_approach_window_s", 20.0))
        self.ka_fast_approach = float(ka.get("shore_fast_approach_mps", 0.3))
        self.ka_dist_drop = float(ka.get("shore_min_distance_drop_m", 0.3))
        self.ka_post_slow = float(ka.get("shore_slow_after_arrival_mps", 0.2))

        # 冷却
        co = ad.get("cooldowns", {})
        self.cooldown_sc = float(co.get("sc_s", 60.0))
        self.cooldown_ks = float(co.get("ks_s", 30.0))
        self.cooldown_ka = float(co.get("ka_s", 60.0))

        # ── 输出 ──
        self.on_anomaly = on_anomaly
        self.csv_path = ad.get("csv", "anomaly_events.csv")
        self.csv_file = None
        self.csv_writer = None
        self.event_count = 0

        # ── 轨迹历史：track_id → deque(maxlen=600) ──
        self.track_history = defaultdict(lambda: deque(maxlen=600))
        self._mono = time.monotonic

        # ── 搭靠状态 ──
        # (tid_a, tid_b) → {
        #     "approach_start_mono": float|None,
        #     "approach_start_dist": float|None,
        #     "close_start_mono": float|None,
        #     "distance_history": deque([(mono, dist), ...])
        # }
        self.rendezvous_pairs = {}

        # ── 靠岸状态机（路径A）──
        # tid → {"state": "NORMAL"|"DEVIATED"|"IN_ZONE"|"STOPPED", "stopped_mono": float|None, ...}
        self.docking_states = {}

        # ── 靠岸运动特征（路径B）──
        # tid → {"shore_dist_history": deque([(mono, dist, x, y), ...]),
        #         "arrival_mono": float|None, "post_arrival_speed_ok": bool}
        self.shore_approach = {}

        # ── 冷却 ──
        self.last_triggered = {}  # (event_type, *keys) → mono

        # ── KS 超速持续计时 ──
        # tid → {"speed_start_mono": float|None}
        self.ks_speed_timers = {}

    # ════════════════════════════════════════════════════════════
    # 区域判断
    # ════════════════════════════════════════════════════════════

    def _in_zone(self, x, y, polygon):
        if not polygon or len(polygon) < 3:
            return True
        return point_in_polygon(x, y, polygon)

    def _in_authorized_docking(self, x, y):
        if self.auth_dock_pos is None:
            return False
        return point_in_circle(x, y, self.auth_dock_pos[0], self.auth_dock_pos[1], self.auth_dock_radius)

    def _deviation(self, x, y):
        return distance_to_channel_center(x, y, self.normal_channel)

    def _shore_dist(self, x, y):
        return dist_to_shore(x, y, self.docking_zone)

    # ════════════════════════════════════════════════════════════
    # 冷却
    # ════════════════════════════════════════════════════════════

    def _in_cooldown(self, event_type, now, *keys):
        ek = (event_type,) + tuple(keys)
        cd = {"SC": self.cooldown_sc, "KS": self.cooldown_ks, "KA": self.cooldown_ka}
        if ek in self.last_triggered:
            if now - self.last_triggered[ek] < cd.get(event_type, 60.0):
                return True
        self.last_triggered[ek] = now
        return False

    # ════════════════════════════════════════════════════════════
    # 主入口
    # ════════════════════════════════════════════════════════════

    def feed(self, targets, _now=None):
        """每帧调用。targets: [{track_id, x_m, y_m, speed_m_s}, ...]
        _now: 可选，用于离线回放时传入模拟时间戳，实时模式自动取 time.monotonic()"""
        if not self.enabled or not targets:
            return

        now_mono = _now if _now is not None else self._mono()

        # 更新轨迹历史
        active_tids = set()
        for t in targets:
            tid = t.get("track_id")
            if tid is None:
                continue
            active_tids.add(tid)
            self.track_history[tid].append({
                "mono": now_mono,
                "x_m": t["x_m"],
                "y_m": t["y_m"],
                "speed_m_s": t["speed_m_s"],
            })

        # 清理不再活跃的轨迹状态
        for tid in list(self.docking_states):
            if tid not in active_tids:
                del self.docking_states[tid]
        for tid in list(self.shore_approach):
            if tid not in active_tids:
                del self.shore_approach[tid]
        for tid in list(self.ks_speed_timers):
            if tid not in active_tids:
                del self.ks_speed_timers[tid]

        # 检测
        ka_targets = set()
        ka_targets.update(self._check_shore_docking(targets, now_mono))
        self._check_rendezvous(targets, now_mono)
        self._check_fast_passage(targets, now_mono, ka_targets)

    # ════════════════════════════════════════════════════════════
    # SC 船-船搭靠（参考原型 detect_berthing）
    # ════════════════════════════════════════════════════════════

    def _check_rendezvous(self, targets, now_mono):
        """
        简化版 SC：两船距离 < 阈值 + 持续 ≥ 阈值 → 报警。
        实验室环境不需要速度差/接近窗口/距离下降量。
        """
        active = [t for t in targets if t.get("track_id") is not None]
        if len(active) < 2:
            self.rendezvous_pairs.clear()
            return

        checked = set()

        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                a, b = active[i], active[j]
                ta, tb = a["track_id"], b["track_id"]
                pk = (min(ta, tb), max(ta, tb))
                checked.add(pk)

                dist = math.hypot(a["x_m"] - b["x_m"], a["y_m"] - b["y_m"])

                if pk not in self.rendezvous_pairs:
                    self.rendezvous_pairs[pk] = {"close_start_mono": None}
                ps = self.rendezvous_pairs[pk]

                # 距离 < 阈值 = 并靠并计时，距离 > 阈值 = 分开则清零
                if dist < self.sc_dist_threshold:
                    if ps["close_start_mono"] is None:
                        ps["close_start_mono"] = now_mono
                    dur = now_mono - ps["close_start_mono"]
                    if dur >= self.sc_duration_s:
                        if not self._in_cooldown("SC", now_mono, pk[0], pk[1]):
                            self._emit("SC", "船-船搭靠", [ta, tb], dur, {
                                "distance_m": round(dist, 3)})
                else:
                    ps["close_start_mono"] = None

        # 清理消失的 pair
        for pk in list(self.rendezvous_pairs):
            if pk not in checked:
                del self.rendezvous_pairs[pk]

    # ════════════════════════════════════════════════════════════
    # KS 快速通过（参考原型 detect_high_speed + 保留加速度判断）
    # ════════════════════════════════════════════════════════════

    def _check_fast_passage(self, targets, now_mono, ka_targets):
        """
        全海域判断（不限定区域）：
          ① 速度 > 绝对阈值
          ② 持续超速 ≥ 最短时长
          ③ 满足（速度突增 ≥ N倍 或 加速度 > 阈值）
        """
        for t in targets:
            tid = t.get("track_id")
            if tid is None:
                continue

            # 靠岸目标去重
            if tid in ka_targets:
                continue

            cur_speed = abs(t["speed_m_s"])

            if cur_speed < self.ks_speed_min:
                # 速度不达标 → 重置超速计时
                if tid in self.ks_speed_timers:
                    self.ks_speed_timers[tid]["speed_start_mono"] = None
                continue

            # 速度达标，初始化计时
            if tid not in self.ks_speed_timers:
                self.ks_speed_timers[tid] = {"speed_start_mono": None}
            if self.ks_speed_timers[tid]["speed_start_mono"] is None:
                self.ks_speed_timers[tid]["speed_start_mono"] = now_mono

            speed_dur = now_mono - self.ks_speed_timers[tid]["speed_start_mono"]
            if speed_dur < self.ks_min_duration_s:
                continue

            # 检查速度突增 / 加速度
            hist = list(self.track_history[tid])
            if len(hist) < 3:
                continue

            prev_speeds = [abs(h["speed_m_s"]) for h in hist[:-2]]
            if not prev_speeds:
                continue
            avg_speed = sum(prev_speeds) / len(prev_speeds)

            surge = (avg_speed > 0.05 and cur_speed / avg_speed >= self.ks_speed_ratio)

            prev = hist[-2]
            dt = now_mono - prev["mono"]
            accel = 0.0
            if 0.001 < dt <= 2.0:
                accel = (cur_speed - abs(prev["speed_m_s"])) / dt
            high_accel = accel > self.ks_accel_min

            # KS触发：速度>阈值 + 持续时间达标即可（surge/accel可选检查）
            trigger_ks = True
            reason = "speed_threshold"
            if surge:
                reason = "speed_surge"
            elif high_accel:
                reason = "high_acceleration"

            if trigger_ks:
                if not self._in_cooldown("KS", now_mono, tid):
                    self.ks_speed_timers[tid]["speed_start_mono"] = None
                    self._emit("KS", "快速通过", [tid], speed_dur, {
                        "current_speed_m_s": round(cur_speed, 3),
                        "average_speed_m_s": round(avg_speed, 3),
                        "speed_ratio": round(cur_speed / max(avg_speed, 0.01), 2),
                        "acceleration_m_s2": round(accel, 3),
                        "trigger_reason": reason,
                        "speed_duration_s": round(speed_dur, 1),
                        "position": {"x": round(t["x_m"], 2), "y": round(t["y_m"], 2)},
                    })

    # ════════════════════════════════════════════════════════════
    # KA 船靠岸送人上岸
    #   路径A（区域状态机）+ 路径B（运动特征，参考原型 detect_shore）
    #   返回触发 KA 的 track_id 集合（用于去重）
    # ════════════════════════════════════════════════════════════

    def _check_shore_docking(self, targets, now_mono):
        triggered = set()

        for t in targets:
            tid = t.get("track_id")
            if tid is None:
                continue

            x, y, spd = t["x_m"], t["y_m"], abs(t["speed_m_s"])
            deviated = self._deviation(x, y) > self.ka_deviation_m
            in_auth = self._in_authorized_docking(x, y)

            # ── 路径A：偏航+减速停留（全海域，不依赖禁区多边形）──
            trig_a = self._ka_path_a(tid, x, y, spd, deviated, in_auth, now_mono)
            if trig_a:
                triggered.add(tid)

            # ── 路径B：距岸边运动特征（不依赖区域标定）──
            trig_b = self._ka_path_b(tid, x, y, spd, now_mono)
            if trig_b:
                triggered.add(tid)

        return triggered

    def _ka_path_a(self, tid, x, y, spd, deviated, in_auth, now_mono):
        """路径A：NORMAL → DEVIATED（偏航）→ STOPPED（减速停留）→ 报警"""
        if tid not in self.docking_states:
            self.docking_states[tid] = {
                "state": "NORMAL",
                "stopped_mono": None,
                "was_moving": False,
            }
        ds = self.docking_states[tid]
        state = ds["state"]

        # 记录目标是否曾经以正常速度移动过（避免一启动就停在区外的虚假报警）
        if spd >= 0.1:
            ds["was_moving"] = True

        if state == "NORMAL":
            if deviated and not in_auth and ds["was_moving"]:
                ds["state"] = "DEVIATED"

        elif state == "DEVIATED":
            if spd < self.ka_speed_low and not in_auth:
                ds["state"] = "STOPPED"
                ds["stopped_mono"] = now_mono
            elif not deviated:
                ds["state"] = "NORMAL"

        elif state == "STOPPED":
            if spd < self.ka_speed_low and not in_auth:
                dur = now_mono - (ds["stopped_mono"] or now_mono)
                if dur >= self.ka_duration_s:
                    if not self._in_cooldown("KA", now_mono, tid):
                        self._emit("KA", "船靠岸送人上岸", [tid], dur, {
                            "path": "deviation_stop",
                            "position": {"x": round(x, 2), "y": round(y, 2)},
                            "speed_m_s": round(spd, 3),
                            "deviation_m": round(self._deviation(x, y), 3),
                        })
                        ds["state"] = "NORMAL"
                        ds["stopped_mono"] = None
                        return True
            else:
                if spd >= self.ka_speed_low or not deviated:
                    ds["state"] = "NORMAL"
                    ds["stopped_mono"] = None

        return False

    def _ka_path_b(self, tid, x, y, spd, now_mono):
        """
        路径B：运动特征判断（参考原型 detect_shore）
          ① 维护距岸边距离的历史（接近窗口时长）
          ② 首次进入岸边距离阈值时，检查接近窗口：
               接近速度 ≥ 阈值 + 距离下降量 ≥ 阈值 → 标记为"快速接近"
          ③ 到达后速度持续低于阈值 → 报警
        """
        shore_d = self._shore_dist(x, y)

        # 初始化
        if tid not in self.shore_approach:
            self.shore_approach[tid] = {
                "history": deque(maxlen=600),
                "arrival_mono": None,
                "approached_fast": False,
            }
        sa = self.shore_approach[tid]

        # 维护历史（保留接近窗口时长内的记录）
        sa["history"].append((now_mono, shore_d, x, y))
        cutoff = now_mono - self.ka_approach_window_s
        while sa["history"] and sa["history"][0][0] < cutoff:
            sa["history"].popleft()

        # 判断是否到达岸边
        arrived = shore_d <= self.ka_shore_dist

        if arrived and sa["arrival_mono"] is None:
            # 首次到达：检查接近窗口内的运动特征
            sa["arrival_mono"] = now_mono
            hist_list = list(sa["history"])
            if len(hist_list) >= 3:
                first_mono, first_dist, fx, fy = hist_list[0]
                last_mono, last_dist, lx, ly = hist_list[-1]

                drop = first_dist - last_dist
                dur = last_mono - first_mono
                avg_speed = math.hypot(lx - fx, ly - fy) / max(dur, 1e-6)

                if avg_speed >= self.ka_fast_approach and drop >= self.ka_dist_drop:
                    sa["approached_fast"] = True

        if not arrived:
            sa["arrival_mono"] = None
            sa["approached_fast"] = False

        # 快速接近 + 到达后低速 → 报警
        if sa["approached_fast"] and arrived and sa["arrival_mono"] is not None:
            if spd <= self.ka_post_slow:
                dur = now_mono - sa["arrival_mono"]
                if dur >= self.ka_duration_s:
                    if not self._in_cooldown("KA", now_mono, tid):
                        self._emit("KA", "船靠岸送人上岸", [tid], dur, {
                            "path": "motion_feature",
                            "position": {"x": round(x, 2), "y": round(y, 2)},
                            "speed_m_s": round(spd, 3),
                            "shore_distance_m": round(shore_d, 3),
                        })
                        sa["approached_fast"] = False
                        sa["arrival_mono"] = None
                        return True
            else:
                # 又动了 → 重置
                sa["arrival_mono"] = None
                sa["approached_fast"] = False

        return False

    # ════════════════════════════════════════════════════════════
    # 事件输出
    # ════════════════════════════════════════════════════════════

    def _emit(self, event_type, event_label, track_ids, duration_s, details):
        self.event_count += 1
        event_time = datetime.now().isoformat(timespec="milliseconds")

        event = {
            "event_time": event_time,
            "event_type": event_type,
            "event_label": event_label,
            "track_ids": ",".join(str(t) for t in track_ids),
            "duration_s": f"{duration_s:.1f}",
            "details": json.dumps(details, ensure_ascii=False),
        }

        if self.csv_writer is None:
            self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8-sig")
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=list(event.keys()))
            self.csv_writer.writeheader()

        self.csv_writer.writerow(event)
        self.csv_file.flush()

        print(f"\n{'='*50}")
        print(f"[ALARM #{self.event_count}] [{event_type}] {event_label}")
        print(f"  track_ids: {track_ids}")
        print(f"  duration: {duration_s:.1f}s")
        print(f"  details: {json.dumps(details, ensure_ascii=False)}")
        print(f"{'='*50}")

        if self.on_anomaly:
            try:
                self.on_anomaly(event)
            except Exception as exc:
                print(f"  [callback failed] {exc}")

    # ════════════════════════════════════════════════════════════
    # 生命周期
    # ════════════════════════════════════════════════════════════

    def summary(self):
        return {
            "total_events": self.event_count,
            "active_rendezvous_pairs": len(self.rendezvous_pairs),
            "active_docking_tracks": len(self.docking_states),
            "history_tracks": len(self.track_history),
        }

    def is_active(self):
        """是否有异常正在持续中（用于驱动摄像头持续跟踪）"""
        for ps in self.rendezvous_pairs.values():
            if ps.get("close_start_mono") is not None:
                return True
        for ds in self.docking_states.values():
            if ds.get("state") in ("DEVIATED", "STOPPED") and ds.get("was_moving"):
                return True
        for st in self.ks_speed_timers.values():
            if st.get("speed_start_mono") is not None:
                return True
        return False

    def close(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None


# ============================================================
# 离线回放
# ============================================================

def replay_from_csv(csv_path, config, on_anomaly=None):
    """从 radar_targets.csv 回放，用于离线调参"""
    detector = AnomalyDetector(config, on_anomaly=on_anomaly)
    print(f"离线回放: {csv_path}")

    with open(csv_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print("CSV 为空")
        return detector

    # 按 packet_no 分组
    frames = OrderedDict()
    for row in rows:
        pn = int(row.get("packet_no", 0))
        if pn not in frames:
            frames[pn] = []
        tid = int(row.get("track_id", 0)) if row.get("track_id") else 0
        frames[pn].append({
            "track_id": tid,
            "x_m": float(row.get("x_m", 0)),
            "y_m": float(row.get("y_m", 0)),
            "speed_m_s": float(row.get("speed_m_s", 0)),
        })

    # 使用 packet_no 模拟时间流逝，每帧间隔 0.1 秒（与雷达 ~10Hz 更新率一致）
    total_frames = len(frames)
    for idx, (pn, targets) in enumerate(frames.items()):
        sim_time = idx * 0.1  # 模拟时间（秒），每帧 100ms
        detector.feed(targets, _now=sim_time)

    print(f"\n回放完成: {len(frames)} 帧, 检测到 {detector.event_count} 次异常")
    return detector


# ============================================================
# CLI
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="异常行为检测")
    parser.add_argument("--csv", default="", help="离线回放 CSV 路径")
    parser.add_argument("--config", default="live_config.json", help="配置文件路径")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    with open(config_path, "r", encoding="utf-8") as f:
        full_config = json.load(f)

    ad_config = {
        "zones": full_config.get("zones", {}),
        "anomaly_detection": full_config.get("anomaly_detection", {}),
    }

    # 创建带时间戳的输出文件夹：csv文件/20260626_143025/
    from datetime import datetime
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = config_path.parent / "csv文件" / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    csv_name = Path(ad_config["anomaly_detection"].get("csv", "anomaly_events.csv")).name
    ad_config["anomaly_detection"]["csv"] = str(out_dir / csv_name)

    if args.csv:
        detector = replay_from_csv(args.csv, ad_config)
    else:
        print("实时模式需由 radar_live.py 启动。离线回放请用 --csv 参数。")
        return

    detector.close()
    print(f"输出目录: {out_dir}")


if __name__ == "__main__":
    main()
