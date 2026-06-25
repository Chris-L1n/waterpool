#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
异常行为检测模块

检测三类异常行为：
  1) 船-船搭靠 (SC)   — 两船近距离并靠 ≥15秒
  2) 快速通过 (KS)     — 船只突然明显加速穿越区域
  3) 船靠岸送人上岸 (KA) — 偏航 → 进入非正规靠岸区 → 减速 → 岸边停留

------------------------------------------
异常1：船-船搭靠（SC）
行为描述：两船在水面上近距离并靠，船舷接触或贴近，保持不少于15秒。

1号船先行 → 2号船缓慢靠近 → 两船进入搭靠区 → 并靠保持≥15秒 → 系统报警

检测特征：

特征	判定条件
两船距离	相对距离 < 阈值（如0.5m）
持续时间	近距离状态持续 ≥ 15秒
速度变化	被靠近船减速至近乎停止
触发位置	两船均在搭靠区域内
------------------------------------------

------------------------------------------
异常2：快速通过（KS）
行为描述：船只突然明显加速，以异常高速穿越特定区域。

船只正常速度进入 → 在快速通过区突然明显加速 → 穿越后减速停止 → 系统报警

检测特征：

特征	判定条件
速度突增	当前速度明显高于历史平均速度（如 > 2倍）
绝对速度	速度超过正常基准阈值（如 > 1.5 m/s）
加速度	短时间内的加速度显著（如 Δv > 0.5 m/s²）
触发位置	加速发生在快速通过区域内
------------------------------------------

------------------------------------------
异常3：船靠岸送人上岸（KA）
行为描述：船只偏离正常航道，驶向非正规靠岸区，减速后在岸边停留，模拟非法送人上岸。

实验操作（文档第46-50行）：

船只沿正常航道行驶 → 偏航转向非正规靠岸区 → 减速 → 岸边停留10-15秒 → 系统报警

检测特征：

特征	判定条件
偏航	目标位置偏离正常航道中心线超过阈值
进入禁区	目标进入非正规靠岸区
减速	速度显著下降（如 < 0.2 m/s，近乎停止）
停留时间	在岸边区域停留 ≥ 10秒
综合	偏航 → 进入禁区 → 减速停留 三个阶段的时序组合
------------------------------------------
"""


"""
radar_track_parser_v4.py  在300多行
已有的雷达算法：

choose_nearest（多维过滤选最近目标）

SimpleTracker（最近邻多目标跟踪）

不会和异常行为检测算法冲突

but， SimpleTracker 的质量直接影响异常检测。
如果 track_id 关联错误（两个目标交叉后ID互换），异常检测也会出错。
但这是数据质量问题，不是算法冲突。
"""
import argparse
import csv
import json
import math
import os
import time
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path


# ============================================================
# 工具函数
# ============================================================

def point_in_polygon(x, y, polygon):
    """射线法判断点 (x, y) 是否在多边形内"""
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
    """判断点 (x, y) 是否在圆心 (cx, cy)、半径 radius 的圆内"""
    return math.hypot(x - cx, y - cy) <= radius


# ============================================================
# 异常检测器
# ============================================================

class AnomalyDetector:
    """三类异常行为实时检测器"""

    def __init__(self, config, on_anomaly=None):
        """
        config: 字典，包含 "zones" 和 "anomaly_detection" 两个顶层 key
        on_anomaly: 可选回调 (event_dict) -> None，用于联动 PTZ
        """
        # ── 读取区域配置 ──
        z = config.get("zones", {})
        self.normal_channel = z.get("normal_channel", {}).get("points", [])
        self.rendezvous_zone = z.get("rendezvous_zone", {}).get("points", [])
        self.fast_passage_zone = z.get("fast_passage_zone", {}).get("points", [])
        self.unauthorized_docking_zone = z.get("unauthorized_docking_zone", {}).get("points", [])
        ap = z.get("authorized_docking_point", {})
        self.auth_dock_pos = ap.get("position", None)
        self.auth_dock_radius = float(ap.get("radius_m", 0.3))

        # ── 读取检测参数 ──
        ad = config.get("anomaly_detection", {})
        self.enabled = bool(ad.get("enabled", True))

        sc = ad.get("ship_rendezvous", {})
        self.sc_dist_threshold = float(sc.get("distance_threshold_m", 0.5))
        self.sc_duration_threshold = float(sc.get("duration_threshold_s", 15.0))

        ks = ad.get("fast_passage", {})
        self.ks_speed_threshold = float(ks.get("speed_threshold_m_s", 1.5))
        self.ks_accel_threshold = float(ks.get("acceleration_threshold_m_s2", 0.3))
        self.ks_speed_ratio = float(ks.get("speed_ratio_threshold", 2.0))

        ka = ad.get("shore_docking", {})
        self.ka_deviation_threshold = float(ka.get("deviation_threshold_m", 0.3))
        self.ka_speed_threshold = float(ka.get("speed_threshold_m_s", 0.2))
        self.ka_duration_threshold = float(ka.get("duration_threshold_s", 10.0))

        # ── 冷却时间（同类型同目标不重复报警）──
        self.cooldowns = {
            "SC": float(ad.get("sc_cooldown_s", 60.0)),
            "KS": float(ad.get("ks_cooldown_s", 30.0)),
            "KA": float(ad.get("ka_cooldown_s", 60.0)),
        }

        # ── 事件输出 ──
        self.on_anomaly = on_anomaly
        self.csv_path = ad.get("csv", "anomaly_events.csv")
        self.csv_file = None
        self.csv_writer = None
        self.event_count = 0

        # ── 轨迹历史：track_id → deque(maxlen=300) ──
        self.track_history = defaultdict(lambda: deque(maxlen=300))
        self._mono = time.monotonic  # 用于计算时间间隔

        # ── 搭靠状态 ──
        self.rendezvous_pairs = {}  # (tid_a, tid_b) → {"close_start_mono", "tracks"}

        # ── 靠岸状态机 ──
        self.docking_states = {}    # track_id → state machine

        # ── 报警冷却 ──
        self.last_triggered = {}    # (event_type, ...) → mono_time

    # ════════════════════════════════════════════════════════════
    # 区域判断
    # ════════════════════════════════════════════════════════════

    def _in_zone(self, x, y, polygon):
        """点在多边形内；未配置多边形时默认视为区域内（不启用空间过滤）"""
        if not polygon or len(polygon) < 3:
            return True
        return point_in_polygon(x, y, polygon)

    def _in_rendezvous_zone(self, x, y):
        return self._in_zone(x, y, self.rendezvous_zone)

    def _in_fast_passage_zone(self, x, y):
        return self._in_zone(x, y, self.fast_passage_zone)

    def _in_unauthorized_docking_zone(self, x, y):
        return self._in_zone(x, y, self.unauthorized_docking_zone)

    def _in_authorized_docking(self, x, y):
        if self.auth_dock_pos is None:
            return False
        return point_in_circle(x, y, self.auth_dock_pos[0], self.auth_dock_pos[1], self.auth_dock_radius)

    def _distance_to_channel_center(self, x, y):
        """点到航道中心线的近似距离（为航道多边形中心X坐标的距离）"""
        if not self.normal_channel or len(self.normal_channel) < 3:
            return 0.0
        xs = [p[0] for p in self.normal_channel]
        center_x = (min(xs) + max(xs)) / 2.0
        return abs(x - center_x)

    # ════════════════════════════════════════════════════════════
    # 报警冷却
    # ════════════════════════════════════════════════════════════

    def _in_cooldown(self, event_type, *keys):
        """检查该事件类型+key组合是否仍在冷却期内；不在冷却期则记录当前时间"""
        event_key = (event_type,) + tuple(keys)
        now = self._mono()
        if event_key in self.last_triggered:
            if now - self.last_triggered[event_key] < self.cooldowns.get(event_type, 60.0):
                return True
        self.last_triggered[event_key] = now
        return False

    # ════════════════════════════════════════════════════════════
    # 主入口
    # ════════════════════════════════════════════════════════════

    def feed(self, targets):
        """
        每帧调用。
        targets: 带 track_id 字段的目标字典列表
                 [{track_id, x_m, y_m, speed_m_s, ...}, ...]
        """
        if not self.enabled:
            return
        if not targets:
            return

        now_mono = self._mono()

        # 更新轨迹历史（用于计算加速度、平均速度等）
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

        # 检查三种异常
        self._check_rendezvous(targets, now_mono)
        self._check_fast_passage(targets, now_mono)
        self._check_shore_docking(targets, now_mono)

    # ════════════════════════════════════════════════════════════
    # 异常1：船-船搭靠 (SC)
    # ════════════════════════════════════════════════════════════

    def _check_rendezvous(self, targets, now_mono):
        """
        检测逻辑：
          1) 两两配对计算距离
          2) 距离 < sc_dist_threshold 且两船均在搭靠区
          3) 持续 ≥ sc_duration_threshold → 报警
        """
        active = [t for t in targets if t.get("track_id") is not None]
        if len(active) < 2:
            return

        checked_pairs = set()

        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                a, b = active[i], active[j]
                ta, tb = a["track_id"], b["track_id"]
                pk = (min(ta, tb), max(ta, tb))
                checked_pairs.add(pk)

                dist = math.hypot(a["x_m"] - b["x_m"], a["y_m"] - b["y_m"])
                close = (
                    dist < self.sc_dist_threshold
                    and self._in_rendezvous_zone(a["x_m"], a["y_m"])
                    and self._in_rendezvous_zone(b["x_m"], b["y_m"])
                )

                if close:
                    if pk not in self.rendezvous_pairs:
                        self.rendezvous_pairs[pk] = {
                            "close_start": now_mono,
                            "tracks": (ta, tb),
                        }
                    else:
                        dur = now_mono - self.rendezvous_pairs[pk]["close_start"]
                        if dur >= self.sc_duration_threshold:
                            if not self._in_cooldown("SC", pk[0], pk[1]):
                                self._emit("SC", "船-船搭靠", [ta, tb], dur, {
                                    "distance_m": round(dist, 3),
                                    "track_a": {"x": round(a["x_m"], 2), "y": round(a["y_m"], 2),
                                                "speed": round(a["speed_m_s"], 2)},
                                    "track_b": {"x": round(b["x_m"], 2), "y": round(b["y_m"], 2),
                                                "speed": round(b["speed_m_s"], 2)},
                                })
                else:
                    # 两船分离或离开搭靠区 → 清除状态
                    self.rendezvous_pairs.pop(pk, None)

        # 清理已不存在的pair（目标消失）
        for pk in list(self.rendezvous_pairs):
            if pk not in checked_pairs:
                del self.rendezvous_pairs[pk]

    # ════════════════════════════════════════════════════════════
    # 异常2：快速通过 (KS)
    # ════════════════════════════════════════════════════════════

    def _check_fast_passage(self, targets, now_mono):
        """
        检测逻辑：
          1) 目标在快速通过区内
          2) 当前速度 > 绝对阈值
          3) 且满足以下之一：
             a) 当前速度 ≥ 历史平均速度 × speed_ratio（速度突增）
             b) 加速度 > accel_threshold（加速异常）
        """
        for t in targets:
            tid = t.get("track_id")
            if tid is None:
                continue
            if not self._in_fast_passage_zone(t["x_m"], t["y_m"]):
                continue

            hist = list(self.track_history[tid])
            if len(hist) < 3:
                continue

            cur_speed = abs(t["speed_m_s"])
            if cur_speed < self.ks_speed_threshold:
                continue

            # 历史平均速度（不含当前帧）
            prev_speeds = [abs(h["speed_m_s"]) for h in hist[:-1]]
            if not prev_speeds:
                continue
            avg_speed = sum(prev_speeds) / len(prev_speeds)

            # 速度突增倍率
            surge = False
            if avg_speed > 0.05 and cur_speed / avg_speed >= self.ks_speed_ratio:
                surge = True

            # 加速度（使用 monotonic 时间差，精确）
            prev = hist[-2]
            dt = now_mono - prev["mono"]
            accel = 0.0
            if 0.001 < dt <= 2.0:
                accel = (cur_speed - abs(prev["speed_m_s"])) / dt
            high_accel = accel > self.ks_accel_threshold

            if surge or high_accel:
                trigger_reason = "speed_surge" if surge else "high_acceleration"
                if not self._in_cooldown("KS", tid):
                    self._emit("KS", "快速通过", [tid], 0.0, {
                        "current_speed_m_s": round(cur_speed, 3),
                        "average_speed_m_s": round(avg_speed, 3),
                        "speed_ratio": round(cur_speed / max(avg_speed, 0.01), 2),
                        "acceleration_m_s2": round(accel, 3),
                        "trigger_reason": trigger_reason,
                        "position": {"x": round(t["x_m"], 2), "y": round(t["y_m"], 2)},
                    })

    # ════════════════════════════════════════════════════════════
    # 异常3：船靠岸送人上岸 (KA) —— 状态机
    # ════════════════════════════════════════════════════════════

    def _check_shore_docking(self, targets, now_mono):
        """
        状态机：NORMAL → DEVIATED → IN_ZONE → STOPPED → 报警

        - NORMAL:  目标在正常航道内
        - DEVIATED: 目标偏离航道中心线超过阈值
        - IN_ZONE:  目标进入非正规靠岸区
        - STOPPED:  目标减速至近乎停止且未在正规停靠点
        - STOPPED 持续 ≥ ka_duration_threshold → 报警

        任一阶段条件不再满足 → 回到 NORMAL
        """
        for t in targets:
            tid = t.get("track_id")
            if tid is None:
                continue

            x, y, spd = t["x_m"], t["y_m"], abs(t["speed_m_s"])
            in_zone = self._in_unauthorized_docking_zone(x, y)
            deviated = self._distance_to_channel_center(x, y) > self.ka_deviation_threshold
            in_auth = self._in_authorized_docking(x, y)

            # 初始化状态
            if tid not in self.docking_states:
                self.docking_states[tid] = {
                    "state": "NORMAL",
                    "deviated_mono": None,
                    "entered_mono": None,
                    "stopped_mono": None,
                }

            ds = self.docking_states[tid]
            state = ds["state"]

            # ── 状态转换 ──

            if state == "NORMAL":
                if deviated and not in_auth:
                    ds["state"] = "DEVIATED"
                    ds["deviated_mono"] = now_mono

            elif state == "DEVIATED":
                if in_zone:
                    ds["state"] = "IN_ZONE"
                    ds["entered_mono"] = now_mono
                elif not deviated:
                    # 回到航道 → 重置
                    ds["state"] = "NORMAL"
                    ds["deviated_mono"] = None

            elif state == "IN_ZONE":
                if spd < self.ka_speed_threshold and not in_auth:
                    ds["state"] = "STOPPED"
                    ds["stopped_mono"] = now_mono
                elif not in_zone:
                    # 离开区域 → 重置
                    ds["state"] = "NORMAL"
                    for k in ("deviated_mono", "entered_mono", "stopped_mono"):
                        ds[k] = None

            elif state == "STOPPED":
                if spd < self.ka_speed_threshold and in_zone and not in_auth:
                    dur = now_mono - (ds["stopped_mono"] or now_mono)
                    if dur >= self.ka_duration_threshold:
                        if not self._in_cooldown("KA", tid):
                            self._emit("KA", "船靠岸送人上岸", [tid], dur, {
                                "position": {"x": round(x, 2), "y": round(y, 2)},
                                "speed_m_s": round(spd, 3),
                                "deviation_m": round(self._distance_to_channel_center(x, y), 3),
                            })
                else:
                    # 重新移动或离开区域 → 状态机复位
                    if spd >= self.ka_speed_threshold or not in_zone:
                        ds["state"] = "NORMAL"
                        for k in ("deviated_mono", "entered_mono", "stopped_mono"):
                            ds[k] = None

    # ════════════════════════════════════════════════════════════
    # 事件输出
    # ════════════════════════════════════════════════════════════

    def _emit(self, event_type, event_label, track_ids, duration_s, details):
        """写入 CSV 并触发报警回调"""
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

        # 延迟打开 CSV（避免空文件）
        if self.csv_writer is None:
            self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8-sig")
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=list(event.keys()))
            self.csv_writer.writeheader()

        self.csv_writer.writerow(event)
        self.csv_file.flush()

        print(f"\n{'='*50}")
        print(f"⚠ 异常报警 #{self.event_count}: [{event_type}] {event_label}")
        print(f"  涉及目标 track_id: {track_ids}")
        print(f"  持续时间: {duration_s:.1f}s")
        print(f"  详情: {json.dumps(details, ensure_ascii=False)}")
        print(f"{'='*50}")

        if self.on_anomaly:
            try:
                self.on_anomaly(event)
            except Exception as exc:
                print(f"  [异常回调失败] {exc}")

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

    def close(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None


# ============================================================
# 离线回放模式：从 CSV 读取历史数据运行检测
# ============================================================

def replay_from_csv(csv_path, config, on_anomaly=None):
    """
    从 radar_targets.csv（或 radar_tracks.csv）回放数据，
    用于离线调试异常检测参数。
    """
    detector = AnomalyDetector(config, on_anomaly=on_anomaly)
    print(f"离线回放: {csv_path}")

    with open(csv_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print("CSV 为空")
        return detector

    # 按 packet_no 分组，模拟逐帧
    from collections import OrderedDict
    frames = OrderedDict()
    for row in rows:
        pn = int(row.get("packet_no", 0))
        if pn not in frames:
            frames[pn] = []
        frames[pn].append({
            "track_id": int(row.get("track_id", 0)) if row.get("track_id") else 0,
            "x_m": float(row.get("x_m", 0)),
            "y_m": float(row.get("y_m", 0)),
            "speed_m_s": float(row.get("speed_m_s", 0)),
            "index": int(row.get("target_index", 0)),
        })

    for pn, targets in frames.items():
        detector.feed(targets)

    print(f"\n回放完成: {len(frames)} 帧, "
          f"检测到 {detector.event_count} 次异常")
    return detector


# ============================================================
# CLI 入口（离线回放）
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="异常行为检测 — 离线回放或实时检测"
    )
    parser.add_argument("--csv", default="",
                        help="离线回放用 CSV 文件路径（radar_targets.csv 或 radar_tracks.csv）")
    parser.add_argument("--config", default="live_config.json",
                        help="包含 zones 和 anomaly_detection 的配置文件")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    with open(config_path, "r", encoding="utf-8") as f:
        full_config = json.load(f)

    # 构建 anomaly detector 所需的配置
    ad_config = {
        "zones": full_config.get("zones", {}),
        "anomaly_detection": full_config.get("anomaly_detection", {}),
    }
    # 确保 CSV 路径是绝对路径
    csv_val = ad_config["anomaly_detection"].get("csv", "anomaly_events.csv")
    if csv_val and not Path(csv_val).is_absolute():
        ad_config["anomaly_detection"]["csv"] = str(config_path.parent / csv_val)

    if args.csv:
        detector = replay_from_csv(args.csv, ad_config)
    else:
        print("实时模式需由 live_tracking.py 启动。离线回放请用 --csv 参数。")
        return

    detector.close()
    print("摘要:", json.dumps(detector.summary(), ensure_ascii=False))


if __name__ == "__main__":
    main()
