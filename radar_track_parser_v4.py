#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CANalyst-II / ControlCAN.dll 雷达轨迹解析正式版 v4

功能：

 雷达输出的数据类型:
 X_cm	横向偏移（雷达右侧为正）	-32768 ~ +32767 cm
 Y_cm	纵向距离（雷达前方为正）	0 ~ 65535 cm
 speed_cm_s	径向速度（远离为正）	-32768 ~ +32767 cm/s
 PV	信号质量/信噪比	0 ~ 65535

 以及计算的:
 track_id
 distance_m   --目标距雷达的欧氏距离（勾股定理）
 match_distance_m   --当前帧目标与匹配轨迹上一帧位置的欧氏距离

-----------------------------------------------------------
  1) 用 VCI_CAN_OBJ 数组方式批量接收 CAN 帧（已验证能收到 0x421）
  2) 按雷达 CAN 分包协议重组成完整雷达包
  3) 解析命令码 130 的目标输出包
  4) 实时打印目标 X/Y/速度/PV，并按过滤条件选最近目标
  5) 保存 CSV：radar_targets.csv、radar_nearest.csv、radar_tracks.csv
  6) 使用最近邻方法为目标分配稳定 track_id
---------------------------------------------------------------
运行：
  py -3.11-32 radar_track_parser.py --raw
  py -3.11-32 radar_track_parser.py --channel 0 --baud 500
  py -3.11-32 radar_track_parser.py --csv radar_targets.csv

注意：运行前关闭 USB-CAN Tool，否则会占用设备。
"""

import argparse
import csv
import ctypes as ct
import math
import os
import sys
import time
from datetime import datetime

# ============================================================
# ControlCAN 基础定义
# ============================================================

VCI_USBCAN2 = 4

class VCI_INIT_CONFIG(ct.Structure):
    _fields_ = [
        ("AccCode", ct.c_uint32),
        ("AccMask", ct.c_uint32),
        ("Reserved", ct.c_uint32),
        ("Filter", ct.c_ubyte),
        ("Timing0", ct.c_ubyte),
        ("Timing1", ct.c_ubyte),
        ("Mode", ct.c_ubyte),
    ]

class VCI_CAN_OBJ(ct.Structure):
    _fields_ = [
        ("ID", ct.c_uint32),
        ("TimeStamp", ct.c_uint32),
        ("TimeFlag", ct.c_ubyte),
        ("SendType", ct.c_ubyte),
        ("RemoteFlag", ct.c_ubyte),
        ("ExternFlag", ct.c_ubyte),
        ("DataLen", ct.c_ubyte),
        ("Data", ct.c_ubyte * 8),
        ("Reserved", ct.c_ubyte * 3),
    ]

class VCI_BOARD_INFO(ct.Structure):
    _fields_ = [
        ("hw_Version", ct.c_uint16),
        ("fw_Version", ct.c_uint16),
        ("dr_Version", ct.c_uint16),
        ("in_Version", ct.c_uint16),
        ("irq_Num", ct.c_uint16),
        ("can_Num", ct.c_ubyte),
        ("str_Serial_Num", ct.c_ubyte * 20),
        ("str_hw_Type", ct.c_ubyte * 40),
        ("Reserved", ct.c_uint16 * 4),
    ]

TIMING = {
    1000: (0x00, 0x14),
    800:  (0x00, 0x16),
    500:  (0x00, 0x1C),
    250:  (0x01, 0x1C),
    125:  (0x03, 0x1C),
    100:  (0x04, 0x1C),
    50:   (0x09, 0x1C),
}

# ============================================================
# 雷达协议常量：来自老师给的 C++ 逻辑
# ============================================================

CAN_RADAR_RX_ID = 0x421       # 雷达 -> 主机
CAN_RADAR_TX_ID = 0x110       # 主机 -> 雷达
RADAR_CMD_TARGET_OUTPUT = 130 # 0x82 目标输出包

CAN_SEG_FIRST = 0
CAN_SEG_MID = 1
CAN_SEG_LAST = 2
CAN_SEG_SINGLE = 0x3F
CAN_RADAR_PACKET_MAX = 4096

# ============================================================
# DLL / CAN 打开关闭
# ============================================================

def load_dll():
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "ControlCAN.dll"),
        r"C:\Program Files (x86)\USB_CAN TOOL\ControlCAN.dll",
        r"C:\USBCAN\library\ControlCAN.dll",
    ]
    for p in candidates:
        if os.path.exists(p):
            print("加载DLL:", p)
            return ct.WinDLL(p)
    raise FileNotFoundError("找不到 ControlCAN.dll，请把它复制到脚本同目录")


def set_prototypes(dll):
    # 只设置返回值，避免不同 DLL 版本因 argtypes 过严而 ArgumentError
    dll.VCI_OpenDevice.restype = ct.c_uint32
    dll.VCI_CloseDevice.restype = ct.c_uint32
    dll.VCI_InitCAN.restype = ct.c_uint32
    dll.VCI_StartCAN.restype = ct.c_uint32
    dll.VCI_ResetCAN.restype = ct.c_uint32
    dll.VCI_ClearBuffer.restype = ct.c_uint32
    dll.VCI_ReadBoardInfo.restype = ct.c_uint32
    dll.VCI_GetReceiveNum.restype = ct.c_uint32
    dll.VCI_Receive.restype = ct.c_int
    try:
        dll.VCI_Transmit.restype = ct.c_uint32
    except Exception:
        pass


def open_can(dll, channel: int, baud: int) -> bool:
    dt, di, ci = VCI_USBCAN2, 0, channel

    ret = dll.VCI_OpenDevice(ct.c_uint32(dt), ct.c_uint32(di), ct.c_uint32(0))
    print("OpenDevice=", ret)
    if ret != 1:
        print("✗ 打开设备失败：请关闭 USB-CAN Tool / 其他 Python，再拔插 USB 重试")
        return False

    info = VCI_BOARD_INFO()
    ret_info = dll.VCI_ReadBoardInfo(ct.c_uint32(dt), ct.c_uint32(di), ct.byref(info))
    hw = bytes(info.str_hw_Type).decode("gbk", errors="ignore").strip("\x00")
    sn = bytes(info.str_Serial_Num).decode("gbk", errors="ignore").strip("\x00")
    print(f"ReadBoardInfo={ret_info} 设备:{hw} SN:{sn} CAN数:{info.can_Num}")

    t0, t1 = TIMING[baud]

    # 按你 USB-CAN Tool 截图配置：AccCode=0x80000000, AccMask=0xFFFFFFFF, 正常模式
    cfg = VCI_INIT_CONFIG(
        AccCode=0x80000000,
        AccMask=0xFFFFFFFF,
        Reserved=0,
        Filter=1,
        Timing0=t0,
        Timing1=t1,
        Mode=0,
    )

    try:
        dll.VCI_ResetCAN(ct.c_uint32(dt), ct.c_uint32(di), ct.c_uint32(ci))
    except Exception:
        pass

    ret_init = dll.VCI_InitCAN(ct.c_uint32(dt), ct.c_uint32(di), ct.c_uint32(ci), ct.byref(cfg))
    print("InitCAN=", ret_init, f"CH{channel} baud={baud}K Timing0=0x{t0:02X} Timing1=0x{t1:02X}")
    if ret_init != 1:
        print("✗ 初始化 CAN 失败")
        return False

    try:
        dll.VCI_ClearBuffer(ct.c_uint32(dt), ct.c_uint32(di), ct.c_uint32(ci))
    except Exception:
        pass

    ret_start = dll.VCI_StartCAN(ct.c_uint32(dt), ct.c_uint32(di), ct.c_uint32(ci))
    print("StartCAN=", ret_start)
    if ret_start != 1:
        print("✗ 启动 CAN 失败")
        return False
    return True


def close_can(dll):
    try:
        dll.VCI_CloseDevice(ct.c_uint32(VCI_USBCAN2), ct.c_uint32(0))
        print("CloseDevice done")
    except Exception:
        pass

# ============================================================
# 雷达组包与解析
# ============================================================

class RadarReassembler:
    """CAN 分段 -> 完整雷达协议包"""
    def __init__(self, on_packet, verbose_reset=False):
        self.on_packet = on_packet
        self.verbose_reset = verbose_reset
        self.buf = bytearray()
        self.expected_seg = -1
        self.receiving = False  # 是否正在组包
        self.reset_count = 0

    def reset(self, reason=""):
        if reason and self.verbose_reset:
            print(f"[组包重置] {reason}")
        self.buf.clear()
        self.expected_seg = -1
        self.receiving = False
        self.reset_count += 1

    def feed(self, can_id: int, data: bytes, timestamp: int = 0):
        if can_id != CAN_RADAR_RX_ID:
            return
        if not data:
            return

        seg_type = (data[0] >> 6) & 0x03
        seg_count = data[0] & 0x3F
        payload = data[1:]

        if seg_type == CAN_SEG_FIRST:
            if seg_count == CAN_SEG_SINGLE:
                self.on_packet(bytes(payload), timestamp)
                return
            if seg_count != 0:
                self.reset(f"首段序号不是0: {seg_count}")
                return
            self.buf = bytearray(payload)
            self.expected_seg = 1
            self.receiving = True
            return

        if seg_type == CAN_SEG_MID:
            if not self.receiving:
                self.reset("收到中间段但当前未在组包")
                return
            if seg_count != self.expected_seg:
                self.reset(f"中间段序号不连续: got={seg_count}, expect={self.expected_seg}")
                return
            if len(self.buf) + len(payload) > CAN_RADAR_PACKET_MAX:
                self.reset("组包超过最大长度")
                return
            self.buf.extend(payload)
            self.expected_seg += 1
            return

        if seg_type == CAN_SEG_LAST:
            if not self.receiving:
                self.reset("收到末段但当前未在组包")
                return
            if seg_count != self.expected_seg:
                self.reset(f"末段序号不连续: got={seg_count}, expect={self.expected_seg}")
                return
            if len(self.buf) + len(payload) > CAN_RADAR_PACKET_MAX:
                self.reset("组包超过最大长度")
                return
            self.buf.extend(payload)
            packet = bytes(self.buf)
            self.reset()
            self.on_packet(packet, timestamp)
            return

        self.reset(f"未知分段类型: {seg_type}")


def u16_le(b: bytes) -> int:
    return int.from_bytes(b, "little", signed=False)


def i16_le(b: bytes) -> int:
    return int.from_bytes(b, "little", signed=True)


def parse_target_packet(packet: bytes):
    """
    解析雷达命令码 130 目标输出。
    C++ 逻辑：data_len=(frame[5]<<8)|frame[4], target_data_start=frame+8, target_count=(data_len-1)/8
    """
    if len(packet) < 9:
        return None, "包长不足"
    if packet[0] != 0xA5 or packet[1] != 0xA5:
        return None, "包头不是 A5 A5"
    if packet[2] != RADAR_CMD_TARGET_OUTPUT:
        return None, f"非目标输出命令 cmd={packet[2]}"

    data_len = (packet[5] << 8) | packet[4]
    # C++源码逻辑：data_len = 目标点数据 + 1字节校验；目标从 frame + 8 开始
    # 注意：实际完整包长度常见为 6 + data_len + 1，而不是 8 + data_len。
    # 旧版这里用 len(packet) < 8 + data_len 会把正常包误判为“不完整”，导致目标包=0、CSV为空。
    if data_len < 1:
        return None, f"data_len异常: {data_len}"

    target_count = (data_len - 1) // 8
    if target_count <= 0:
        return [], None

    targets = []
    base = 8
    required_len = base + target_count * 8
    if len(packet) < required_len:
        return None, f"目标数据不完整 len={len(packet)} data_len={data_len} target_count={target_count} required={required_len}"
    for i in range(target_count):
        off = base + i * 8
        if off + 8 > len(packet):
            break
        raw = packet[off:off+8]
        x_cm = i16_le(raw[0:2])
        y_cm = u16_le(raw[2:4])
        speed_cm_s = i16_le(raw[4:6])
        pv = u16_le(raw[6:8])
        targets.append({
            "index": i,
            "x_cm": x_cm,
            "y_cm": y_cm,
            "speed_cm_s": speed_cm_s,
            "pv": pv,
            "x_m": x_cm / 100.0,
            "y_m": y_cm / 100.0,
            "speed_m_s": speed_cm_s / 100.0,
            "distance_m": math.sqrt((x_cm / 100.0) ** 2 + (y_cm / 100.0) ** 2),
        })
    return targets, None


# ============================================================
# 简单目标跟踪：最近邻匹配 + 短暂丢失保留
# ============================================================

def choose_nearest(targets, x_max_cm=500, y_min_cm=500, speed_max_cm_s=0,
                   enable_pv_filter=True, pv_near_y_max_cm=1000, pv_min_near=40,
                   pv_mid_y_max_cm=2000, pv_min_mid=30):
    """按 C++ 里的过滤逻辑选择最近目标：|X|<x_max, Y>y_min, speed<=speed_max, PV分段过滤。"""
    best = None
    for t in targets:
        x, y, speed, pv = t["x_cm"], t["y_cm"], t["speed_cm_s"], t["pv"]
        if abs(x) >= x_max_cm:
            continue
        if y <= y_min_cm:
            continue
        if speed > speed_max_cm_s:
            continue
        if enable_pv_filter:
            if y <= pv_near_y_max_cm and pv <= pv_min_near:
                continue
            if pv_near_y_max_cm < y <= pv_mid_y_max_cm and pv <= pv_min_mid:
                continue
        if best is None or y < best["y_cm"]:
            best = t
    return best


# ============================================================
# 简单目标跟踪：最近邻匹配 + 短暂丢失保留
# ============================================================

class SimpleTracker:
    """
    给没有ID的雷达目标分配跨帧稳定的 track_id。

    说明：
      - target_index 只是当前帧里的临时编号，不是固定目标ID。
      - track_id 是程序根据连续帧位置接近程度生成的跟踪编号。
      - 最近邻匹配 + 轨迹重连：新目标出现时先检查是否是最近刚丢的旧轨迹，是则继承旧ID。
      - EMA 位置平滑：每帧对活跃轨迹做指数移动平均，输出 smooth_x_m/smooth_y_m。
    """
    def __init__(self, match_threshold_m=0.4, max_missed=3, ema_alpha=0.25):
        self.match_threshold_m = float(match_threshold_m)
        self.max_missed = int(max_missed)
        self.ema_alpha = float(ema_alpha)  # EMA 平滑系数，约 4 帧收敛
        self.next_track_id = 1
        self.active = {}       # track_id -> {last_x, last_y, sm_x, sm_y, last_time, missed}
        self.history = {}      # track_id -> list[所有目标点]
        self._recently_lost = {}  # track_id -> {last_x, last_y, sm_x, sm_y, lost_at_packet}

    @staticmethod
    def _dist(a, b):
        return math.hypot(a["x_m"] - b["last_x"], a["y_m"] - b["last_y"])

    def _try_reconnect(self, target, packet_no):
        """检查新目标是否是最近丢失的轨迹回来了。是则返回旧 track_id，否则返回 None。"""
        best_id = None
        best_d = self.match_threshold_m * 2.0  # 重连距离放宽一倍
        for tid, info in list(self._recently_lost.items()):
            frames_lost = packet_no - info["lost_at_packet"]
            if frames_lost > self.max_missed * 3:
                del self._recently_lost[tid]
                continue
            d = math.hypot(target["x_m"] - info["last_x"], target["y_m"] - info["last_y"])
            if d < best_d:
                best_id = tid
                best_d = d
        return best_id

    def update(self, targets, packet_no, pc_time=None):
        """输入当前帧 targets，返回添加 track_id 后的新 targets。"""
        if pc_time is None:
            pc_time = datetime.now().isoformat(timespec="milliseconds")

        unmatched_tracks = set(self.active.keys())
        output = []

        current_targets = sorted(targets, key=lambda t: t.get("distance_m", 999999))

        for t in current_targets:
            best_id = None
            best_d = None
            for tid in list(unmatched_tracks):
                d = self._dist(t, self.active[tid])
                if d <= self.match_threshold_m and (best_d is None or d < best_d):
                    best_id = tid
                    best_d = d

            if best_id is None:
                # 无活跃轨迹匹配 → 检查是否是最近丢失的轨迹回来了
                recon_id = self._try_reconnect(t, packet_no)
                if recon_id is not None:
                    best_id = recon_id
                    best_d = None
                    # 恢复平滑位置
                    sm_x = self._recently_lost[recon_id].get("sm_x", t["x_m"])
                    sm_y = self._recently_lost[recon_id].get("sm_y", t["y_m"])
                else:
                    best_id = self.next_track_id
                    self.next_track_id += 1
                    self.history[best_id] = []
                    sm_x, sm_y = t["x_m"], t["y_m"]
            else:
                unmatched_tracks.discard(best_id)
                # EMA 平滑
                old_sm = self.active[best_id]
                alpha = self.ema_alpha
                sm_x = old_sm.get("sm_x", t["x_m"]) * (1 - alpha) + t["x_m"] * alpha
                sm_y = old_sm.get("sm_y", t["y_m"]) * (1 - alpha) + t["y_m"] * alpha

            tt = dict(t)
            tt["track_id"] = best_id
            tt["match_distance_m"] = 0.0 if best_d is None else best_d
            tt["pc_time"] = pc_time
            tt["packet_no"] = packet_no
            tt["smooth_x_m"] = round(sm_x, 4)
            tt["smooth_y_m"] = round(sm_y, 4)
            output.append(tt)

            # 更新活动轨迹
            self.active[best_id] = {
                "last_x": t["x_m"],
                "last_y": t["y_m"],
                "sm_x": sm_x,
                "sm_y": sm_y,
                "last_time": pc_time,
                "missed": 0,
            }
            self.history.setdefault(best_id, []).append(tt)
            # 重连成功后从丢失列表移除
            self._recently_lost.pop(best_id, None)

        # 对本帧未匹配的轨迹，missed + 1；超过 max_missed → 移入丢失列表
        for tid in list(unmatched_tracks):
            self.active[tid]["missed"] += 1
            if self.active[tid]["missed"] > self.max_missed:
                info = self.active.pop(tid)
                self._recently_lost[tid] = {
                    "last_x": info["last_x"],
                    "last_y": info["last_y"],
                    "sm_x": info.get("sm_x", info["last_x"]),
                    "sm_y": info.get("sm_y", info["last_y"]),
                    "lost_at_packet": packet_no,
                }

        output.sort(key=lambda t: t["index"])
        return output

    def summary(self):
        return {tid: len(points) for tid, points in self.history.items()}

# ============================================================
# CSV 输出
# ============================================================

class CsvWriters:
    def __init__(self, target_csv, nearest_csv, tracks_csv, filtered_csv=None):
        self.target_f = open(target_csv, "w", newline="", encoding="utf-8-sig") if target_csv else None
        self.nearest_f = open(nearest_csv, "w", newline="", encoding="utf-8-sig") if nearest_csv else None
        self.tracks_f = open(tracks_csv, "w", newline="", encoding="utf-8-sig") if tracks_csv else None
        self.filtered_f = open(filtered_csv, "w", newline="", encoding="utf-8-sig") if filtered_csv else None
        self.target_w = None
        self.nearest_w = None
        self.tracks_w = None
        self.filtered_w = None

        common_fields = [
            "pc_time", "packet_no", "target_index", "track_id", "match_distance_m",
            "x_cm", "y_cm", "speed_cm_s", "pv", "x_m", "y_m", "speed_m_s", "distance_m",
        ]
        if self.target_f:
            self.target_w = csv.DictWriter(self.target_f, fieldnames=common_fields)
            self.target_w.writeheader()
        if self.nearest_f:
            self.nearest_w = csv.DictWriter(self.nearest_f, fieldnames=common_fields)
            self.nearest_w.writeheader()
        if self.tracks_f:
            self.tracks_w = csv.DictWriter(self.tracks_f, fieldnames=common_fields)
            self.tracks_w.writeheader()
        if self.filtered_f:
            self.filtered_w = csv.DictWriter(self.filtered_f, fieldnames=common_fields)
            self.filtered_w.writeheader()

    def _row(self, packet_no, t, pc_time=None):
        if pc_time is None:
            pc_time = t.get("pc_time") or datetime.now().isoformat(timespec="milliseconds")
        return {
            "pc_time": pc_time,
            "packet_no": packet_no,
            "target_index": t["index"],
            "track_id": t.get("track_id", ""),
            "match_distance_m": f'{t.get("match_distance_m", 0.0):.3f}' if "match_distance_m" in t else "",
            "x_cm": t["x_cm"],
            "y_cm": t["y_cm"],
            "speed_cm_s": t["speed_cm_s"],
            "pv": t["pv"],
            "x_m": f'{t["x_m"]:.3f}',
            "y_m": f'{t["y_m"]:.3f}',
            "speed_m_s": f'{t["speed_m_s"]:.3f}',
            "distance_m": f'{t["distance_m"]:.3f}',
        }

    def write_targets(self, packet_no, targets):
        now = datetime.now().isoformat(timespec="milliseconds")
        for t in targets:
            row = self._row(packet_no, t, now)
            if self.target_w:
                self.target_w.writerow(row)
            if self.tracks_w:
                self.tracks_w.writerow(row)
        if self.target_f:
            self.target_f.flush()
        if self.tracks_f:
            self.tracks_f.flush()

    def write_nearest(self, packet_no, t):
        if not self.nearest_w or t is None:
            return
        self.nearest_w.writerow(self._row(packet_no, t))
        self.nearest_f.flush()

    def write_filtered(self, packet_no, targets):
        if not self.filtered_w:
            return
        now = datetime.now().isoformat(timespec="milliseconds")
        for t in targets:
            self.filtered_w.writerow(self._row(packet_no, t, now))
        self.filtered_f.flush()

    def close(self):
        if self.target_f:
            self.target_f.close()
        if self.nearest_f:
            self.nearest_f.close()
        if self.tracks_f:
            self.tracks_f.close()
        if self.filtered_f:
            self.filtered_f.close()

# ============================================================
# 主循环
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", type=int, default=0, help="0=CAN1, 1=CAN2")
    parser.add_argument("--baud", type=int, default=500, choices=sorted(TIMING.keys()))
    parser.add_argument("--seconds", type=int, default=0, help="运行秒数；0表示一直运行")
    parser.add_argument("--batch", type=int, default=2500, help="数组接收长度，默认2500")
    parser.add_argument("--wait-ms", type=int, default=20, help="VCI_Receive等待时间ms")
    parser.add_argument("--raw", action="store_true", help="打印原始CAN帧")
    parser.add_argument("--packet", action="store_true", help="打印完整协议包HEX")
    parser.add_argument("--no-csv", action="store_true", help="不保存CSV")
    parser.add_argument("--csv", default="radar_targets.csv", help="全部目标CSV文件名")
    parser.add_argument("--nearest-csv", default="radar_nearest.csv", help="最近目标CSV文件名")
    parser.add_argument("--tracks-csv", default="radar_tracks.csv", help="带track_id的轨迹CSV文件名")
    parser.add_argument("--track-threshold-m", type=float, default=0.4, help="Track ID最近邻匹配阈值，单位m，默认0.4")
    parser.add_argument("--max-missed", type=int, default=3, help="目标短暂消失时保留轨迹的帧数，默认3帧")
    parser.add_argument("--print-all", action="store_true", help="打印每个包内全部目标；默认只打印最近目标和目标数")

    # 过滤参数：默认不过滤，所有目标都显示和写入 CSV；nearest 从全部目标中选距离最近的
    parser.add_argument("--x-max-cm", type=int, default=999999, help="横向过滤阈值，默认不过滤")
    parser.add_argument("--y-min-cm", type=int, default=0, help="纵向过滤阈值，默认Y>0cm")
    parser.add_argument("--speed-max-cm-s", type=int, default=999999, help="速度过滤阈值，默认不限制速度")
    parser.add_argument("--pv-filter", action="store_true", help="启用PV/SNR分段过滤；默认关闭")
    parser.add_argument("--pv-near-y-max-cm", type=int, default=1000)
    parser.add_argument("--pv-min-near", type=int, default=40)
    parser.add_argument("--pv-mid-y-max-cm", type=int, default=2000)
    parser.add_argument("--pv-min-mid", type=int, default=30)

    args = parser.parse_args()

    print("雷达轨迹解析器V4：默认不过滤 + Track ID跟踪 + 数组接收 + 目标解析")
    print("=" * 70)
    print("VCI_CAN_OBJ size =", ct.sizeof(VCI_CAN_OBJ))
    print("VCI_INIT_CONFIG size =", ct.sizeof(VCI_INIT_CONFIG))
    print(f"雷达RX ID=0x{CAN_RADAR_RX_ID:X}, CMD_TARGET={RADAR_CMD_TARGET_OUTPUT}")

    dll = load_dll()
    set_prototypes(dll)

    writers = None
    tracker = SimpleTracker(match_threshold_m=args.track_threshold_m, max_missed=args.max_missed)
    packet_no = 0
    can_frame_no = 0
    target_packet_no = 0
    start_time = time.time()
    last_msg = start_time

    if not args.no_csv:
        writers = CsvWriters(args.csv, args.nearest_csv, args.tracks_csv)
        print(f"CSV保存: {args.csv}, {args.nearest_csv}, {args.tracks_csv}")

    def on_packet(packet: bytes, timestamp: int):
        nonlocal packet_no, target_packet_no
        packet_no += 1
        if args.packet:
            print(f"\n[完整包#{packet_no}] len={len(packet)} {packet.hex(' ').upper()}")

        targets, err = parse_target_packet(packet)
        if err:
            if args.packet:
                print("  跳过:", err)
            return

        target_packet_no += 1
        if targets is None:
            return

        # 为当前帧目标分配稳定的 track_id
        targets = tracker.update(targets, target_packet_no)

        nearest = choose_nearest(
            targets,
            x_max_cm=args.x_max_cm,
            y_min_cm=args.y_min_cm,
            speed_max_cm_s=args.speed_max_cm_s,
            enable_pv_filter=args.pv_filter,
            pv_near_y_max_cm=args.pv_near_y_max_cm,
            pv_min_near=args.pv_min_near,
            pv_mid_y_max_cm=args.pv_mid_y_max_cm,
            pv_min_mid=args.pv_min_mid,
        )

        if writers:
            writers.write_targets(target_packet_no, targets)
            writers.write_nearest(target_packet_no, nearest)

        print(f"\n[目标包#{target_packet_no}] 目标数={len(targets)}", end="")
        if nearest:
            print(f"  最近目标#{nearest['index']} track={nearest.get('track_id', '')} X={nearest['x_m']:+.2f}m Y={nearest['y_m']:.2f}m "
                  f"V={nearest['speed_m_s']:+.2f}m/s PV={nearest['pv']}")
        else:
            print("  无目标通过过滤")

        if args.print_all:
            for t in targets:
                mark = " ★" if nearest and t["index"] == nearest["index"] else ""
                print(f"  #{t['index']:02d} track={t.get('track_id', '')} X={t['x_m']:+.2f}m Y={t['y_m']:.2f}m "
                      f"V={t['speed_m_s']:+.2f}m/s PV={t['pv']}{mark}")

    reasm = RadarReassembler(on_packet)

    ArrType = VCI_CAN_OBJ * args.batch
    arr = ArrType()

    ok = False
    try:
        ok = open_can(dll, args.channel, args.baud)
        if not ok:
            return

        print("\n开始接收... Ctrl+C 停止")
        print("提示：默认不过滤；加 --print-all 看每个目标；加 --raw 看原始帧")

        while True:
            if args.seconds > 0 and time.time() - start_time >= args.seconds:
                break

            try:
                num = dll.VCI_GetReceiveNum(ct.c_uint32(VCI_USBCAN2), ct.c_uint32(0), ct.c_uint32(args.channel))
            except Exception:
                num = 0

            ret = dll.VCI_Receive(
                ct.c_uint32(VCI_USBCAN2),
                ct.c_uint32(0),
                ct.c_uint32(args.channel),
                arr,
                ct.c_uint32(args.batch),
                ct.c_int(args.wait_ms),
            )

            if ret and ret > 0:
                for i in range(ret):
                    obj = arr[i]
                    dlc = int(obj.DataLen)
                    data = bytes(obj.Data[:dlc])
                    can_frame_no += 1
                    if args.raw:
                        print(f"[CAN#{can_frame_no:06d}] cache={num} ID=0x{obj.ID:X} LEN={dlc} "
                              f"EXT={obj.ExternFlag} REM={obj.RemoteFlag} TS={obj.TimeStamp} "
                              f"DATA={data.hex(' ').upper()}")
                    reasm.feed(int(obj.ID), data, int(obj.TimeStamp))
                last_msg = time.time()
            else:
                if time.time() - last_msg > 3:
                    print(f"...3秒内未收到，GetReceiveNum={num}, ReceiveRet={ret}")
                    last_msg = time.time()
                time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        close_can(dll)
        if writers:
            writers.close()
        print(f"\n已停止：CAN帧={can_frame_no}，完整包={packet_no}，目标包={target_packet_no}")
        if not args.no_csv:
            print(f"CSV已保存：{args.csv}, {args.nearest_csv}, {args.tracks_csv}")
            print("轨迹统计(track_id: 点数)：", tracker.summary())

if __name__ == "__main__":
    main()
