# 异常检测 — 开发日志

---

## 已实现

### 雷达数据采集与解析

`radar_track_parser_v4.py`

- CAN 帧组包（RadarReassembler, feed() 方法，第214-266行）
- 二进制协议解析，提取 X/Y/速度/PV（parse_target_packet(), 第277-325行）
- 目标跟踪，最近邻匹配分配 track_id（SimpleTracker, 第355-433行）
- CSV 写入，每次启动输出到 `csv文件/{时间戳}/` 文件夹（CsvWriters, 第439-505行）

### VOFA+ 上位机实时可视化

`radar_live.py` 第64-78行（初始化UDP socket）+ 第97-106行（每帧发送数据）

`live_config.json` → `radar.vofa` 段控制开关

程序边跑边把目标 X/Y/速度/PV 通过 UDP 发给 VOFA+，实时看到雷达点云。

### 三种异常行为检测

`anomaly_detector.py`

| 异常 | 代码位置 | 判定逻辑 |
|------|---------|---------|
| SC 船-船搭靠 | `_check_rendezvous()` 第287-374行 | 两船距离<0.5m + 相对速度<0.3m/s + 接近距离下降≥0.5m + 持续15s → 报警 |
| KS 快速通过 | `_check_fast_passage()` 第380-451行 | 速度>1.5m/s + 持续≥2s +（速度突增≥2倍 或 加速度>0.3m/s²）→ 报警 |
| KA 船靠岸 | `_check_shore_docking()` 第461-600行 | 路径A：偏航+减速停留≥10s<br>路径B：快速接近岸边+到达后低速停留≥10s |

全海域检测，不限定区域。SC和KS不依赖zones配置。

### 异常输出

`anomaly_detector.py`

- `anomaly_events.csv` — 每行一个报警事件（第607-625行）
- 终端打印 — 报警时输出分隔线+详情（第628-633行）
- `on_anomaly(event)` 回调 — 预留给摄像头模块（第635-639行）

### 离线回放

`anomaly_detector.py` → `replay_from_csv()` 第663-697行

```powershell
py anomaly_detector.py --csv radar_targets.csv
```

不连雷达，拿历史CSV跑检测，用于调参和排查漏报。

### 参数配置

`live_config.json` → `anomaly_detection` 段（第71-134行）

所有阈值可调，改JSON即生效，不用改代码。每个参数的 `_xxx` 注释键写了设置说明。

---



----------------------------------------------------------------------------

## 未实现

### 摄像头由异常触发（不是持续跟踪）

目前摄像头是 `choose_nearest` → 每帧跟踪最近目标（`live_tracking.py` 第146-188行）。

项目目标：异常检测触发报警后才驱动摄像头转向对应预置位，平时摄像头停在全局观察位不动。**尚未实现。**

### 杂波过滤

雷达密闭房间内杂波多（墙壁/水面反射），当前未使用 PV 值做前置过滤。如果实验时误报多，第一件事就是在 `feed()` 入口加 PV 阈值过滤。

### 阈值未经水池实验验证

`anomaly_detection` 里的所有阈值（0.5m、15s、1.5m/s 等）是初始猜测值，需要用真实水池数据调参。
