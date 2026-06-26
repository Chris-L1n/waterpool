# TODO — 待实现与待优化

> 2026-06-26

---

## 一、摄像头部分

### 1.1 转向预置位而非计算角度

**当前**：`live_tracking.py` 按目标坐标计算角度，连续转动跟踪。

**目标**：检测到异常 → 转到对应预置位 → 截图取证 → 回全局观察位。

| 异常 | 英文标签 | 对应预置位 |
|------|---------|-----------|
| 船-船搭靠 | **SC** | 搭靠区预置位 |
| 快速通过 | **KS** | 快速通过区预置位 |
| 船靠岸送人上岸 | **KA** | 非正规靠岸区预置位 |

**怎么实现**：
1. 乐橙摄像头固件中预先设置三个预置位，获取 preset ID
2. `imou_ptz.py` 增加 `move_to_preset(key, preset_id)` 方法
3. `live_tracking.py` 的 `on_anomaly()` 改为根据 `event_type` (SC/KS/KA) 选 preset，调用 `move_to_preset()`，截图，回全局位

### 1.2 预测式摄像机切换

**当前**：目标超出覆盖范围后被动切换。

**目标**：预测 1 秒后位置，提前判断当前摄像机能否跟上，不能则提前切。

### 1.3 摄像机物理参数未标定

`cameras[].position_m`、`heading_deg`、`max_range_m` 均为虚构值。需实测。

---

## 二、异常检测（SC / KS / KA）

### 2.1 SC 船-船搭靠 测试

| 当前状态 | 说明 |
|---------|------|
| 算法 | ✅ 已完成 |
| 配置 | 实验室调试版（duration=5s, approach=1m） |
| 待验证 | **需要两艘船同时下水**，确认 SimpleTracker 能稳定区分两个 track_id |

### 2.2 KS 快速通过 测试

| 当前状态 | 说明 |
|---------|------|
| 算法 | ✅ 已完成 |
| 配置 | 实验室调试版（speed>0.5m/s, dur>0.3s, surge/accel 已关闭） |
| 待验证 | 单船加速到 >0.5m/s 保持 0.3 秒 |

### 2.3 KA 船靠岸送人上岸 测试

| 当前状态 | 说明 |
|---------|------|
| 算法 | ✅ 已完成（双路径：偏航+减速停留 / 运动特征） |
| 配置 | 实验室版（was_moving 保护、dur=10s） |
| 待验证 | 船故意开到岸边、减速、停留 10 秒 |

---

## 三、雷达滤波部分

### 3.1 PV 阈值未精确标定

当前 `min_pv: 30` 是保守估计。建议采集空场+单船数据，画出 PV 分布直方图来确定精确阈值。

### 3.2 `is_active()` 灵敏度

已加 `was_moving` 保护。待两船实验时验证不会误触。

---

## 四、实验验证

### 4.1 基准测试（防止误报）

| 场景 | 预期 | 状态 |
|------|------|------|
| 空水池 | 不触发 SC / KS / KA | ✅ 已验证 |
| 单船正常慢速航行 | 不触发 KS（<0.5m/s） | 待确认 |
| 单船在航道内正常停靠 | 不触发 KA（未偏航） | 待确认 |

### 4.2 异常场景测试

| 场景 | 预期触发 | 状态 |
|------|---------|------|
| 两船并靠 5 秒 | **SC** | 待测试 |
| 船突然加速到 >0.5m/s | **KS** | 待测试 |
| 船偏航+岸边减速停 10 秒 | **KA** | 待测试 |

### 4.3 区域坐标未标定

`zones` 段中三个多边形（`normal_channel`、`unauthorized_docking_zone`、`authorized_docking_point`）均为模板值。

---

## 五、已完成

| 功能 | 文件 |
|------|------|
| **SC** 船-船搭靠检测 | `anomaly_detector.py` |
| **KS** 快速通过检测 | `anomaly_detector.py` |
| **KA** 船靠岸送人上岸检测 | `anomaly_detector.py` |
| SimpleTracker 目标跟踪 | `radar_track_parser_v4.py` |
| BoatTargetSelector 稳定目标选择 | `boat_target_filter.py` |
| 异常触发摄像头（is_active 持续跟踪） | `radar_live.py` + `live_tracking.py` |
| VOFA+ 实时可视化 | `radar_live.py` |
| 离线回放模式 | `anomaly_detector.py` CLI |
| 时间戳文件夹 | `live_tracking.py` |
| 全参数 JSON 配置 | `live_config.json` |
