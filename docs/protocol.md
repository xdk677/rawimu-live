# RAWIMU / RAWIMUSX 帧格式（实测确认）

本文是 `src/rawimu_live.py` 解析时使用的帧结构，参数与 `formats.json` 里的字段表一一对应。

## 1. 帧结构

```
┌────────┬──────────────────────────┬──────────────┬──────────┐
│ 同步头  │ 报文头（长 28B / 短 12B）  │ 数据体 40B   │ CRC32 4B │
└────────┴──────────────────────────┴──────────────┴──────────┘
```

- 长头：`AA 44 12`，`body_len` 在偏移 8（u16，**不含 CRC**）
- 短头：`AA 44 13`（同类还有 `10/11`），`body_len` 在偏移 3（u8）
- 一帧总长 = 头长 + body_len + 4
- CRC32：覆盖「头 + 体」，**反射多项式 0xEDB88320、初值 0、末尾不取反**（不是标准 zlib CRC）

### 长头字段（28 B）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 3 | Sync | `AA 44 12` |
| 3 | 1 | header_len | 28 |
| 4 | 2 | msg_id | `0x010C`=RAWIMU，`0x05B6`=RAWIMUSX |
| 6 | 1 | type | — |
| 7 | 1 | port | — |
| 8 | 2 | body_len | 40 |
| 10 | 2 | seq | 序号 |
| 12 | 1 | idle | — |
| 13 | 1 | sync_byte | 采集软件常写随机值 |
| 14 | 2 | week | GPS 周（u16） |
| 16 | 4 | ms | 周内毫秒（u32） |
| 20 | 4 | receiver_status | u32 |
| 24 | 2 | reserved | — |
| 26 | 2 | version | 如 803 |

短头（12 B）：`sync(3) | body_len(1) | msg_id(2) | week(2) | ms(4)`。

## 2. 数据体（40 B）

### RAWIMU（0x010C，长头）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u32 | Week |
| 4 | f64 | Seconds_into_Week（**double，单位 s**） |
| 12 | u32 | IMU_Status |
| 16 | i32×3 | Z / -Y / X 加速度 |
| 28 | i32×3 | Z / -Y / X 角速度 |

### RAWIMUSX（0x05B6，短头）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u8 | imu_info |
| 1 | u8 | imu_type |
| 2 | u16 | Week |
| 4 | f64 | Seconds_into_Week |
| 12 | u32 | IMU_Status |
| 16 | i32×3 | Z / -Y / X 加速度 |
| 28 | i32×3 | Z / -Y / X 角速度 |

> 两种格式的加速度/角速度**物理顺序都是 Z、Y、X**，且报文里的 Y 本身就是 -Y，
> 程序按 `order: ["z","y","x"]` + `invert_y` 还原成标准 `[X, Y, Z]`。

## 3. IMU_Status 位域

| 位 | 含义 |
|---|---|
| bit 0–7 | 采样计数（每帧 +1，模 256） |
| bit 8–15 | 状态字节（厂商定义） |
| bit 16–31 | 温度原始值（**有符号 int16**） |

温度 = `int16(status >> 16) * temp_lsb + temp_offset`。

## 4. 量纲（定点换算）

| 量 | LSB | 备注 |
|---|---|---|
| 陀螺 | **1/65536 deg/s** | 32 位定点，16 位小数 |
| 加速度 | **0.5/65536 m/s²** | 同上；实测静止 \|a\|≈9.81 |
| 温度 | 0.01 °C | 随 IMU 型号不同（见 `imu_models.json`） |

> 仓库里 `GNSS_NAV/src/processing/load_imu_model.m` 的同类系数小了 100 倍
> （陀螺 `1/(100·2^16)`、加计 `1/200/2^16`），直接套用会让姿态“只跟重力走”、yaw 不动。
> 本文的系数是用「手转 95° 对比积分角」和「静止噪声量级」两条独立证据标定出来的。

## 5. 时间与 dt

- 用数据体的 `Seconds_into_Week`（double）做帧间差分，异常时退回 `1/freq`
- 头部的 `ms` 只用于显示/校验；`Week(u32)` 在启动阶段可能不可信，时间基准用头部 week(u16)

## 6. ASCII 日志（#RAWIMUA / #RAWIMUSB）

部分设备/配置直接输出 ASCII 形式（实测 100 Hz，921600 波特率）：

```
#RAWIMUA,COM3,0,98.3,FINESTEERING,1356,82.500,00000000,0399,816;2446,93852.015821000,11dd0080,1303471,-6343,-6566,6013,-3722,4165*0f02cca6
```

| 段 | 内容 |
|---|---|
| 日志头 | `#RAWIMUA, port, seq, idle%, time_status, week, seconds, receiver_status, reserved, sw_version` |
| `;` 之后的数据体 | `week, seconds, imu_status(hex), acc_z, acc_y, acc_x, gyr_z, gyr_y, gyr_x` |
| 末尾 | `*` + 8 位十六进制 CRC32 |

- **CRC**：覆盖「去掉前导 `#` 之后」的全部字符，算法与二进制帧相同（初值 0、末尾不取反）
- **定点**：与二进制完全一致（陀螺 1/65536 deg/s、加计 0.5/65536 m/s²）
- 数据体的 `week`/`seconds` 与日志头里的同名字段一致，解析用数据体那份
- 同一条流里通常还有 `#INSPVAXA`、`$GPGGA`、`#BESTGNSSPOSA` 等其它日志，工具只识别注册过的格式，其余仅统计（`--selftest` 会列出未识别前缀）

在 `formats.json` 里新增 ASCII 格式时用 `kind: "ascii"` + `prefix` + 下标 `idx`，见 `examples/formats.example.json`。