# -*- coding: utf-8 -*-
"""
RAWIMU / RAWIMUSX 串口实时姿态显示（Python + Tkinter，可打包成 exe）

与 MATLAB 版 rawimu_live.m 功能对应：
  · 串口/文件回放收帧，同步头 + CRC 校验
  · 解 RAWIMU(0x010C) 与 RAWIMUSX(0x05B6)，换算物理量
  · 陀螺四元数积分出姿态（重力只做 roll/pitch 慢修正，永不修 yaw）
  · 三维姿态场景（飞机 + 罗盘台 + 航向指针 + 影子）+ 曲线 + 状态面板
  · 开机静止段自动估零偏、自动标定加计标度

用法：
    RawImuLive.exe                              # 启动后选串口
    RawImuLive.exe --port COM49 --baud 460800
    RawImuLive.exe --replay ..\rawimu_test.dat  # 回放录制文件
    python rawimu_live.py --selftest --replay ..\rawimu_test.dat --truth truth.csv
"""

from __future__ import print_function

import argparse
import json
import math
import os
import struct
import sys
import threading
import time
import traceback
from collections import deque

G = 9.80665

# ---------------------------------------------------------------- 默认配置
DEFAULT_CFG = {
    "port": "COM49",
    "baud": 460800,
    "imu_model": "M1633",
    "mode": "complementary",     # complementary | gyro
    "invert_y": True,
    "axis_map": [0, 1, 2],
    "axis_sign": [1, 1, 1],
    "gyro_lsb": 1.0 / 65536.0,            # deg/s per LSB
    "accel_lsb": 0.5 / 65536.0,           # m/s^2 per LSB
    "temp_lsb": 0.01,
    "temp_offset": 0.0,
    "freq": 100.0,
    "bias_sec": 3.0,
    "auto_bias": True,
    "auto_accel_scale": True,
    "level_tau": 2.0,
    "accel_leveling": True,
    "window_s": 30.0,
    "refresh_hz": 20.0,
    "view_az": -125.0,
    "view_el": -26.0,
    "replay_speed": 1.0,
}

# ---------------------------------------------------------------- IMU 型号表
# 加新型号：**不用改代码**，编辑程序目录下的 imu_models.json 即可（格式见该文件内说明）。
# 每个型号可覆盖的字段：gyro_lsb / accel_lsb / temp_lsb / temp_offset / freq /
#   invert_y / axis_map / axis_sign / note
IMU_MODELS = {
    "M1633": {"gyro_lsb": 1.0 / 65536.0, "accel_lsb": 0.5 / 65536.0,
              "temp_lsb": 0.01, "temp_offset": 0.0, "freq": 100.0,
              "note": "实测默认（COM49 那台 RAWIMUSX，32 位定点 16 位小数）"},
    "B240": {"gyro_lsb": 1.0 / 65536.0, "accel_lsb": 0.5 / 65536.0,
             "temp_lsb": 1.0 / 256.0, "temp_offset": 25.0, "freq": 100.0,
             "note": "B240，温度 1/256"},
    "B240_16BIT": {"gyro_lsb": 1.0 / 100.0, "accel_lsb": G / 2000.0,
                   "temp_lsb": 1.0 / 256.0, "temp_offset": 25.0, "freq": 100.0,
                   "note": "16 位定标版本（不是 32 位定点）"},
    "B200": {"gyro_lsb": 1.0 / 65536.0, "accel_lsb": 0.5 / 5000.0 / 65536.0 * 100,
             "temp_lsb": 1.0 / 200.0, "temp_offset": 50.0, "freq": 100.0},
    "X2_2": {"gyro_lsb": 100.0 / 80.0 / 65536.0, "accel_lsb": G / 4905.0 / 65536.0,
             "temp_lsb": 1.0 / 30.0, "temp_offset": 25.0, "freq": 100.0},
}
IMU_MODEL_FIELDS = ("gyro_lsb", "accel_lsb", "temp_lsb", "temp_offset", "freq",
                    "invert_y", "axis_map", "axis_sign", "note")

IMU_MODELS_TEMPLATE = {
    "_说明": "新增 IMU 型号：复制一组，改名字和参数即可；同名会覆盖内置值。"
            " gyro_lsb 单位 deg/s per LSB，accel_lsb 单位 m/s^2 per LSB，"
            " temp_lsb 单位 °C per LSB。axis_map/axis_sign 用于非标准安装（一般不用改）。",
    "MY_IMU_A": {
        "gyro_lsb": 1.52587890625e-05,
        "accel_lsb": 7.62939453125e-06,
        "temp_lsb": 0.01,
        "temp_offset": 0.0,
        "freq": 100,
        "invert_y": True,
        "axis_map": [0, 1, 2],
        "axis_sign": [1, 1, 1],
        "note": "示例：32 位定点 16 位小数"
    },
    "MY_IMU_B": {
        "gyro_lsb": 0.001,
        "accel_lsb": 0.0005,
        "temp_lsb": 0.00390625,
        "temp_offset": 25.0,
        "freq": 125,
        "invert_y": False,
        "note": "示例：16 位定标、125 Hz、Y 轴不取反"
    }
}


def _json_candidates(fname):
    """程序目录（exe 所在）和当前目录都找，源码/exe 两种形态都适用"""
    paths = []
    try:
        here = os.path.dirname(os.path.abspath(sys.argv[0]))
    except Exception:
        here = ""
    for p in (os.path.join(here, fname) if here else None,
              os.path.join(os.getcwd(), fname)):
        if p and p not in paths:
            paths.append(p)
    return paths


def load_imu_models(paths=None):
    """内置表 + imu_models.json 里的自定义型号"""
    models = {k: dict(v) for k, v in IMU_MODELS.items()}
    for p in (paths if paths is not None else _json_candidates("imu_models.json")):
        if not p or not os.path.isfile(p):
            continue
        try:
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print("[imu_models] 读取失败 %s: %s" % (p, e))
            continue
        for name, spec in (data or {}).items():
            if name.startswith("_") or not isinstance(spec, dict):
                continue
            cur = dict(models.get(name, {}))
            cur.update({k: v for k, v in spec.items() if k in IMU_MODEL_FIELDS})
            models[name] = cur
    return models


def apply_model(cfg, models, name):
    """把某个型号的系数写进 cfg；未知型号保持原样并返回 False"""
    spec = models.get(name)
    if spec is None:
        return False
    cfg["imu_model"] = name
    for k in IMU_MODEL_FIELDS:
        if k in spec and k != "note":
            cfg[k] = spec[k]
    return True

# ---------------------------------------------------------------- CRC32
_CRC_TABLE = None


def _crc_table():
    global _CRC_TABLE
    if _CRC_TABLE is None:
        t = []
        for k in range(256):
            c = k
            for _ in range(8):
                c = (0xEDB88320 ^ (c >> 1)) if (c & 1) else (c >> 1)
            t.append(c)
        _CRC_TABLE = t
    return _CRC_TABLE


def crc32_raw(data):
    """设备用的 CRC32：反射多项式 0xEDB88320、初值 0、末尾不取反"""
    t = _crc_table()
    crc = 0
    for b in bytearray(data):
        crc = t[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc & 0xFFFFFFFF


# ---------------------------------------------------------------- 帧解析
RAWIMU_ID = 0x010C
RAWIMUSX_ID = 0x05B6


class FrameParser(object):
    """字节流 -> 帧；按消息 ID 查格式注册表，未注册的只计数（可抓样本用于逆向）"""

    def __init__(self, on_frame, registry):
        self.on_frame = on_frame
        self.reg = registry
        self.buf = bytearray()
        self.n_bytes = 0
        self.n_frame = 0
        self.n_crc = 0
        self.n_drop = 0
        self.n_other = 0
        self.n_short = 0
        self.id_counts = {}
        self.fmt_counts = {}
        self.unknown = []          # [(msg_id, hlen, body_len, hex_head)] 最多 8 条

    def feed(self, chunk):
        self.buf += chunk
        self.n_bytes += len(chunk)
        buf = self.buf
        n = len(buf)
        i = 0
        while i <= n - 16:
            if buf[i] != 0xAA or buf[i + 1] != 0x44:
                i += 1
                self.n_drop += 1
                continue
            s3 = buf[i + 2]
            if s3 == 0x12:
                hlen = 28
            elif s3 in (0x13, 0x10, 0x11):
                hlen = 12
            else:
                i += 1
                self.n_drop += 1
                continue
            if n - i < hlen + 4:
                break
            if hlen == 28:
                ln = buf[i + 8] | (buf[i + 9] << 8)
            else:
                ln = buf[i + 3]
            total = hlen + ln + 4
            if n - i < total:
                break
            body = buf[i + hlen:i + hlen + ln]
            crc_rx = struct.unpack_from("<I", buf, i + total - 4)[0]
            if crc32_raw(buf[i:i + hlen + ln]) != crc_rx:
                self.n_crc += 1
                self.n_drop += 1
                i += 1
                continue
            msg_id = buf[i + 4] | (buf[i + 5] << 8)
            self.n_frame += 1
            self.id_counts[msg_id] = self.id_counts.get(msg_id, 0) + 1
            spec = self.reg.by_id.get(msg_id)
            if spec is not None and ln == spec["body_len"] and hlen == spec["header_len"]:
                name = spec["name"]
                self.fmt_counts[name] = self.fmt_counts.get(name, 0) + 1
                self.on_frame(spec, body)
            elif hlen == 28:
                self.n_other += 1
                self._remember(msg_id, hlen, ln, body)
            else:
                self.n_short += 1
                self._remember(msg_id, hlen, ln, body)
            i += total
        del buf[:i]
        if len(buf) > 1 << 20:
            del buf[:-4096]
            self.n_drop += 1

    def _remember(self, msg_id, hlen, ln, body):
        """记下未识别报文的样本，方便照着填 formats.json"""
        if len(self.unknown) >= 8:
            return
        if any(u[0] == msg_id for u in self.unknown):
            return
        self.unknown.append((msg_id, hlen, ln, bytes(body[:56]).hex()))


def fmt_text(counts):
    """报文格式计数 -> 一行短文本"""
    if not counts:
        return "-"
    items = sorted(counts.items(), key=lambda kv: -kv[1])
    return " ".join("%s×%d" % (k, v) for k, v in items[:3])


def to_physical(raw, spec, cfg):
    """原始计数 -> 物理量（轴向映射、Y 取反、量纲换算、温度/计数器）"""
    acc = list(raw.get("acc", [0.0, 0.0, 0.0]))
    gyr = list(raw.get("gyr", [0.0, 0.0, 0.0]))
    if cfg["invert_y"]:
        acc[1] = -acc[1]
        gyr[1] = -gyr[1]
    m = cfg["axis_map"]
    sg = cfg["axis_sign"]
    acc = [acc[m[0]] * sg[0], acc[m[1]] * sg[1], acc[m[2]] * sg[2]]
    gyr = [gyr[m[0]] * sg[0], gyr[m[1]] * sg[1], gyr[m[2]] * sg[2]]

    status = raw.get("status", 0) & 0xFFFFFFFF
    traw = (status >> 16) & 0xFFFF
    if traw >= 0x8000:
        traw -= 0x10000
    temp = traw * cfg["temp_lsb"] + cfg["temp_offset"]
    return {
        "kind": spec["name"], "week": raw.get("week", 0), "sec": raw.get("sow", 0.0),
        "status": status, "acc_raw": acc, "gyr_raw": gyr,
        "acc": [acc[0] * cfg["accel_lsb"], acc[1] * cfg["accel_lsb"], acc[2] * cfg["accel_lsb"]],
        "gyr": [gyr[0] * cfg["gyro_lsb"], gyr[1] * cfg["gyro_lsb"], gyr[2] * cfg["gyro_lsb"]],
        "temp": temp, "counter": status & 0xFF, "stat_byte": (status >> 8) & 0xFF,
        "imu_info": raw.get("imu_info"), "imu_type": raw.get("imu_type"),
    }


# ---------------------------------------------------------------- 报文格式（可扩展）
# 每种 IMU 报文用"字段表"描述；新增格式改程序目录下的 formats.json，不用改代码。
def _rd(body, off, typ):
    if typ == "u8":
        return body[off]
    if typ == "i8":
        return struct.unpack_from("<b", body, off)[0]
    if typ == "u16":
        return struct.unpack_from("<H", body, off)[0]
    if typ == "i16":
        return struct.unpack_from("<h", body, off)[0]
    if typ == "u32":
        return struct.unpack_from("<I", body, off)[0]
    if typ == "i32":
        return struct.unpack_from("<i", body, off)[0]
    if typ == "f32":
        return struct.unpack_from("<f", body, off)[0]
    if typ == "f64":
        return struct.unpack_from("<d", body, off)[0]
    if typ == "i32x3":
        return list(struct.unpack_from("<3i", body, off))
    raise ValueError("未知字段类型: %s" % typ)


def _order_xyz(raw, order):
    """把报文里的 [c0,c1,c2] 按名字重排成 [x,y,z]；order 形如 ["z","y","x"]"""
    pos = {"x": 0, "y": 1, "z": 2}
    out = [0.0, 0.0, 0.0]
    for k, name in enumerate(order):
        out[pos[name]] = float(raw[k])
    return out


def build_decoder(spec):
    """由字段表生成解码函数：body -> 原始计数"""
    def decode(body):
        d = {"name": spec.get("name", "?")}
        d["week"] = _rd(body, spec["week"]["off"], spec["week"]["type"]) if spec.get("week") else 0
        d["sow"] = _rd(body, spec["sow"]["off"], spec["sow"]["type"])
        d["status"] = _rd(body, spec["status"]["off"], spec["status"]["type"]) if spec.get("status") else 0
        for key in ("acc", "gyr"):
            f = spec.get(key)
            if f:
                d[key] = _order_xyz(_rd(body, f["off"], f.get("type", "i32x3")),
                                    f.get("order", ["x", "y", "z"]))
        for key in ("imu_info", "imu_type"):
            f = spec.get(key)
            d[key] = _rd(body, f["off"], f["type"]) if f else None
        return d
    return decode


FORMAT_SPECS = {
    "RAWIMU": {
        "name": "RAWIMU", "id": 0x010C, "header_len": 28, "body_len": 40,
        "week": {"off": 0, "type": "u32"},
        "sow": {"off": 4, "type": "f64"},
        "status": {"off": 12, "type": "u32"},
        "acc": {"off": 16, "order": ["z", "y", "x"]},
        "gyr": {"off": 28, "order": ["z", "y", "x"]},
        "note": "NovAtel/Bynav RAWIMU：长头 28B + 体 40B",
    },
    "RAWIMUSX": {
        "name": "RAWIMUSX", "id": 0x05B6, "header_len": 12, "body_len": 40,
        "week": {"off": 2, "type": "u16"},
        "sow": {"off": 4, "type": "f64"},
        "status": {"off": 12, "type": "u32"},
        "imu_info": {"off": 0, "type": "u8"},
        "imu_type": {"off": 1, "type": "u8"},
        "acc": {"off": 16, "order": ["z", "y", "x"]},
        "gyr": {"off": 28, "order": ["z", "y", "x"]},
        "note": "NovAtel/Bynav RAWIMUSX：短头 12B + 体 40B",
    },
}

FORMATS_TEMPLATE = {
    "_说明": "新增 IMU 报文格式：复制一组，改 id / header_len / body_len / 偏移即可，不用改代码。"
            "id 用十进制（0x010C=268）。header_len 28=长头、12=短头。"
            "acc/gyr 的 order 是报文里的物理顺序，如 [\"z\",\"y\",\"x\"]。"
            "类型支持 u8/i8/u16/i16/u32/i32/f32/f64/i32x3。",
    "RAWIMUA": {
        "name": "RAWIMUA", "id": 269, "header_len": 28, "body_len": 40,
        "week": {"off": 0, "type": "u16"},
        "sow": {"off": 2, "type": "u32"},
        "status": {"off": 6, "type": "u32"},
        "acc": {"off": 10, "order": ["z", "y", "x"]},
        "gyr": {"off": 22, "order": ["z", "y", "x"]},
        "enabled": False,
        "note": "示例：RAWIMUA(0x010D)，偏移按手册核对后把 enabled 改成 true"
    },
    "RAWIMUSB": {
        "name": "RAWIMUSB", "id": 270, "header_len": 28, "body_len": 40,
        "week": {"off": 0, "type": "u16"},
        "sow": {"off": 2, "type": "u32"},
        "status": {"off": 6, "type": "u32"},
        "acc": {"off": 10, "order": ["z", "y", "x"]},
        "gyr": {"off": 22, "order": ["z", "y", "x"]},
        "enabled": False,
        "note": "示例：RAWIMUSB(0x010E)，偏移按手册核对后把 enabled 改成 true"
    }
}


class FormatRegistry(object):
    """报文格式注册表：内置两种 + formats.json 里自定义的"""

    def __init__(self, specs=None, json_paths=None):
        self.specs = {}
        self.by_id = {}
        self.errors = []
        for s in (specs or FORMAT_SPECS).values():
            self.add(s)
        for p in (json_paths if json_paths is not None else _json_candidates("formats.json")):
            self.load_json(p)

    def add(self, spec):
        spec = dict(spec)
        if "id" not in spec or "sow" not in spec:
            self.errors.append("格式缺少 id/sow: %r" % (spec.get("name"),))
            return False
        spec.setdefault("header_len", 28)
        spec.setdefault("body_len", 40)
        spec.setdefault("name", "ID_%04X" % int(spec["id"]))
        try:
            spec["decode"] = build_decoder(spec)
        except Exception as e:
            self.errors.append("%s 字段表有误: %s" % (spec["name"], e))
            return False
        self.specs[spec["name"]] = spec
        self.by_id[int(spec["id"]) & 0xFFFF] = spec
        return True

    def load_json(self, path):
        if not path or not os.path.isfile(path):
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            self.errors.append("读取 %s 失败: %s" % (path, e))
            return
        for name, spec in (data or {}).items():
            if name.startswith("_") or not isinstance(spec, dict):
                continue
            if spec.get("enabled", True) is False:   # 示例格式默认不启用，核对偏移后再打开
                continue
            spec.setdefault("name", name)
            self.add(spec)


# ---------------------------------------------------------------- 姿态数学
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def quat_to_dcm(q):
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def quat_to_euler(q):
    R = quat_to_dcm(q)
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    yaw = math.atan2(R[1][0], R[0][0])
    roll = math.atan2(R[2][1], R[2][2])
    return roll, pitch, yaw


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return quat_normalize((cy * cp * cr + sy * sp * sr,
                           cy * cp * sr - sy * sp * cr,
                           cy * sp * cr + sy * cp * sr,
                           sy * cp * cr - cy * sp * sr))


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Attitude(object):
    """陀螺积分 + 可选重力慢修正"""

    def __init__(self, cfg):
        self.cfg = cfg
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.bias = [0.0, 0.0, 0.0]
        self.bias_acc = [0.0, 0.0, 0.0]
        self.bias_n = 0
        self.bias_done = False
        self.acc_sum = 0.0
        self.acc_n = 0
        self.acc_cal_done = False
        self.ang_acc = [0.0, 0.0, 0.0]
        self.roll = self.pitch = self.yaw = 0.0
        self.roll_a = self.pitch_a = 0.0
        self.log = []

    def reset_attitude(self):
        self.q = (1.0, 0.0, 0.0, 0.0)

    def reset_calibration(self):
        self.bias = [0.0, 0.0, 0.0]
        self.bias_acc = [0.0, 0.0, 0.0]
        self.bias_n = 0
        self.bias_done = False
        self.acc_sum = 0.0
        self.acc_n = 0
        self.acc_cal_done = False

    def update(self, smp, dt):
        cfg = self.cfg
        g = smp["gyr"]
        a = smp["acc"]
        araw = smp["acc_raw"]

        # 加计标度自校准（开头静止段 |a| 归一到 g）
        if cfg["auto_accel_scale"] and not self.acc_cal_done:
            self.acc_sum += math.sqrt(araw[0] ** 2 + araw[1] ** 2 + araw[2] ** 2)
            self.acc_n += 1
            need = max(10, int(round(cfg["bias_sec"] * cfg["freq"])))
            if self.acc_n >= need:
                lsb = G / (self.acc_sum / self.acc_n)
                if 0.5 * cfg["accel_lsb"] < lsb < 2.0 * cfg["accel_lsb"]:
                    cfg["accel_lsb"] = lsb
                    self.log.append("加计标度自校准: %.6g m/s2/LSB" % lsb)
                else:
                    self.log.append("加计自校准跳过(非静止?): %.3g" % lsb)
                self.acc_cal_done = True
                # 用新标度重算本帧
                a = [araw[0] * cfg["accel_lsb"], araw[1] * cfg["accel_lsb"], araw[2] * cfg["accel_lsb"]]
                smp["acc"] = a

        # 陀螺零偏
        if cfg["auto_bias"] and not self.bias_done:
            for k in range(3):
                self.bias_acc[k] += g[k]
            self.bias_n += 1
            if self.bias_n >= max(10, int(round(cfg["bias_sec"] * cfg["freq"]))):
                n = float(self.bias_n)
                self.bias = [self.bias_acc[k] / n for k in range(3)]
                self.bias_done = True
                self.log.append("零偏: %.4f %.4f %.4f deg/s" % tuple(self.bias))

        w = [g[k] - self.bias[k] for k in range(3)]
        for k in range(3):
            self.ang_acc[k] += w[k] * dt

        # 1) 陀螺积分（姿态主体）
        wrad = [math.radians(v) for v in w]
        wn = math.sqrt(sum(v * v for v in wrad))
        th = wn * dt
        if th > 1e-12:
            axis = [v / wn for v in wrad]
            s = math.sin(th / 2)
            dq = (math.cos(th / 2), s * axis[0], s * axis[1], s * axis[2])
            self.q = quat_normalize(quat_mul(self.q, dq))

        roll, pitch, yaw = quat_to_euler(self.q)

        # 2) 重力方向：参考曲线 + 可选慢修正
        an = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        acc_ok = abs(an - G) < 0.6
        if acc_ok:
            ra = math.atan2(a[1], a[2])
            pa = math.atan2(-a[0], math.hypot(a[1], a[2]))
            # 取与积分 roll 最近的等价分支，避免 ±180° 跳变（这就是参考线竖条毛刺的来源）
            ra += 2.0 * math.pi * round((roll - ra) / (2.0 * math.pi))
            self.roll_a, self.pitch_a = ra, pa
        else:
            # 有线性加速度/失重时重力方向不可信：参考线留空，别画成假数据
            self.roll_a = self.pitch_a = float("nan")
            pa = None

        if (cfg["accel_leveling"] and cfg["mode"] == "complementary"
                and pa is not None and abs(math.degrees(wn)) < 20.0):
            # 四元数域反馈（Mahony 式）：比较“预测重力方向 vs 实测重力方向”，
            # 用叉积做小角度修正——连续、无欧拉角奇异，只修倾斜，永不修 yaw
            Rd = quat_to_dcm(self.q)
            gp = (Rd[2][0], Rd[2][1], Rd[2][2])        # R^T·(0,0,1) = 第三行
            gm = (a[0] / an, a[1] / an, a[2] / an)
            e = cross(gp, gm)
            ang = norm(e)
            if ang > 1e-9:
                kk = (dt / max(0.05, cfg["level_tau"])) * ang
                ax = (e[0] / ang, e[1] / ang, e[2] / ang)
                half = -0.5 * kk
                s = math.sin(half)
                self.q = quat_normalize(quat_mul(
                    self.q, (math.cos(half), s * ax[0], s * ax[1], s * ax[2])))
                roll, pitch, yaw = quat_to_euler(self.q)
        self.roll, self.pitch, self.yaw = roll, pitch, yaw
        return roll, pitch, yaw


# ---------------------------------------------------------------- 数据源
class Session(object):
    """共享状态：环形缓冲 + 统计 + 校准"""

    def __init__(self, cfg, formats=None):
        self.cfg = cfg
        self.lock = threading.Lock()
        n = 6000
        self.t = deque(maxlen=n)
        self.ch = [deque(maxlen=n) for _ in range(9)]   # roll,pitch,yaw,wx,wy,wz,ax,ay,az
        self.rollA = deque(maxlen=n)
        self.pitchA = deque(maxlen=n)
        self.att = Attitude(cfg)
        self.formats = formats if formats is not None else FormatRegistry()
        self.parser = FrameParser(self._on_frame, self.formats)
        self.running = False
        self.error = ""
        self.status = ""
        self.week = None
        self.prev_t = None
        self.t0 = None
        self.t_last = 0.0
        self.n_imu = 0
        self.dt_sum = 0.0
        self.dt_n = 0
        self.rate_hz = 0.0
        self.temp = float("nan")
        self.acc_norm_sum = 0.0
        self.acc_norm_n = 0
        self.counter_prev = None
        self.counter_steps = {}
        self.stat_bytes = set()
        self.smp = None
        self.csv_file = None
        self.recording = False
        self.csv_name = ""

    # 帧回调（解析线程里调用）
    def _on_frame(self, spec, body):
        cfg = self.cfg
        try:
            smp = to_physical(spec["decode"](body), spec, cfg)
        except Exception:
            return
        tAbs = smp["week"] * 604800.0 + smp["sec"]
        if self.prev_t is not None:
            dt = tAbs - self.prev_t
            if not (0.0 < dt < 0.5):
                dt = 1.0 / cfg["freq"]
        else:
            dt = 1.0 / cfg["freq"]
        self.prev_t = tAbs
        if self.t0 is None:
            self.t0 = tAbs
        tRel = tAbs - self.t0

        roll, pitch, yaw = self.att.update(smp, dt)

        with self.lock:
            self.t.append(tRel)
            vals = (math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                    smp["gyr"][0] - self.att.bias[0],
                    smp["gyr"][1] - self.att.bias[1],
                    smp["gyr"][2] - self.att.bias[2],
                    smp["acc"][0], smp["acc"][1], smp["acc"][2])
            for k in range(9):
                self.ch[k].append(vals[k])
            self.rollA.append(math.degrees(self.att.roll_a))
            self.pitchA.append(math.degrees(self.att.pitch_a))
            self.smp = smp
            self.n_imu += 1
            self.dt_sum += dt
            self.dt_n += 1
            self.t_last = tRel
            self.temp = smp["temp"]
            an = math.sqrt(sum(v * v for v in smp["acc"]))
            self.acc_norm_sum += an
            self.acc_norm_n += 1
            self.stat_bytes.add(smp["stat_byte"])
            if self.counter_prev is not None:
                step = (smp["counter"] - self.counter_prev) % 256
                self.counter_steps[step] = self.counter_steps.get(step, 0) + 1
            self.counter_prev = smp["counter"]
            if self.n_imu > 5:
                span = self.t[-1] - self.t[max(0, len(self.t) - 101)]
                if span > 0.05:
                    self.rate_hz = (min(100, len(self.t) - 1)) / span
            if self.recording and self.csv_file:
                self.csv_file.write("%.6f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.2f,%u,%u,%u,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\r\n"
                                   % (tRel, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5],
                                      vals[6], vals[7], vals[8], smp["temp"], smp["status"],
                                      smp["stat_byte"], smp["counter"],
                                      smp["gyr_raw"][0], smp["gyr_raw"][1], smp["gyr_raw"][2],
                                      smp["acc_raw"][0], smp["acc_raw"][1], smp["acc_raw"][2]))

    # 录制
    def start_record(self, path):
        self.csv_file = open(path, "w")
        self.csv_file.write("t_s,roll,pitch,yaw,wx,wy,wz,ax,ay,az,temp,status,stat_byte,counter,"
                            "gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw\r\n")
        self.csv_name = path
        self.recording = True

    def stop_record(self):
        self.recording = False
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
            self.csv_file = None

    def snapshot(self):
        with self.lock:
            return (list(self.t), [list(c) for c in self.ch],
                    list(self.rollA), list(self.pitchA), self.att, self.smp)


class SerialWorker(threading.Thread):
    def __init__(self, sess):
        threading.Thread.__init__(self)
        self.daemon = True
        self.s = sess

    def run(self):
        sess = self.s
        try:
            import serial
        except Exception as e:
            sess.error = "pyserial 不可用: %s" % e
            return
        try:
            ser = serial.Serial(sess.cfg["port"], int(sess.cfg["baud"]), timeout=0.05)
        except Exception as e:
            sess.error = "打开串口失败: %s" % e
            return
        sess.running = True
        sess.status = "已连接 %s @ %d" % (sess.cfg["port"], sess.cfg["baud"])
        try:
            while sess.running:
                n = ser.in_waiting
                if n:
                    sess.parser.feed(ser.read(min(n, 65536)))
                else:
                    time.sleep(0.002)
        except Exception as e:
            sess.error = "串口读取异常: %s" % e
        finally:
            sess.running = False
            try:
                ser.close()
            except Exception:
                pass


class ReplayWorker(threading.Thread):
    def __init__(self, sess, path, speed=1.0):
        threading.Thread.__init__(self)
        self.daemon = True
        self.s = sess
        self.path = path
        self.speed = speed

    def run(self):
        sess = self.s
        try:
            f = open(self.path, "rb")
        except Exception as e:
            sess.error = "打开文件失败: %s" % e
            return
        sess.running = True
        sess.status = "回放 %s" % os.path.basename(self.path)
        chunk = 4096
        try:
            while sess.running:
                data = f.read(chunk)
                if not data:
                    break
                sess.parser.feed(data)
                if self.speed and self.speed > 0:
                    time.sleep(chunk / (72.0 * sess.cfg["freq"] * self.speed))
        except Exception as e:
            sess.error = "回放异常: %s" % e
        finally:
            f.close()
            sess.running = False


# ---------------------------------------------------------------- 三维场景
def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(v):
    return math.sqrt(dot(v, v))


def unit(v):
    n = norm(v) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def rotate(R, v):
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


def rotor(axis, angle):
    """Rodrigues 旋转矩阵"""
    x, y, z = unit(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
            (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
            (z * x * C - y * s, z * y * C + x * s, c + z * z * C))


def mat_mul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def tube(xs, rs, cy, cz, n=10):
    """沿 X 轴的圆管，返回 (verts, faces)"""
    verts = []
    for i, x in enumerate(xs):
        for j in range(n):
            a = 2 * math.pi * j / n + math.pi / n
            verts.append((x, cy + rs[i] * math.cos(a), cz + rs[i] * math.sin(a)))
    faces = []
    for i in range(len(xs) - 1):
        for j in range(n):
            j2 = (j + 1) % n
            a1 = i * n + j
            a2 = i * n + j2
            b1 = (i + 1) * n + j
            b2 = (i + 1) * n + j2
            faces.append((a1, b1, b2, a2))
    return verts, faces


def aircraft_parts():
    """机体轴 X 前 / Y 右 / Z 下；返回 [(verts, faces, rgb)]"""
    parts = []
    v, f = tube([-1.15, -0.85, -0.35, 0.55, 1.00], [0.015, 0.060, 0.095, 0.095, 0.055], 0, 0)
    parts.append((v, f, (245, 245, 250)))
    v, f = tube([1.00, 1.34], [0.055, 0.010], 0, 0)
    parts.append((v, f, (220, 45, 45)))
    parts.append(([(0.35, 0.10, -0.02), (-0.30, 0.10, -0.02), (-0.55, 1.05, -0.11), (-0.10, 1.05, -0.11)],
                  [(0, 1, 2, 3)], (188, 198, 212)))
    parts.append(([(0.35, -0.10, -0.02), (-0.30, -0.10, -0.02), (-0.55, -1.05, -0.11), (-0.10, -1.05, -0.11)],
                  [(0, 1, 2, 3)], (188, 198, 212)))
    parts.append(([(-0.80, 0.09, -0.01), (-1.02, 0.09, -0.01), (-1.16, 0.42, -0.04), (-1.00, 0.42, -0.04)],
                  [(0, 1, 2, 3)], (198, 206, 218)))
    parts.append(([(-0.80, -0.09, -0.01), (-1.02, -0.09, -0.01), (-1.16, -0.42, -0.04), (-1.00, -0.42, -0.04)],
                  [(0, 1, 2, 3)], (198, 206, 218)))
    parts.append(([(-0.62, 0, -0.04), (-1.10, 0, -0.04), (-1.14, 0, -0.60), (-0.86, 0, -0.60)],
                  [(0, 1, 2, 3)], (60, 96, 158)))
    v, f = tube([0.30, -0.34], [0.075, 0.075], 0.42, 0.16, 8)
    parts.append((v, f, (108, 112, 122)))
    v, f = tube([0.30, -0.34], [0.075, 0.075], -0.42, 0.16, 8)
    parts.append((v, f, (108, 112, 122)))
    out = []
    for v, f, c in parts:
        out.append(([(p[0] * 1.25, p[1] * 1.25, p[2] * 1.25) for p in v], f, c))
    return out


class Scene3D(object):
    """在 Tk Canvas 上画的简版 3D 场景（正交投影 + 画家算法 + 平面着色）"""

    def __init__(self, canvas, cfg):
        self.c = canvas
        self.cfg = cfg
        self.parts = aircraft_parts()
        self.al = 2.20
        self.zg = 0.76 * self.al
        self.rp = 0.90 * self.al
        self.rc = 0.80 * self.rp
        self.w = canvas.winfo_reqwidth()
        self.h = canvas.winfo_reqheight()
        self.scale = 90.0
        self.light = unit((-0.4, -0.6, -0.7))
        self.items_poly = []
        self.items_line = []
        self._build_static()

    # ---- 相机 ----
    def set_view(self, w, h):
        self.w, self.h = max(50, w), max(50, h)
        el = math.radians(self.cfg["view_el"])
        az = math.radians(self.cfg["view_az"])
        cam = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
        self.fwd = (-cam[0], -cam[1], -cam[2])
        up_world = (0.0, 0.0, -1.0)
        r = cross(self.fwd, up_world)
        if norm(r) < 1e-6:
            r = (1.0, 0.0, 0.0)
        self.right = unit(r)
        self.up = unit(cross(self.right, self.fwd))
        self.cx = self.w * 0.5
        self.cy = self.h * 0.56
        self.scale = min(self.w, self.h * 1.30) / (2.45 * self.al)

    def project(self, p):
        u = dot(p, self.right)
        v = dot(p, self.up)
        d = dot(p, self.fwd)
        return (self.cx + self.scale * u, self.cy - self.scale * v, d)

    # ---- 静态元素：天空、圆台、罗盘、字母 ----
    def _build_static(self):
        c = self.c
        c.create_rectangle(0, 0, 4000, 3000, fill="#cbd9f2", width=0)
        self.disc = c.create_polygon(0, 0, 0, 0, fill="#e2eede", outline="#9fb79a", width=2)
        self.ring2 = c.create_polygon(0, 0, 0, 0, fill="", outline="#c3d3bd")
        self.ring1 = c.create_polygon(0, 0, 0, 0, fill="", outline="#c3d3bd")
        self.ticks = [c.create_line(0, 0, 0, 0, fill="#8c9f88") for _ in range(12)]
        self.labels = [c.create_text(0, 0, text=t, fill="#39603a", font=("Microsoft YaHei", 11, "bold"))
                       for t in ("N", "E", "S", "W")]
        self.needle = c.create_line(0, 0, 0, 0, fill="#cc2626", width=3)
        self.triad = [c.create_line(0, 0, 0, 0, fill=col, width=3) for col in ("#d22", "#1b1", "#22d")]
        self.triad_txt = [c.create_text(0, 0, text=t, fill=col, font=("Arial", 11, "bold"))
                          for t, col in (("X", "#d22"), ("Y", "#1b1"), ("Z", "#22d"))]
        self.att_txt = c.create_text(10, 10, anchor="nw", fill="#101418",
                                     font=("Microsoft YaHei", 11, "bold"), text="")
        nfaces = sum(len(p[1]) for p in self.parts)
        self.shadow = [c.create_polygon(0, 0, 0, 0, fill="#b3c4af", outline="")
                       for _ in range(nfaces)]
        self.faces = [c.create_polygon(0, 0, 0, 0, fill="#dddddd", outline="#33383f", width=1)
                      for _ in range(nfaces)]

    def _circle(self, r, z, n=72):
        pts = []
        for k in range(n):
            a = 2 * math.pi * k / n
            x, y, _ = self.project((r * math.cos(a), r * math.sin(a), z))
            pts.extend((x, y))
        return pts

    def resize(self, w, h):
        self.set_view(w, h)
        self.c.coords(self.disc, *self._circle(self.rp, self.zg))
        self.c.coords(self.ring1, *self._circle(self.rp * 0.80, self.zg - 0.01))
        self.c.coords(self.ring2, *self._circle(self.rp * 0.40, self.zg - 0.01))
        for k in range(12):
            a = math.radians(k * 30)
            r1, r2 = self.rp * 0.93, self.rp * 0.99
            p1 = self.project((r1 * math.cos(a), r1 * math.sin(a), self.zg - 0.01))
            p2 = self.project((r2 * math.cos(a), r2 * math.sin(a), self.zg - 0.01))
            self.c.coords(self.ticks[k], p1[0], p1[1], p2[0], p2[1])
        for k, a_deg in enumerate((0, 90, 180, 270)):
            a = math.radians(a_deg)
            p = self.project((self.rp * 0.88 * math.cos(a), self.rp * 0.88 * math.sin(a), self.zg - 0.01))
            self.c.coords(self.labels[k], p[0], p[1])

    # ---- 每帧更新 ----
    def update(self, R, roll, pitch, yaw, text):
        c = self.c
        rot = [[R[i][j] for j in range(3)] for i in range(3)]
        polys = []
        fi = 0
        si = 0
        for verts, faces, color in self.parts:
            nav = [rotate(rot, v) for v in verts]
            # 影子（投影到台面）
            if si < len(self.shadow):
                sv = [self.project((p[0] + 0.30, p[1] + 0.22, self.zg - 0.03)) for p in nav]
                for f in faces:
                    pts = []
                    for idx in f:
                        pts.extend((sv[idx][0], sv[idx][1]))
                    c.coords(self.shadow[si], *pts)
                    c.tag_raise(self.shadow[si])
                    si += 1
            for f in faces:
                pts = []
                depth = 0.0
                for idx in f:
                    pr = self.project(nav[idx])
                    pts.extend((pr[0], pr[1]))
                    depth += pr[2]
                depth /= len(f)
                nrm = unit(cross(sub(nav[f[1]], nav[f[0]]), sub(nav[f[2]], nav[f[0]])))
                shade = 0.55 + 0.45 * abs(dot(nrm, self.light))
                col = "#%02x%02x%02x" % tuple(min(255, int(ch * shade)) for ch in color)
                polys.append((depth, fi, pts, col))
                fi += 1
        polys.sort(key=lambda x: -x[0])
        for depth, idx, pts, col in polys:
            it = self.faces[idx]
            c.itemconfigure(it, fill=col)
            c.coords(it, *pts)
            c.tag_raise(it)
        # 三轴
        L = 1.55
        for k, base in enumerate(((1, 0, 0), (0, 1, 0), (0, 0, 1))):
            ax = nav_axis(rot, k)
            e = (ax[0] * L, ax[1] * L, ax[2] * L)
            p0 = self.project((0, 0, 0))
            p1 = self.project(e)
            c.coords(self.triad[k], p0[0], p0[1], p1[0], p1[1])
            c.coords(self.triad_txt[k], p1[0] + 8, p1[1] - 8)
        # 航向指针
        hx = rot[0][0] * self.rc
        hy = rot[1][0] * self.rc
        a = self.project((0, 0, self.zg - 0.02))
        b = self.project((hx, hy, self.zg - 0.02))
        c.coords(self.needle, a[0], a[1], b[0], b[1])
        c.coords(self.att_txt, 10, 10)
        c.itemconfigure(self.att_txt, text=text)


def nav_axis(R, k):
    return (R[0][k], R[1][k], R[2][k])


# ---------------------------------------------------------------- 曲线面板
class PlotPanel(object):
    def __init__(self, canvas, title, ylabel, series, ylim=None, colors=("#cc2222", "#1a9a1a", "#2244cc"),
                 dashed=(), min_span=0.0):
        self.c = canvas
        self.title = title
        self.ylabel = ylabel
        self.series = series          # 名称列表
        self.colors = colors
        self.dashed = dashed
        self.ylim = ylim
        self.min_span = min_span
        self.w = self.h = 100
        self.cols = list(colors) + ["#b03a2e", "#196f3d"]   # 参考线用更深的红/绿，才看得清
        self.dash = dashed
        self.pool = {}          # (系列, 段) -> 线对象；NaN 处断线，避免长直线/竖条
        self.cur = None         # 当前 Y 量程（自适应 + 滞回）
        self.n_series = 5

    def _item(self, k, si):
        it = self.pool.get((k, si))
        if it is None:
            col = self.cols[k % len(self.cols)]
            is_ref = k in self.dash
            it = self.c.create_line(0, 0, 0, 0, fill=col,
                                    width=2 if is_ref else 1,
                                    dash=(7, 4) if is_ref else None)
            self.pool[(k, si)] = it
        return it

    def resize(self, w, h):
        self.w, self.h = max(30, w), max(30, h)

    # ---- 轴刻度：1/2/5 取整 + 量程自适应（带滞回，避免抖动） ----
    @staticmethod
    def _nice_step(span, target=4):
        if span <= 0:
            return 1.0
        raw = span / float(max(1, target))
        mag = 10.0 ** math.floor(math.log10(raw))
        for m in (1.0, 2.0, 5.0, 10.0):
            if raw <= m * mag:
                return m * mag
        return 10.0 * mag

    @staticmethod
    def _fmt(v, step):
        if step >= 1 and abs(step - round(step)) < 1e-9:
            return "%.0f" % v
        if step >= 0.1:
            return "%.1f" % v
        if step >= 0.01:
            return "%.2f" % v
        if step >= 0.001:
            return "%.3f" % v
        return "%.2g" % v

    def _y_range(self, ys):
        if self.ylim:
            lo, hi = self.ylim
            return lo, hi, self._nice_step(hi - lo, 4)
        vals = [v for y in ys for v in y if v == v]
        if not vals:
            return self.cur or (0.0, 1.0, 0.5)
        dlo, dhi = min(vals), max(vals)
        if dhi - dlo < self.min_span:        # 数据几乎不变时用最小量程，别放大到噪声
            mid = 0.5 * (dhi + dlo)
            half = 0.5 * max(self.min_span, 0.02)
            dlo, dhi = mid - half, mid + half
        m = 0.08 * (dhi - dlo)
        lo, hi = dlo - m, dhi + m            # 量程贴着数据（不强行取整到步长倍数）
        step = self._nice_step(hi - lo, 5)
        for _ in range(3):                   # 保证视野内至少 4 条横线
            if (hi - lo) / step >= 3.5:
                break
            step *= 0.5
        if (hi - lo) / step > 10:            # 线太密就加倍
            step *= 2.0
        cur = self.cur                      # 滞回：旧量程还能装下、且不算太富余就沿用
        if cur is not None:
            clo, chi, cstep = cur
            if clo <= dlo and dhi <= chi and (dhi - dlo) >= 0.5 * (chi - clo):
                return cur
        self.cur = (lo, hi, step)
        return lo, hi, step

    def update(self, t, ys):
        c = self.c
        if not t:
            return
        L, R, T, B = 60, 10, 24, 22
        pw, ph = self.w - L - R, self.h - T - B
        if pw < 10 or ph < 10:
            return
        t0, t1 = t[0], max(t[0] + 1e-3, t[-1])
        lo, hi, ystep = self._y_range(ys)
        sx = pw / (t1 - t0)
        sy = ph / (hi - lo)
        # 画顺序：先参考虚线（在底层），再实线（压在上面）
        order = [k for k in range(len(ys)) if k in self.dash] + \
                [k for k in range(len(ys)) if k not in self.dash]
        for k in order:
            y = ys[k]
            segs, cur = [], []
            for tt, vv in zip(t, y):
                if vv != vv:
                    if len(cur) >= 4:
                        segs.append(cur)
                    cur = []
                else:
                    cur.extend((L + (tt - t0) * sx, T + ph - (vv - lo) * sy))
            if len(cur) >= 4:
                segs.append(cur)
            for si, pts in enumerate(segs[:16]):
                it = self._item(k, si)
                c.coords(it, *pts)
                c.itemconfigure(it, state="normal")
                c.tag_raise(it)
            for si in range(len(segs), 16):
                it = self.pool.get((k, si))
                if it is not None:
                    c.itemconfigure(it, state="hidden")
        # 网格与刻度（用 tag 统一擦除/置底）
        c.delete("dyn")
        k = int(math.floor(lo / ystep))
        while k * ystep <= hi + 1e-9:
            v = k * ystep
            yy = T + ph - (v - lo) * sy
            c.create_line(L, yy, L + pw, yy, fill="#eceff3", tags="dyn")
            c.create_text(L - 6, yy, text=self._fmt(v, ystep), anchor="e", fill="#5a6470",
                          font=("Consolas", 8), tags="dyn")
            k += 1
        # 时间轴刻度：从 0.5 s 到 5 min 自适应
        xstep = self._nice_step(t1 - t0, 5)
        for cand in (0.5, 1, 2, 5, 10, 15, 30, 60, 120, 300):
            if cand >= xstep:
                xstep = cand
                break
        k0 = int(math.ceil(t0 / xstep))
        while k0 * xstep <= t1 + 1e-9:
            v = k0 * xstep
            xx = L + (v - t0) * sx
            c.create_line(xx, T, xx, T + ph, fill="#f4f6f8", tags="dyn")
            c.create_text(xx, T + ph + 4, text=("%.0f s" % v), anchor="n", fill="#5a6470",
                          font=("Consolas", 8), tags="dyn")
            k0 += 1
        c.create_text(L, 8, text="%s (%s)" % (self.title, self.ylabel), anchor="nw",
                      fill="#1d2126", font=("Microsoft YaHei", 10, "bold"), tags="dyn")
        c.tag_lower("dyn")


# ---------------------------------------------------------------- 主界面
class App(object):
    def __init__(self, root, cfg, replay=None, autostart=False, models=None, formats=None):
        import tkinter as tk
        from tkinter import ttk
        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.cfg = cfg
        self.replay = replay
        self.models = models or load_imu_models()
        self.formats = formats or FormatRegistry()
        self.sess = Session(cfg, self.formats)
        self.worker = None
        root.title("RAWIMU 实时姿态 (Python)")
        root.geometry("1480x860")
        root.configure(bg="#f2f4f6")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ---- 顶部控制条 ----
        bar = tk.Frame(root, bg="#f2f4f6")
        bar.pack(side="top", fill="x", padx=6, pady=4)
        tk.Label(bar, text="端口", bg="#f2f4f6").pack(side="left")
        self.port_var = tk.StringVar(value=cfg["port"])
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=10)
        self.port_box.pack(side="left", padx=(2, 8))
        self.refresh_ports()
        tk.Label(bar, text="波特率", bg="#f2f4f6").pack(side="left")
        self.baud_var = tk.StringVar(value=str(cfg["baud"]))
        ttk.Combobox(bar, textvariable=self.baud_var, width=8,
                     values=("921600", "460800", "230400", "115200")).pack(side="left", padx=(2, 8))
        self.btn_open = ttk.Button(bar, text="打开", command=self.toggle_port)
        self.btn_open.pack(side="left", padx=2)
        # IMU 型号（来自 imu_models.json + 内置表，切换即生效）
        tk.Label(bar, text="IMU", bg="#f2f4f6").pack(side="left", padx=(10, 0))
        self.imu_var = tk.StringVar(value=cfg.get("imu_model", ""))
        self.imu_box = ttk.Combobox(bar, textvariable=self.imu_var, width=12,
                                    values=sorted(self.models.keys()), state="readonly")
        self.imu_box.pack(side="left", padx=(2, 8))
        self.imu_box.bind("<<ComboboxSelected>>", lambda e: self.on_imu())
        self.mode_var = tk.StringVar(value=cfg["mode"])
        ttk.Radiobutton(bar, text="陀螺+重力修正", value="complementary",
                        variable=self.mode_var, command=self.on_mode).pack(side="left", padx=(12, 0))
        ttk.Radiobutton(bar, text="纯陀螺积分", value="gyro",
                        variable=self.mode_var, command=self.on_mode).pack(side="left")
        ttk.Button(bar, text="姿态归零", command=self.on_reset).pack(side="left", padx=(12, 2))
        ttk.Button(bar, text="重新标定", command=self.on_recal).pack(side="left", padx=2)
        self.btn_rec = ttk.Button(bar, text="开始记录", command=self.toggle_record)
        self.btn_rec.pack(side="left", padx=(12, 2))

        # ---- 主体三列 ----
        body = tk.Frame(root, bg="#f2f4f6")
        body.pack(side="top", fill="both", expand=True, padx=6, pady=(0, 6))
        body.columnconfigure(0, weight=31, uniform="col")
        body.columnconfigure(1, weight=34, uniform="col")
        body.columnconfigure(2, weight=35, uniform="col")
        body.rowconfigure(0, weight=1, uniform="row")
        body.rowconfigure(1, weight=1, uniform="row")

        self.cv3d = tk.Canvas(body, bg="#cbd9f2", highlightthickness=1,
                              highlightbackground="#c8ccd2")
        self.cv3d.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=3, pady=3)
        self.scene = Scene3D(self.cv3d, cfg)

        mk = lambda t, y, s, **kw: tk.Canvas(body, bg="white", highlightthickness=1,
                                             highlightbackground="#dde1e6")
        self.cv_eul = mk("", "", None)
        self.cv_gyr = mk("", "", None)
        self.cv_acc = mk("", "", None)
        self.cv_eul.grid(row=0, column=1, sticky="nsew", padx=3, pady=3)
        self.cv_gyr.grid(row=0, column=2, sticky="nsew", padx=3, pady=3)
        self.cv_acc.grid(row=1, column=1, sticky="nsew", padx=3, pady=3)
        self.p_eul = PlotPanel(self.cv_eul, "姿态角（实线=陀螺积分 虚线=重力反算）", "deg", [],
                               dashed=(3, 4), min_span=2.0)
        self.p_gyr = PlotPanel(self.cv_gyr, "陀螺角速度（已去零偏） 红X 绿Y 蓝Z", "deg/s", [], min_span=0.1)
        self.p_acc = PlotPanel(self.cv_acc, "加速度 红X 绿Y 蓝Z", "m/s2", [], min_span=2.0)

        self.cv_sta = tk.Canvas(body, bg="white", highlightthickness=1,
                                highlightbackground="#dde1e6")
        self.cv_sta.grid(row=1, column=2, sticky="nsew", padx=3, pady=3)
        self.sta_text = self.cv_sta.create_text(12, 10, anchor="nw", fill="#1d2126",
                                                font=("Consolas", 10), text="")

        self.cv3d.bind("<Configure>", self.on_3d_resize)
        for cv, pnl in ((self.cv_eul, self.p_eul), (self.cv_gyr, self.p_gyr), (self.cv_acc, self.p_acc)):
            cv.bind("<Configure>", (lambda e, p=pnl, c=cv: (p.resize(e.width, e.height), p.update([], []))))

        root.bind("<Escape>", lambda e: self.on_close())
        root.bind("<space>", lambda e: self.on_reset())
        self.last_ui = 0.0
        root.after(int(1000.0 / max(1, cfg["refresh_hz"])), self.tick)
        if autostart:
            if replay:
                self.start_replay(replay)
            else:
                self.toggle_port()

    # ---- 控制 ----
    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        self.port_box["values"] = ports or ("COM1",)
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_port(self):
        if self.worker and self.worker.is_alive():
            self.sess.running = False
            self.btn_open.configure(text="打开")
            return
        self.cfg["port"] = self.port_var.get()
        try:
            self.cfg["baud"] = int(self.baud_var.get())
        except ValueError:
            self.cfg["baud"] = 460800
        self.worker = SerialWorker(self.sess)
        self.worker.start()
        self.btn_open.configure(text="关闭")

    def start_replay(self, path, speed=None):
        # 录制的文件没有“上电静止段”，自动零偏会把转动当成零偏
        self.cfg["auto_bias"] = False
        self.cfg["auto_accel_scale"] = False
        self.worker = ReplayWorker(self.sess, path, speed or self.cfg["replay_speed"])
        self.worker.start()
        self.btn_open.configure(text="关闭")

    def on_imu(self):
        """切换 IMU 型号：改量纲系数并重新标定（零偏/加计标度）"""
        name = self.imu_var.get()
        if apply_model(self.cfg, self.models, name):
            self.sess.att.reset_calibration()
            self.sess.status = "IMU 型号已切换: %s" % name

    def on_mode(self):
        self.cfg["mode"] = self.mode_var.get()

    def on_reset(self):
        self.sess.att.reset_attitude()

    def on_recal(self):
        self.sess.att.reset_calibration()

    def toggle_record(self):
        if self.sess.recording:
            self.sess.stop_record()
            self.btn_rec.configure(text="开始记录")
        else:
            name = time.strftime("rawimu_%Y%m%d_%H%M%S.csv")
            self.sess.start_record(name)
            self.btn_rec.configure(text="停止记录")

    def on_3d_resize(self, e):
        self.scene.resize(e.width, e.height)

    def on_close(self):
        self.sess.running = False
        self.sess.stop_record()
        try:
            self.root.destroy()
        except Exception:
            pass

    # ---- 刷新 ----
    def tick(self):
        try:
            self.refresh()
        except Exception:
            traceback.print_exc()
        self.root.after(int(1000.0 / max(1, self.cfg["refresh_hz"])), self.tick)

    def refresh(self):
        sess = self.sess
        t, ch, rollA, pitchA, att, smp = sess.snapshot()
        if sess.error:
            self.sta_text and self.cv_sta.itemconfigure(
                self.sta_text, text="错误: " + sess.error)
        if not t:
            return
        win = self.cfg["window_s"]
        tmax = t[-1]
        tmin = tmax - win
        idx = 0
        while idx < len(t) and t[idx] < tmin:
            idx += 1
        tt = t[idx:]
        c = [ch[k][idx:] for k in range(9)]
        ra = rollA[idx:]
        pa = pitchA[idx:]

        R = quat_to_dcm(att.q)
        roll, pitch, yaw = (math.degrees(v) for v in (att.roll, att.pitch, att.yaw))
        self.scene.update(R, roll, pitch, yaw,
                          u"姿态（X前/Y右/Z下）\nroll %+7.2f°\npitch %+7.2f°\nyaw %+7.2f°" % (roll, pitch, yaw))
        self.p_eul.update(tt, [c[0], c[1], c[2], ra, pa])
        self.p_gyr.update(tt, [c[3], c[4], c[5]])
        self.p_acc.update(tt, [c[6], c[7], c[8]])

        a_mean = sess.acc_norm_sum / max(1, sess.acc_norm_n)
        dt_mean = sess.dt_sum / max(1, sess.dt_n)
        mode = "gyro+accel(tau=%.1fs)" % self.cfg["level_tau"] if (
            self.cfg["mode"] == "complementary" and self.cfg["accel_leveling"]) else "gyro only"
        lines = [
            "%-10s: %8.1f Hz" % ("Rate", sess.rate_hz),
            "%-10s: %8.2f ms" % ("Interval", dt_mean * 1000),
            "%-10s: %8.2f C" % ("Temp", smp["temp"] if smp else float("nan")),
            "%-10s: %8.3f m/s2" % ("|a|", a_mean),
            "%-10s: 0x%04X" % ("Status", smp["status"] if smp else 0),
            "%-10s: 0x%02X" % ("StatByte", smp["stat_byte"] if smp else 0),
            "%-10s: %8d" % ("Counter", smp["counter"] if smp else 0),
            "%-10s: %.3f %.3f %.3f" % ("Bias dps", att.bias[0], att.bias[1], att.bias[2]),
            "%-10s: %s" % ("Mode", mode),
            "%-10s: %5.1f %5.1f %5.1f" % ("IntAngle", att.ang_acc[0], att.ang_acc[1], att.ang_acc[2]),
            "",
            "%-10s: %s" % ("Format", fmt_text(sess.parser.fmt_counts)),
            "%-10s: %8d" % ("RAWIMU", sess.n_imu),
            "%-10s: %8d" % ("Frames", sess.parser.n_frame),
            "%-10s: %8d" % ("ShortHdr", sess.parser.n_short),
            "%-10s: %8d" % ("CRC err", sess.parser.n_crc),
            "%-10s: %8d" % ("SkipBytes", sess.parser.n_drop),
            "%-10s: %8.1f kB" % ("Bytes", sess.parser.n_bytes / 1024.0),
            "",
            "%-10s: %s" % ("Source", sess.status[:28]),
        ]
        self.cv_sta.itemconfigure(self.sta_text, text="\n".join(lines))


# ---------------------------------------------------------------- 窗口截图
def grab_window(title, out_path, hwnd=None):
    """把窗口自身渲染成图片（PrintWindow，锁屏/遮挡也能取到）"""
    import ctypes
    from ctypes import wintypes
    user32 = ctypes.windll.user32
    gdi32 = ctypes.windll.gdi32
    hwnd = user32.FindWindowW(None, title)
    if not hwnd:
        return False

    class BIH(ctypes.Structure):
        _fields_ = [("biSize", wintypes.DWORD), ("biWidth", ctypes.c_long),
                    ("biHeight", ctypes.c_long), ("biPlanes", wintypes.WORD),
                    ("biBitCount", wintypes.WORD), ("biCompression", wintypes.DWORD),
                    ("biSizeImage", wintypes.DWORD), ("biXPelsPerMeter", ctypes.c_long),
                    ("biYPelsPerMeter", ctypes.c_long), ("biClrUsed", wintypes.DWORD),
                    ("biClrImportant", wintypes.DWORD)]

    r = wintypes.RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(r))
    w, h = r.right - r.left, r.bottom - r.top
    if w <= 0 or h <= 0:
        return False
    hdc = user32.GetWindowDC(hwnd)
    mdc = gdi32.CreateCompatibleDC(hdc)
    bmp = gdi32.CreateCompatibleBitmap(hdc, w, h)
    gdi32.SelectObject(mdc, bmp)
    user32.PrintWindow(hwnd, mdc, 2)
    bi = BIH()
    bi.biSize = ctypes.sizeof(BIH)
    bi.biWidth, bi.biHeight = w, -h
    bi.biPlanes, bi.biBitCount, bi.biCompression = 1, 32, 0
    buf = ctypes.create_string_buffer(w * h * 4)
    gdi32.GetDIBits(mdc, bmp, 0, h, buf, ctypes.byref(bi), 0)
    ok = False
    try:
        from PIL import Image
        Image.frombuffer("RGBA", (w, h), buf, "raw", "BGRA", 0, 1).convert("RGB").save(out_path)
        ok = True
    except Exception:
        try:
            import struct as _st
            with open(out_path, "wb") as f:
                row = w * 4
                f.write(b"BM" + _st.pack("<IHHI", 14 + 40 + row * h, 0, 0, 54))
                f.write(_st.pack("<IiiHHIIiiII", 40, w, -h, 1, 32, 0, row * h, 2835, 2835, 0, 0))
                f.write(buf.raw)
            ok = True
        except Exception:
            ok = False
    gdi32.DeleteObject(bmp)
    gdi32.DeleteDC(mdc)
    user32.ReleaseDC(hwnd, hdc)
    return ok

# ---------------------------------------------------------------- 自检 / 无界面
def run_selftest(cfg, path, truth=None, dump_unknown=False):
    # 录制/回放通常没有“开头静止段”，关掉自动标定，否则会把转动当成零偏
    cfg["auto_bias"] = False
    cfg["auto_accel_scale"] = False
    sess = Session(cfg)
    data = open(path, "rb").read()
    t0 = time.time()
    step = 8192
    for i in range(0, len(data), step):
        sess.parser.feed(data[i:i + step])
    dt = time.time() - t0
    t, ch, rollA, pitchA, att, smp = sess.snapshot()
    print("帧 %d  IMU %d  其它 %d  短头 %d  CRC错 %d  丢弃字节 %d"
          % (sess.parser.n_frame, sess.n_imu, sess.parser.n_other, sess.parser.n_short,
             sess.parser.n_crc, sess.parser.n_drop))
    print("报文格式统计: %s" % fmt_text(sess.parser.fmt_counts))
    if dump_unknown and sess.parser.unknown:
        print("未识别报文样本（照着填 formats.json）：")
        for mid, hlen, ln, hx in sess.parser.unknown:
            print("  id=0x%04X(%d) 头长=%d 体长=%d 前56字节=%s" % (mid, mid, hlen, ln, hx))
    elif sess.parser.unknown:
        print("未识别 ID: %s" % ", ".join("0x%04X" % u[0] for u in sess.parser.unknown))
    print("解析耗时 %.2f s（%.1f MB/s）" % (dt, len(data) / 1048576.0 / max(1e-6, dt)))
    if t:
        print("时长 %.1f s  平均帧间隔 %.3f ms  温度 %.2f C  平均|a| %.3f m/s2"
              % (t[-1] - t[0], 1000 * sess.dt_sum / max(1, sess.dt_n),
                 sess.temp, sess.acc_norm_sum / max(1, sess.acc_norm_n)))
        print("末姿态 roll %.2f pitch %.2f yaw %.2f" % (
            math.degrees(att.roll), math.degrees(att.pitch), math.degrees(att.yaw)))
        print("累计转角 %.1f %.1f %.1f  deg" % tuple(att.ang_acc))
    if truth and t:
        import csv
        rows = []
        with open(truth) as f:
            r = csv.reader(f)
            next(r, None)
            for line in r:
                if len(line) >= 4:
                    rows.append([float(x) for x in line[:4]])
        n = min(len(t) - 1, len(rows) - 1)
        err = [0.0, 0.0, 0.0]
        for k in range(n):
            for j in range(3):
                d = (ch[j][k] - rows[k + 1][j + 1] + 180.0) % 360.0 - 180.0
                err[j] += abs(d)
        err = [e / n for e in err]
        print("对比真值 meanerr roll=%.5f pitch=%.5f yaw=%.5f deg" % tuple(err))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="RAWIMU 实时姿态显示（支持自定义 IMU 型号与报文格式）")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int)
    ap.add_argument("--replay")
    ap.add_argument("--speed", type=float)
    ap.add_argument("--imu", default=None, help="IMU 型号名（见 --list-models）")
    ap.add_argument("--mode", choices=("complementary", "gyro"))
    ap.add_argument("--config")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--truth")
    ap.add_argument("--no-autostart", action="store_true")
    ap.add_argument("--snapshot", help="截图保存路径（调试/留证）")
    ap.add_argument("--snapshot-delay", type=float, default=3.0)
    ap.add_argument("--list-models", action="store_true", help="列出全部 IMU 型号")
    ap.add_argument("--list-formats", action="store_true", help="列出全部报文格式")
    ap.add_argument("--dump-unknown", action="store_true",
                    help="回放时打印未识别报文样本（用于逆向/填 formats.json）")
    ap.add_argument("--write-templates", action="store_true",
                    help="在当前目录生成 imu_models.json / formats.json 模板")
    a = ap.parse_args(argv)

    models = load_imu_models()
    formats = FormatRegistry()

    if a.write_templates:
        for fname, tmpl in (("imu_models.json", IMU_MODELS_TEMPLATE),
                            ("formats.json", FORMATS_TEMPLATE)):
            path = os.path.join(os.getcwd(), fname)
            if os.path.exists(path):
                print("已存在，未覆盖:", path)
            else:
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(tmpl, f, ensure_ascii=False, indent=2)
                print("已生成模板:", path)
        return 0

    if a.list_models:
        print("%-12s %-14s %-14s %-8s %s" % ("型号", "gyro_lsb", "accel_lsb", "freq", "备注"))
        for name in sorted(models):
            m = models[name]
            print("%-12s %-14.6g %-14.6g %-8.1f %s" % (
                name, m.get("gyro_lsb", 0), m.get("accel_lsb", 0), m.get("freq", 0),
                m.get("note", "")))
        return 0

    if a.list_formats:
        print("%-10s %-8s %-6s %-6s %s" % ("名称", "id(hex)", "头长", "体长", "备注"))
        for name, s in sorted(formats.specs.items()):
            print("%-10s 0x%04X   %-6d %-6d %s" % (
                name, int(s["id"]), s["header_len"], s["body_len"], s.get("note", "")))
        for e in formats.errors:
            print("  [错误]", e)
        return 0

    cfg = dict(DEFAULT_CFG)
    cfg.update(models.get(DEFAULT_CFG["imu_model"], {}))
    if a.config and os.path.isfile(a.config):
        with open(a.config) as f:
            cfg.update(json.load(f))
    if a.imu:
        if not apply_model(cfg, models, a.imu):
            print("未知 IMU 型号: %s（用 --list-models 查看，或写进 imu_models.json）" % a.imu)
            return 2
    if a.port:
        cfg["port"] = a.port
    if a.baud:
        cfg["baud"] = a.baud
    if a.mode:
        cfg["mode"] = a.mode
    if a.speed:
        cfg["replay_speed"] = a.speed

    if a.selftest:
        if not a.replay:
            print("需要 --replay 文件")
            return 2
        return run_selftest(cfg, a.replay, a.truth, a.dump_unknown)

    import tkinter as tk
    root = tk.Tk()
    app = App(root, cfg, replay=a.replay, autostart=not a.no_autostart,
              models=models, formats=formats)
    if a.snapshot:
        def _shot():
            ok = grab_window(root.title(), a.snapshot, int(root.wm_frame(), 16))
            print("snapshot", "ok" if ok else "failed", a.snapshot)
            root.after(200, root.destroy)
        root.after(int(a.snapshot_delay * 1000), _shot)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
