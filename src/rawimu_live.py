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
import copy
import json
import math
import os
import struct
import sys
import threading
import time
import traceback
import zlib
from collections import deque
from bisect import bisect_left, bisect_right
import weakref

# A private dependency directory is supported for existing Python 3.7 installations.
_SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
_PRIVATE_PACKAGES = os.path.join(os.path.dirname(_SOURCE_DIR), '.vendor')
for _package_path in (_SOURCE_DIR, _PRIVATE_PACKAGES):
    if os.path.isdir(_package_path) and _package_path not in sys.path:
        sys.path.insert(0, _package_path)

G = 9.80665
from plot_navigation import PlotNavigation, clip_polyline
from time_axis import TimeAxis, AXIS_MODES
from network_source import NetworkWorker, NETWORK_MODES, validate_endpoint
from command_transport import CommandChannel

# Project convention from ../rawimu_status.m: stat bit 0..7 = word bit 8..15.
IMU_STATUS_FAULT_NAMES = ("GyroX", "GyroY", "GyroZ", "AccX", "AccY", "AccZ", "Temp", "FUSA")


def imu_fault_names(word):
    return [name for bit, name in enumerate(IMU_STATUS_FAULT_NAMES) if int(word) & (1 << (bit + 8))]

# ---------------------------------------------------------------- 默认配置
DEFAULT_CFG = {
    "source": "serial",
    "network_host": "127.0.0.1",
    "network_port": 3001,
    "time_axis": "elapsed",
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
    "fast_align": True,          # 开局静止时直接用重力定 roll/pitch
    "tau_adaptive": True,        # 误差大时临时加快修正
    "window_s": 30.0,
    "refresh_hz": 20.0,
    "view_az": -125.0,
    "view_el": -26.0,
    "replay_speed": 1.0,
    "renderer": "auto",            # auto: textured OpenGL when available; canvas: compatibility mesh
}

# ---------------------------------------------------------------- IMU 型号表
# 加新型号：**不用改代码**，编辑程序目录下的 imu_models.json 即可（格式见该文件内说明）。
# 每个型号可覆盖的字段：gyro_lsb / accel_lsb / temp_lsb / temp_offset / freq /
#   invert_y / axis_map / axis_sign / note
IMU_MODELS = {
    "M1633": {"gyro_lsb": 1.0 / 65536.0, "accel_lsb": 0.5 / 65536.0,
              "temp_lsb": 0.01, "temp_offset": 0.0, "freq": 100.0,
              "note": "M1633，32 位定点 16 位小数（默认）"},
    # COM50 实机（M22）静止标定：|a| 计数 1304056 @ g=9.80665 -> 7.5201e-6 m/s^2/LSB
    "M22": {"gyro_lsb": 1.0 / 65536.0, "accel_lsb": 7.520111259e-06,
            "temp_lsb": 0.01, "temp_offset": 0.0, "freq": 100.0,
            "note": "M22（COM50 实机标定；陀螺 1/65536 deg/s，加计 7.5201e-6 m/s^2）"},
    # main_of_dpx -> nav_imu_data -> IMU_ID_M21/B240, delta increments at 100 Hz.
    "M21": {"gyro_lsb": 1.0 / 65536.0, "accel_lsb": 9.80665 / (20 * 65536.0),
            "temp_lsb": 1.0 / 256.0, "temp_offset": 25.0, "freq": 100.0,
            "note": "M21 / B240，与 MATLAB RAWIMU 换算一致；温度 raw/256+25"},
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
def crc32_raw(data):
    """设备用的 CRC32：反射多项式 0xEDB88320、初值 0、末尾不取反"""
    # zlib 内部使用前后取反；传入全 1 初值并撤销最终取反即为设备算法。
    return (zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF) & 0xFFFFFFFF


# ---------------------------------------------------------------- 帧解析
RAWIMU_ID = 0x010C
RAWIMUSX_ID = 0x05B6


class FrameParser(object):
    """字节流 -> 帧；按消息 ID 查格式注册表，未注册的只计数（可抓样本用于逆向）"""

    def __init__(self, on_frame, registry, on_gpimu=None):
        self.on_frame = on_frame
        self.on_gpimu = on_gpimu
        self.n_gpimu = self.n_gpimu_crc_bad = self.n_gpimu_malformed = 0
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
        self.n_text = 0            # ASCII 行数
        self.text_unknown = []     # 未识别 ASCII 前缀，最多 8 个
        self.n_text_crc_bad = 0
        self.frame_end = 0
        self.frame_header_time = None

    def feed(self, chunk):
        self.buf += chunk
        self.n_bytes += len(chunk)
        buf = self.buf
        n = len(buf)
        i = 0
        while i <= n - 16:
            b0 = buf[i]
            # ---- ASCII 日志行（#RAWIMUA / $GPGGA ...）----
            if b0 in (0x23, 0x24):                       # '#' / '$'
                nl = buf.find(b"\n", i)
                # Mixed streams may contain unframed binary diagnostics with '#'
                # or '$' bytes. Never consume a binary frame as an ASCII line.
                end = nl if nl >= 0 else min(n, i+8193)
                candidate = buf[i:end]
                if (len(candidate) > 8192 or
                        any(value not in (9, 13) and not 32 <= value <= 126 for value in candidate)):
                    i += 1
                    self.n_drop += 1
                    continue
                if nl < 0:
                    if n - i > 8192:                     # 超长行，防卡死
                        i += 1
                        self.n_drop += 1
                        continue
                    break
                line = bytes(buf[i:nl]).strip()
                i = nl + 1
                self.frame_end = self.n_bytes - n + i
                self._handle_line(line)
                continue
            if b0 != 0xAA or buf[i + 1] != 0x44:
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
                self.frame_header_time = ((struct.unpack_from('<H', buf, i+14)[0],
                                           struct.unpack_from('<I', buf, i+16)[0]/1000.) if hlen == 28 else None)
                name = spec["name"]
                self.fmt_counts[name] = self.fmt_counts.get(name, 0) + 1
                self.frame_end = self.n_bytes - n + i + total
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

    def _handle_line(self, line):
        """处理一条 ASCII 日志行"""
        self.n_text += 1
        if not line:
            return
        try:
            text = line.decode("ascii", "replace").strip()
        except Exception:
            return
        if not text or text[0] not in "#$":
            return
        if text.startswith('$GPIMU,'):
            from gpimu_data import decode_gpimu, GPIMUChecksumError
            try:
                raw = decode_gpimu(text)
            except GPIMUChecksumError:
                self.n_gpimu_crc_bad += 1
            except (ValueError, TypeError, OverflowError):
                self.n_gpimu_malformed += 1
            else:
                self.n_gpimu += 1
                if self.on_gpimu is not None:
                    self.on_gpimu(raw)
            return
        # 去掉末尾的 *crc（8 位十六进制）；CRC 覆盖的是去掉前导 '#' 之后的内容
        star = text.rfind("*")
        payload = text
        crc_rx = None
        if star > 0 and len(text) - star == 9:
            payload = text[:star]
            try:
                crc_rx = int(text[star + 1:], 16)
            except ValueError:
                crc_rx = None
        prefix = payload.split(",", 1)[0]
        spec = self.reg.by_prefix.get(prefix)
        if spec is not None and crc_rx is not None:
            try:
                if crc32_raw(payload[1:].encode("ascii", "ignore")) != crc_rx:
                    self.n_text_crc_bad += 1
                    return
            except Exception:
                pass
        if spec is None:
            if len(self.text_unknown) < 8 and prefix not in self.text_unknown:
                self.text_unknown.append(prefix)
            return
        self.fmt_counts[spec["name"]] = self.fmt_counts.get(spec["name"], 0) + 1
        try:
            self.on_frame(spec, payload)
        except Exception:
            pass


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


def build_ascii_decoder(spec):
    """ASCII 日志（如 #RAWIMUA,COM3,...;week,sec,status,az,ay,ax,gz,gy,gx*crc）

    字段用 "idx" 指定在“分号之后”数据体里的下标；status 按 16 进制解析。
    """
    def decode(text):
        body = text.split(";", 1)[1] if ";" in text else ""
        vals = [s.strip() for s in body.split(",")]
        d = {"name": spec.get("name", "?")}
        try:
            d["week"] = int(vals[spec["week"]["idx"]]) if spec.get("week") else 0
            d["sow"] = float(vals[spec["sow"]["idx"]])
            d["status"] = int(vals[spec["status"]["idx"]], 16) if spec.get("status") else 0
        except (IndexError, ValueError):
            raise ValueError("ASCII 字段不足或被截断")
        for key in ("acc", "gyr"):
            f = spec.get(key)
            if f:
                idx = f["idx"]
                raw = [float(vals[i]) for i in idx]
                d[key] = _order_xyz(raw, f.get("order", ["x", "y", "z"]))
        for key in ("imu_info", "imu_type"):
            d[key] = None
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
    # ---- ASCII 日志（# 开头、分号后是数据体）----
    "RAWIMUA": {
        "name": "RAWIMUA", "kind": "ascii", "prefix": "#RAWIMUA",
        "week": {"idx": 0}, "sow": {"idx": 1}, "status": {"idx": 2},
        "acc": {"idx": [3, 4, 5], "order": ["z", "y", "x"]},
        "gyr": {"idx": [6, 7, 8], "order": ["z", "y", "x"]},
        "note": "ASCII RAWIMUA：week,sec,status,az,ay,ax,gz,gy,gx（分号后）",
    },
    "RAWIMUSB": {
        "name": "RAWIMUSB", "kind": "ascii", "prefix": "#RAWIMUSB",
        "week": {"idx": 0}, "sow": {"idx": 1}, "status": {"idx": 2},
        "acc": {"idx": [3, 4, 5], "order": ["z", "y", "x"]},
        "gyr": {"idx": [6, 7, 8], "order": ["z", "y", "x"]},
        "note": "ASCII RAWIMUSB：字段顺序同 RAWIMUA（按手册核对）",
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
    """报文格式注册表：内置（二进制 + ASCII）+ formats.json 里自定义的"""

    def __init__(self, specs=None, json_paths=None):
        self.specs = {}
        self.by_id = {}
        self.by_prefix = {}
        self.errors = []
        for s in (specs or FORMAT_SPECS).values():
            self.add(s)
        for p in (json_paths if json_paths is not None else _json_candidates("formats.json")):
            self.load_json(p)

    def add(self, spec):
        spec = dict(spec)
        kind = spec.get("kind", "binary")
        try:
            if kind == "ascii":
                if "prefix" not in spec or "sow" not in spec:
                    raise ValueError("ASCII 格式需要 prefix/sow")
                spec["decode"] = build_ascii_decoder(spec)
                spec.setdefault("name", spec["prefix"])
                self.by_prefix[spec["prefix"]] = spec
            else:
                if "id" not in spec or "sow" not in spec:
                    raise ValueError("二进制格式需要 id/sow")
                spec.setdefault("header_len", 28)
                spec.setdefault("body_len", 40)
                spec.setdefault("name", "ID_%04X" % (int(spec["id"]) & 0xFFFF))
                spec["decode"] = build_decoder(spec)
                self.by_id[int(spec["id"]) & 0xFFFF] = spec
        except Exception as e:
            self.errors.append("%s 字段表有误: %s" % (spec.get("name") or spec.get("prefix"), e))
            return False
        self.specs[spec["name"]] = spec
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
        self.align_done = False      # 快速初始对准是否完成
        self.align_tried = False     # 放弃对准（一直没静止）
        self.align_acc = [0.0, 0.0, 0.0]
        self.align_n = 0
        self.align_t = 0.0
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
        self.roll = self.pitch = self.yaw = 0.0

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

        # 0) 快速初始对准：静止时直接用重力定 roll/pitch，省掉开机几秒的慢慢收敛
        if cfg["fast_align"] and not self.align_done and not self.align_tried:
            an0 = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
            wn0 = math.degrees(math.sqrt(sum(v * v for v in w)))
            if an0 > 0.1 and abs(an0 - G) < 0.6 and wn0 < 8.0:
                for k in range(3):
                    self.align_acc[k] += a[k]
                self.align_n += 1
                need = max(5, int(round(0.30 * cfg["freq"])))     # 0.3 s 平均
                if self.align_n >= need:
                    m = [v / self.align_n for v in self.align_acc]
                    ra = math.atan2(m[1], m[2])
                    pa = math.atan2(-m[0], math.hypot(m[1], m[2]))
                    self.q = euler_to_quat(ra, pa, 0.0)
                    self.align_done = True
                    self.log.append("快速对准 roll %.2f pitch %.2f" %
                                    (math.degrees(ra), math.degrees(pa)))
            else:
                self.align_n = 0
                self.align_acc = [0.0, 0.0, 0.0]
                self.align_t += dt
                if self.align_t > 5.0:          # 一直没静止，放弃对准，退回慢收敛
                    self.align_tried = True

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
                tau = cfg["level_tau"]
                if cfg["tau_adaptive"]:
                    # 误差大 → 临时加快（大角度摆动时跟得紧），误差小 → 慢（平滑噪声）
                    err_deg = math.degrees(math.asin(min(1.0, ang)))
                    if err_deg > 5.0:
                        tau *= 0.2
                    elif err_deg > 1.0:
                        tau *= 0.5
                kk = (dt / max(0.05, tau)) * ang
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
        self.lock = threading.RLock()
        from performance_data import SampleArchive
        self.analysis_archive = SampleArchive()
        self.analysis_cfg = copy.deepcopy(cfg)
        self.analysis_revision = 0
        self.replay_elapsed = None
        self.total_imu_received = self.total_gpimu_received = 0
        self.clear_generation = 0
        self._plot_origin = 0.
        self._gp_plot_origin = 0.
        self._error_baseline = {}
        # 60s visible window + 5s moving-average context + boundary samples.
        n = max(6000, int(math.ceil(cfg["freq"] * 65)) + 2)
        from gpimu_data import GPIMUStore
        self.gpimu = GPIMUStore(n)
        self.raw_comparison = deque(maxlen=n)
        self.gpimu_replay_only = False
        self.t = deque(maxlen=n)
        self.ch = [deque(maxlen=n) for _ in range(9)]   # roll,pitch,yaw,wx,wy,wz,ax,ay,az
        self.physical_channels = [deque(maxlen=n) for _ in range(6)]
        self.rollA = deque(maxlen=n)
        self.pitchA = deque(maxlen=n)
        self.imu_status = deque(maxlen=n)
        self.temperature = deque(maxlen=n)
        self.weeks = deque(maxlen=n)
        self.seconds = deque(maxlen=n)
        self.time_intervals = deque(maxlen=n)
        self.header_intervals = deque(maxlen=n)
        self._previous_header = None
        self.counter_intervals = deque(maxlen=n)
        self.att = Attitude(cfg)
        self.formats = formats if formats is not None else FormatRegistry()
        self.parser = FrameParser(self._on_frame, self.formats, self._on_gpimu)
        self.running = False
        self.error = ""
        self.record_error = ""
        self.state = "idle"
        self.last_sample_wall = None
        self.status = ""
        self.week = None
        self.prev_t = None
        self.t0 = None
        self.t_last = 0.0
        self.n_imu = 0
        self.dt_sum = 0.0
        self.dt_n = 0
        self.interval_anomalies = 0
        self.last_interval_ms = None
        self.rate_hz = 0.0
        self.temp = float("nan")
        self.acc_norm_sum = 0.0
        self.acc_norm_n = 0
        self.n_acc_spike = 0
        self.counter_prev = None
        self.counter_steps = {}
        self.stat_bytes = set()
        self.smp = None
        self.csv_file = None
        self.recording = False
        self.csv_name = ""
        self.raw_lock = threading.RLock()
        self.raw_file = None
        self.raw_recording = False
        self.raw_name = ""
        self.raw_bytes = 0
        self.raw_error = ""

    def feed(self, chunk):
        """Tap the exact received bytes before framing, filtering or CRC validation."""
        with self.raw_lock:
            if chunk and self.raw_recording and self.raw_file is not None:
                try:
                    written = self.raw_file.write(chunk)
                    if written is not None:
                        self.raw_bytes += written
                    if written != len(chunk):
                        raise OSError("原始数据写入不完整")
                except (OSError, ValueError) as exc:
                    self.raw_error = "DAT 记录失败: %s" % exc
                    self.stop_raw_record()
        self.parser.feed(chunk)

    def start_raw_record(self, path):
        with self.raw_lock:
            self.stop_raw_record()
            self.raw_error = ""
            try:
                handle = open(path, "wb")
            except (OSError, ValueError) as exc:
                self.raw_error = "无法开始 DAT 记录: %s" % exc
                return False
            self.raw_file = handle
            self.raw_name, self.raw_bytes = path, 0
            self.raw_recording = True
            return True

    def stop_raw_record(self):
        with self.raw_lock:
            self.raw_recording = False
            if self.raw_file is not None:
                try:
                    self.raw_file.close()
                except (OSError, ValueError) as exc:
                    self.raw_error = (self.raw_error + "；" if self.raw_error else "") + "DAT 保存失败: %s" % exc
                finally:
                    self.raw_file = None

    # 帧回调（解析线程里调用）
    def _on_frame(self, spec, body):
        with self.lock:
            self._process_frame(spec, body)

    def _on_gpimu(self, raw):
        with self.lock:
            self.total_gpimu_received += 1
            elapsed = self.replay_elapsed if self.gpimu_replay_only else None
            if elapsed is not None:
                if self._gp_plot_origin is None:
                    self._gp_plot_origin = elapsed
                elapsed -= self._gp_plot_origin
            self.gpimu.append(raw, self.cfg['freq'], elapsed)

    def gpimu_snapshot(self):
        import numpy as np
        with self.lock:
            return {'rows': self.gpimu.snapshot(), 'raw_rows': np.asarray(list(self.raw_comparison) if self.gpimu.total else [], dtype=float).reshape(-1, 7),
                    'total': self.gpimu.total, 'checksum_bad': self.parser.n_gpimu_crc_bad-self._error_baseline.get('n_gpimu_crc_bad',0),
                    'malformed': self.parser.n_gpimu_malformed-self._error_baseline.get('n_gpimu_malformed',0), 'source': '当前实时连接 / 回放',
                    'raw_model': self.analysis_cfg.get('imu_model', ''), 'raw_total': self.n_imu}

    def _process_frame(self, spec, body):
        cfg = self.cfg
        try:
            smp = to_physical(spec["decode"](body), spec, cfg)
        except Exception:
            return
        tAbs = smp["week"] * 604800.0 + smp["sec"]
        header = self.parser.frame_header_time
        header_ms = float('nan')
        if header is not None and self._previous_header is not None:
            header_ms = ((header[0]-self._previous_header[0])*604800. + header[1]-self._previous_header[1])*1000.
        self._previous_header = header
        raw_interval_ms = float("nan")
        if self.prev_t is not None:
            dt = tAbs - self.prev_t
            self.last_interval_ms = dt * 1000.
            raw_interval_ms = self.last_interval_ms
            expected = 1. / cfg["freq"]
            if not math.isfinite(dt) or abs(dt - expected) > max(.0005, expected * .15):
                self.interval_anomalies += 1
            if not (0.0 < dt < 0.5):
                dt = 1.0 / cfg["freq"]
        else:
            dt = 1.0 / cfg["freq"]
        self.prev_t = tAbs
        if self.t0 is None:
            self.t0 = tAbs
        tRel = tAbs - self.t0
        if self.replay_elapsed is not None:
            tRel = self.replay_elapsed
        if self._plot_origin is None:
            self._plot_origin = tRel

        self.analysis_archive.append(smp, raw_interval_ms / 1000., self.analysis_cfg, self.analysis_revision)
        raw_values = [v*self.analysis_cfg['gyro_lsb'] for v in smp['gyr_raw']]
        raw_values += [v*self.analysis_cfg['accel_lsb'] for v in smp['acc_raw']]
        self.raw_comparison.append((tAbs if smp['week'] > 0 else float('nan'), *raw_values))
        for history, value in zip(self.physical_channels, raw_values):
            history.append(value)
        roll, pitch, yaw = self.att.update(smp, dt)

        with self.lock:
            self.t.append(tRel-self._plot_origin)
            vals = (math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                    smp["gyr"][0] - self.att.bias[0],
                    smp["gyr"][1] - self.att.bias[1],
                    smp["gyr"][2] - self.att.bias[2],
                    smp["acc"][0], smp["acc"][1], smp["acc"][2])
            for k in range(9):
                self.ch[k].append(vals[k])
            self.rollA.append(math.degrees(self.att.roll_a))
            self.pitchA.append(math.degrees(self.att.pitch_a))
            self.imu_status.append(smp["status"])
            self.temperature.append(smp["temp"])
            self.weeks.append(smp["week"])
            self.seconds.append(smp["sec"])
            self.time_intervals.append(raw_interval_ms)
            self.header_intervals.append(header_ms)
            self.smp = smp
            self.n_imu += 1
            self.total_imu_received += 1
            self.last_sample_wall = time.monotonic()
            self.dt_sum += dt
            self.dt_n += 1
            self.t_last = tRel-self._plot_origin
            self.temp = smp["temp"]
            an = math.sqrt(sum(v * v for v in smp["acc"]))
            if math.isfinite(an) and an < 100.0:      # 剔除异常帧，避免统计被个别坏点带偏
                self.acc_norm_sum += an
                self.acc_norm_n += 1
            elif math.isfinite(an):
                self.n_acc_spike += 1
            self.stat_bytes.add(smp["stat_byte"])
            counter_interval = float("nan")
            if self.counter_prev is not None:
                step = (smp["counter"] - self.counter_prev) % 256
                self.counter_steps[step] = self.counter_steps.get(step, 0) + 1
                counter_interval = smp["counter"] - self.counter_prev
                if counter_interval == -255:
                    counter_interval = 1
            self.counter_intervals.append(counter_interval)
            self.counter_prev = smp["counter"]
            if self.n_imu > 5:
                span = self.t[-1] - self.t[max(0, len(self.t) - 101)]
                if span > 0.05:
                    self.rate_hz = (min(100, len(self.t) - 1)) / span
            if self.recording and self.csv_file:
                row = ("%.6f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.2f,%u,%u,%u,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\r\n"
                                   % (tRel, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5],
                                      vals[6], vals[7], vals[8], smp["temp"], smp["status"],
                                      smp["stat_byte"], smp["counter"],
                                      smp["gyr_raw"][0], smp["gyr_raw"][1], smp["gyr_raw"][2],
                                      smp["acc_raw"][0], smp["acc_raw"][1], smp["acc_raw"][2]))
                try:
                    self.csv_file.write(row)
                except (OSError, ValueError) as e:
                    self.record_error = "记录失败: %s" % e
                    self.stop_record()

    # 录制
    def start_record(self, path):
        with self.lock:
            self.stop_record()
            self.record_error = ""
            try:
                f = open(path, "w", newline="", encoding="utf-8")
                try:
                    f.write("t_s,roll,pitch,yaw,wx,wy,wz,ax,ay,az,temp,status,stat_byte,counter,"
                            "gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw\r\n")
                except Exception:
                    f.close()
                    raise
            except (OSError, ValueError) as e:
                self.record_error = "无法开始记录: %s" % e
                return False
            self.csv_file = f
            self.csv_name = path
            self.recording = True
            return True

    def stop_record(self):
        with self.lock:
            self.recording = False
            if self.csv_file:
                try:
                    self.csv_file.close()
                except (OSError, ValueError) as e:
                    self.record_error = "记录保存失败: %s" % e
                self.csv_file = None

    def reset_attitude(self):
        with self.lock:
            self.att.reset_attitude()

    def close_analysis(self):
        self.analysis_archive.close()

    def clear_data(self):
        """Reset display/analysis only; keep transport, parser fragments and recording time."""
        from performance_data import SampleArchive
        from gpimu_data import GPIMUStore
        with self.lock:
            for values in ([self.t, self.rollA, self.pitchA, self.imu_status, self.temperature,
                            self.weeks, self.seconds, self.time_intervals, self.header_intervals,
                            self.counter_intervals, self.raw_comparison] + self.ch + self.physical_channels):
                values.clear()
            self.gpimu = GPIMUStore(self.t.maxlen)
            self.analysis_archive.close()
            self.analysis_archive = SampleArchive()
            self.clear_generation += 1
            self._plot_origin = self._gp_plot_origin = None
            self._previous_header = self.prev_t = self.counter_prev = None
            self.n_imu = self.dt_n = self.interval_anomalies = 0
            self.dt_sum = self.acc_norm_sum = 0.
            self.acc_norm_n = self.n_acc_spike = 0
            self.counter_steps.clear()
            self.stat_bytes.clear()
            self.smp = self.last_sample_wall = self.last_interval_ms = None
            self.t_last = self.rate_hz = 0.
            self.temp = float('nan')
            self._error_baseline = {name: getattr(self.parser,name) for name in
                ('n_crc','n_text_crc_bad','n_gpimu_crc_bad','n_gpimu_malformed')}

    def reset_calibration(self):
        with self.lock:
            self.att.reset_calibration()

    def set_model(self, models, name):
        with self.lock:
            if apply_model(self.cfg, models, name):
                self.analysis_cfg = copy.deepcopy(self.cfg)
                self.analysis_revision += 1
                size = max(6000, int(math.ceil(self.cfg["freq"] * 65)) + 2)
                self.gpimu.resize(size)
                self.raw_comparison = deque(self.raw_comparison, maxlen=size)
                self.t = deque(self.t, maxlen=size)
                self.ch = [deque(channel, maxlen=size) for channel in self.ch]
                self.physical_channels = [deque(channel, maxlen=size) for channel in self.physical_channels]
                self.rollA = deque(self.rollA, maxlen=size)
                self.pitchA = deque(self.pitchA, maxlen=size)
                self.imu_status = deque(self.imu_status, maxlen=size)
                self.temperature = deque(self.temperature, maxlen=size)
                self.weeks = deque(self.weeks, maxlen=size)
                self.seconds = deque(self.seconds, maxlen=size)
                self.time_intervals = deque(self.time_intervals, maxlen=size)
                self.header_intervals = deque(self.header_intervals, maxlen=size)
                self.counter_intervals = deque(self.counter_intervals, maxlen=size)
                self.att.reset_calibration()
                return True
        return False

    def snapshot(self):
        with self.lock:
            return (list(self.t), [list(c) for c in self.ch],
                    list(self.rollA), list(self.pitchA), copy.deepcopy(self.att),
                    copy.deepcopy(self.smp))

    def status_snapshot(self):
        """Return independent, sample-aligned histories without changing snapshot's API."""
        with self.lock:
            words = list(self.imu_status)
            physical = {key: list(channel) for key, channel in zip(('gx','gy','gz','ax','ay','az'), self.physical_channels)}
            return dict(physical, **{"time": list(self.t), "status": words,
                    "stat_byte": [(word >> 8) & 0xFF for word in words],
                    "counter": [word & 0xFF for word in words],
                    "time_interval_ms": list(self.time_intervals),
                    "header_interval_ms": list(self.header_intervals),
                    "counter_interval": list(self.counter_intervals),
                    "temperature": list(self.temperature), "week": list(self.weeks), "sow": list(self.seconds)})

    def integrity_snapshot(self):
        with self.lock:
            expected_ms = 1000. / self.cfg["freq"]
            return {"crc_errors": self.parser.n_crc + self.parser.n_text_crc_bad-self._error_baseline.get('n_crc',0)-self._error_baseline.get('n_text_crc_bad',0),
                    "counter_jumps": sum(count for step, count in self.counter_steps.items() if step not in (0, 1)),
                    "counter_duplicates": self.counter_steps.get(0, 0),
                    "interval_anomalies": self.interval_anomalies,
                    "last_interval_ms": self.last_interval_ms,
                    "expected_ms": expected_ms, "tolerance_ms": max(.5, expected_ms * .15),
                    "has_samples": self.n_imu > 0}


class DataWorker(threading.Thread):
    def __init__(self, sess):
        threading.Thread.__init__(self)
        self.daemon = True
        self.s = sess
        self.stop_event = threading.Event()
        self.commands = CommandChannel()

    def stop(self):
        self.stop_event.set()
        self.s.running = False
        self.commands.deactivate()


class SerialWorker(DataWorker):

    def _write_command(self, ser, data):
        written = 0
        for first in range(0, len(data), 256):
            if self.stop_event.is_set():
                raise InterruptedError('连接已停止')
            part = data[first:first+256]
            count = ser.write(part)
            written += count or 0
            if count != len(part):
                return written
        return written

    def run(self):
        sess = self.s
        if self.stop_event.is_set():
            sess.stop_raw_record()
            return
        sess.error = ""
        sess.state = "connecting"
        try:
            import serial
        except Exception as e:
            sess.error = "pyserial 不可用: %s" % e
            sess.state = "error"
            sess.stop_raw_record()
            return
        try:
            ser = serial.Serial(sess.cfg["port"], int(sess.cfg["baud"]), timeout=0.05, write_timeout=.25)
        except Exception as e:
            sess.error = "打开串口失败: %s" % e
            sess.state = "error"
            sess.stop_raw_record()
            return
        sess.running = True
        sess.state = "serial"
        sess.status = "已连接 %s @ %d" % (sess.cfg["port"], sess.cfg["baud"])
        try:
            self.commands.activate(ser, '%s @ %d' % (sess.cfg['port'], sess.cfg['baud']))
            self.commands.start_sender(ser, lambda data, target: self._write_command(ser, data))
            while sess.running and not self.stop_event.is_set():
                n = ser.in_waiting
                if n:
                    chunk = ser.read(min(n, 65536))
                    self.commands.observe(chunk)
                    sess.feed(chunk)
                else:
                    self.stop_event.wait(0.01)
        except Exception as e:
            sess.error = "串口读取异常: %s" % e
        finally:
            self.commands.deactivate()
            with sess.raw_lock:
                sess.running = False
                sess.state = "error" if sess.error else "disconnected"
                sess.stop_raw_record()
            try:
                ser.close()
            except Exception:
                pass
            self.commands.wait_senders()


class ReplayWorker(DataWorker):
    def __init__(self, sess, path, speed=1.0, index=None, start_position=0., paused=False):
        DataWorker.__init__(self, sess)
        self.path = path
        self.speed = speed
        self.index = index
        self.start_position = start_position
        self._paused = paused
        self._control_lock = threading.RLock()
        self._wake = threading.Event()
        self._position = 0.
        self._seeking = start_position > 0
        self._scan_progress = 0.
        self._wall_anchor = None
        self._data_anchor = 0.
        self._paused_at = None

    def timeline(self):
        with self._control_lock:
            return {'ready': self.index is not None, 'duration': self.index.duration if self.index else 0.,
                    'position': self._position, 'paused': self._paused, 'seeking': self._seeking,
                    'scan_progress': self._scan_progress}

    def set_paused(self, paused):
        with self._control_lock:
            if bool(paused) != self._paused:
                now = time.monotonic()
                if paused:
                    self._paused_at = now
                elif self._paused_at is not None and self._wall_anchor is not None:
                    self._wall_anchor += now - self._paused_at
                self._paused = bool(paused)
        self._wake.set()

    def stop(self):
        DataWorker.stop(self)
        self._wake.set()

    def _wait_for_frame(self, stamp):
        while not self.stop_event.is_set() and self.s.running:
            with self._control_lock:
                if self._paused:
                    self.s.state = 'paused'
                    delay = .05
                else:
                    self.s.state = 'replay'
                    delay = ((stamp - self._data_anchor) / self.speed - (time.monotonic() - self._wall_anchor)) if self.speed else 0.
                    if delay <= 1e-6:
                        return True
                    delay = min(.05, delay)
            self._wake.wait(delay)
            self._wake.clear()
        return False

    def run(self):
        sess = self.s
        if self.stop_event.is_set():
            return
        sess.error = ""
        sess.state = "connecting"
        if not math.isfinite(self.speed) or self.speed < 0:
            sess.error = "回放速度必须为非负有限数值"
            sess.state = "error"
            return
        sess.running = True
        try:
            from replay_timeline import build_replay_index
            if self.index is None:
                sess.state = 'indexing'
                sess.status = '正在建立回放时间轴：' + os.path.basename(self.path)
                def progress(done, total):
                    with self._control_lock:
                        self._scan_progress = done / float(max(1, total))
                index = build_replay_index(self.path, sess.formats, sess.cfg['freq'],
                                           self.stop_event, progress, FrameParser)
                with self._control_lock:
                    self.index = index
            elif not self.index.matches(self.path):
                raise ValueError('回放文件已变化，请重新打开')
            target_frame = self.index.frame_at(self.start_position)
            sess.gpimu_replay_only = self.index.gpimu_only
            sess.status = '回放 %s%s' % (os.path.basename(self.path), ' · 从起点重算姿态后定位' if target_frame else '')
            with open(self.path, 'rb') as source:
                def check_source():
                    stat = os.fstat(source.fileno())
                    if (stat.st_size, stat.st_mtime_ns) != self.index.signature[1:]:
                        raise ValueError('回放文件已变化，请重新打开')
                for number, end in enumerate(self.index.ends):
                    if self.stop_event.is_set() or not sess.running:
                        break
                    stamp = self.index.times[number]
                    if number > target_frame and not self._wait_for_frame(stamp):
                        break
                    check_source()
                    sess.replay_elapsed = stamp
                    remaining = end - source.tell()
                    if remaining <= 0:
                        raise ValueError('回放时间轴偏移无效，请重新打开')
                    # End each read at the indexed valid IMU frame, including any original
                    # bad/unknown records preceding it. Malformed callbacks consume no index.
                    while remaining and not self.stop_event.is_set() and sess.running:
                        data = source.read(min(65536, remaining))
                        if not data:
                            raise ValueError('回放文件被截断，请重新打开')
                        sess.parser.feed(data)
                        remaining -= len(data)
                    if self.stop_event.is_set() or not sess.running:
                        break
                    if (sess.total_gpimu_received if self.index.gpimu_only else sess.total_imu_received) != number + 1:
                        raise ValueError('回放解析与时间轴不一致，请检查报文格式后重新打开')
                    with self._control_lock:
                        self._position = stamp
                        if number == target_frame:
                            self._seeking = False
                            self._wall_anchor = time.monotonic()
                            self._data_anchor = stamp
                            self._paused_at = self._wall_anchor if self._paused else None
                    sess.state = 'seeking' if number < target_frame else 'paused' if self._paused else 'replay'
                if not self.stop_event.is_set() and sess.running:
                    check_source()
                    # Preserve trailing CRC/unknown-message accounting at normal EOF.
                    while True:
                        if self.stop_event.is_set() or not sess.running:
                            break
                        data = source.read(65536)
                        if not data:
                            break
                        check_source()
                        sess.parser.feed(data)
                    if not self.index.matches(self.path):
                        raise ValueError('回放文件已变化，请重新打开')
        except InterruptedError:
            pass
        except Exception as e:
            sess.error = "回放异常: %s" % e
        finally:
            sess.replay_elapsed = None
            sess.running = False
            sess.state = ("error" if sess.error else
                          "disconnected" if self.stop_event.is_set() else "finished")


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
    """Low-poly F-22 silhouette, body axes X forward / Y right / Z down."""
    parts = []

    def solid(points, offset, color):
        count = len(points)
        vertices = list(points) + [tuple(p[k] + offset[k] for k in range(3)) for p in points]
        faces = [tuple(range(count)), tuple(range(count, 2 * count))]
        faces.extend((i, (i + 1) % count, (i + 1) % count + count, i + count) for i in range(count))
        parts.append((vertices, faces, color))

    # Broad, faceted chine fuselage with a pointed nose and blended wing roots.
    stations = ((2.00, .006, -.01, .01), (1.45, .13, -.10, .10),
                (.80, .36, -.21, .18), (.15, .49, -.15, .20),
                (-.65, .47, -.12, .18), (-1.55, .40, -.08, .12))
    vertices = []
    for x, width, roof, belly in stations:
        height = belly - roof
        vertices.extend(((x, 0, roof), (x, width * .65, roof + height * .12),
                         (x, width, 0), (x, width * .65, belly),
                         (x, 0, belly + height * .04), (x, -width * .65, belly),
                         (x, -width, 0), (x, -width * .65, roof + height * .12)))
    faces = []
    for station in range(len(stations) - 1):
        for k in range(8):
            a, b = station * 8 + k, station * 8 + (k + 1) % 8
            faces.append((a, b, b + 8, a + 8))
    faces.extend((tuple(range(8)), tuple(range(len(vertices) - 8, len(vertices)))))
    parts.append((vertices, faces, (151, 164, 174)))

    for side in (-1, 1):
        # Cropped diamond wings and aft diamond stabilators, with real thickness.
        solid([(.65, side * .30, -.035), (-.40, side * 1.35, -.035),
               (-.95, side * 1.35, -.035), (-1.20, side * .30, -.035)],
              (0, 0, .035), (165, 177, 186))
        solid([(-.80, side * .35, -.025), (-1.22, side * .88, -.025),
               (-1.85, side * .84, -.025), (-1.58, side * .28, -.025)],
              (0, 0, .03), (145, 159, 170))
        # Two outward-canted vertical fins, rather than a central tail.
        solid([(-.57, side * .32, -.10), (-1.50, side * .34, -.07),
               (-1.68, side * .67, -.84), (-1.04, side * .64, -.84)],
              (0, side * .018, 0), (128, 145, 158))
        parts.append(([ (.48, side * .40, -.039), (-.43, side * 1.20, -.039),
                        (-.82, side * .61, -.039)], [(0, 1, 2)], (132, 148, 162)))

        # Side intake ducts: angular lips and dark front openings.
        y = side * .36
        intake = [(.72, y - .13, -.06), (.72, y + .13, -.06),
                  (.72, y + .13, .16), (.72, y - .13, .16),
                  (-.30, y - .15, -.08), (-.30, y + .15, -.08),
                  (-.30, y + .15, .17), (-.30, y - .15, .17)]
        parts.append((intake, [(0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)], (116, 132, 145)))
        parts.append(([(.728, y - .105, -.035), (.728, y + .105, -.035),
                       (.728, y + .105, .135), (.728, y - .105, .135)], [(0, 1, 2, 3)], (37, 48, 58)))

        # Twin rectangular exhausts with dark recessed nozzle faces.
        y = side * .23
        nozzle = [(-1.48, y - .18, -.07), (-1.48, y + .18, -.07),
                  (-1.48, y + .18, .13), (-1.48, y - .18, .13),
                  (-1.94, y - .145, -.025), (-1.94, y + .145, -.025),
                  (-1.94, y + .145, .115), (-1.94, y - .145, .115)]
        parts.append((nozzle, [(0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)], (91, 103, 113)))
        parts.append(([(-1.945, y - .12, -.005), (-1.945, y + .12, -.005),
                       (-1.945, y + .12, .095), (-1.945, y - .12, .095)], [(0, 1, 2, 3)], (31, 37, 43)))

    # Raised, tinted canopy; its apex stays well below either tail-fin tip.
    canopy = [(1.31, 0, -.14), (1.06, -.115, -.19), (.46, -.13, -.19),
              (.38, 0, -.15), (.46, .13, -.19), (1.06, .115, -.19),
              (1.01, 0, -.39), (.59, 0, -.36)]
    parts.append((canopy, [(0, 1, 6), (0, 6, 5), (1, 2, 7, 6),
                           (5, 6, 7, 4), (2, 3, 7), (3, 4, 7)], (180, 150, 91)))
    return parts


class Scene3D(object):
    """Textured OpenGL aircraft with a Canvas compatibility renderer."""

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
        self.gl_view = None
        self.gpu_error = ""
        self._last_update = None
        self._build_static()
        self.resize(max(320, self.w), max(300, self.h))
        if cfg.get("renderer", "auto") != "canvas":
            try:
                from aircraft_gl import DetailedF22View
                self.gl_view = DetailedF22View(canvas, cfg, on_error=self._gpu_failed)
                self.gl_view.place(x=0, y=0, relwidth=1, relheight=1)
            except Exception as exc:
                self.gpu_error = str(exc)

    def _gpu_failed(self, message):
        self.gpu_error = message
        if self._last_update is not None:
            self.update(*self._last_update)

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
        self.cy = self.h * 0.47
        self.scale = min(self.w * .92, self.h * 1.10) / (2.45 * self.al)

    def project(self, p):
        u = dot(p, self.right)
        v = dot(p, self.up)
        d = dot(p, self.fwd)
        return (self.cx + self.scale * u, self.cy - self.scale * v, d)

    # ---- 静态元素：天空、圆台、罗盘、字母 ----
    def _build_static(self):
        c = self.c
        self.background = c.create_rectangle(0, 0, 4000, 3000, fill="#edf3fb", width=0)
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
        self.c.coords(self.background, 0, 0, self.w, self.h)
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
        self._last_update = (R, roll, pitch, yaw, text)
        if self.gl_view is not None and not self.gl_view.failed:
            self.gl_view.set_attitude(R)
            if not self.gl_view.failed:
                return
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
def plot_segments(t, y, width, wrap=False):
    """Reduce finite runs to time/pixel buckets, preserving ordered extrema and gaps."""
    if not t:
        return []
    width = max(1, int(width))
    segments = []
    origin = t[0]
    scale = width / max(1e-9, t[-1] - origin)

    def emit(start, end):
        count = end - start
        if count < 2:
            return
        if count <= 2 * width:
            segments.append(list(zip(t[start:end], y[start:end])))
            return
        reduced = []
        # Native list slicing/min/max do the heavy scan in C. Preserve samples in
        # their original order, including narrow single-sample positive/negative peaks.
        first = start
        while first < end:
            pixel = min(width - 1, int((t[first] - origin) * scale))
            boundary = origin + (pixel + 1) / scale
            stop = bisect_left(t, boundary, first + 1, end)
            values = y[first:stop]
            low, high = min(values), max(values)
            indices = sorted({first, stop - 1, y.index(low, first, stop), y.index(high, first, stop)})
            reduced.extend((t[k], y[k]) for k in indices)
            first = stop
        segments.append(reduced)

    start, previous = 0, None
    isfinite = math.isfinite
    for index, value in enumerate(y):
        invalid = not isfinite(value)
        if invalid or (wrap and previous is not None and abs(value - previous) > 180):
            emit(start, index)
            start = index + 1 if invalid else index
        previous = None if invalid else value
    emit(start, len(y))
    return segments


def step_points(points):
    """Zero-order hold: a digital value changes at the new sample, never between samples."""
    if not points:
        return []
    result = [points[0]]
    previous = points[0][1]
    for tt, value in points[1:]:
        result.append((tt, previous))
        if value != previous:
            result.append((tt, value))
        previous = value
    if len(result) == 1:
        result.append(result[0])
    return result


def clip_points(points, lower, upper):
    """Clip a polyline, including held states already expanded by step_points."""
    if not points or points[-1][0] < lower or points[0][0] > upper:
        return []
    times = [point[0] for point in points]
    first, stop = bisect_left(times, lower), bisect_right(times, upper)
    result = list(points[first:stop])
    for boundary, index, prepend in ((lower, first, True), (upper, stop, False)):
        if 0 < index < len(points) and times[index - 1] < boundary < times[index]:
            a, b = points[index - 1], points[index]
            value = a[1] + (b[1] - a[1]) * (boundary - a[0]) / (b[0] - a[0])
            point = (boundary, value)
            if prepend:
                result.insert(0, point)
            else:
                result.append(point)
    if len(result) == 1:
        result.append(result[0])
    return result


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
        self.decor = {}
        self.decor_cache = {}
        self.hidden_decor = set()
        self.wrap_angles = False
        self.stepped = False
        self.hex_digits = 0
        self.integer_values = False
        self.time_range = None
        self._last_t, self._last_ys = [], []
        self._bounds = None
        self._cursor = None
        self.render_enabled = True
        self.left_margin = 64
        self.compact_header = False
        self.value_decimals = 2
        self.legend_values = True
        self.reference_value = None
        self.title_color = "#172b49"
        self.navigation = PlotNavigation(canvas, self._redraw_cached, enabled=False)
        self._cursor_value = None
        self.time_axis = None

    def bind_navigation(self, owner=None):
        self.navigation.owner = weakref.ref(owner) if owner is not None else None
        self.navigation.enabled = owner is None

    def _redraw_cached(self):
        self.update(self._last_t, self._last_ys)
        self.show_cursor(self._cursor_value)

    def clear_plot(self):
        self.navigation.view = self.navigation.drag = self.navigation.base = None
        self.cur = None
        self._last_t, self._last_ys = [], [[] for _ in self.series]
        self.show_cursor(None)
        for item in self.pool.values():
            self.c.itemconfigure(item, state='hidden')

    def value_text(self, value):
        if not math.isfinite(value):
            return "—"
        if self.hex_digits:
            return "0x%0*X" % (self.hex_digits, int(round(value)))
        return "%d" % value if self.integer_values else "%+.*f" % (self.value_decimals, value)

    def bind_cursor(self, callback):
        def motion(event):
            if self._bounds and self._last_t:
                left, top, width, height, t0, t1 = self._bounds
                if left <= event.x <= left + width and top <= event.y <= top + height:
                    callback(t0 + (event.x - left) * (t1 - t0) / width)
        self.c.bind("<Motion>", motion)

    def show_cursor(self, value):
        self._cursor_value = value
        if not self._bounds:
            return
        left, top, width, height, t0, t1 = self._bounds
        if self._cursor is None:
            self._cursor = self.c.create_line(0, 0, 0, 0, fill="#546b85", dash=(3, 3), width=1)
        if value is None or not t0 <= value <= t1:
            self.c.itemconfigure(self._cursor, state="hidden")
            return
        x = left + (value - t0) * width / (t1 - t0)
        self.c.coords(self._cursor, x, top, x, top + height)
        self.c.itemconfigure(self._cursor, state="normal")
        self.c.tag_raise(self._cursor)

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

    @classmethod
    def _time_step(cls, span):
        step = cls._nice_step(span, 5)
        return next((value for value in (.5, 1, 2, 5, 10, 15, 30, 60, 120, 300)
                     if value >= step), step)

    def _y_range(self, ys):
        if self.ylim:
            lo, hi = self.ylim
            return lo, hi, self._nice_step(hi - lo, 4)
        vals = [v for y in ys for v in y if math.isfinite(v)]
        if not vals:
            return self.cur or (0.0, 1.0, 0.5)
        dlo, dhi = min(vals), max(vals)
        minimum_span = self.min_span if self.min_span > 0 else .02
        if dhi - dlo < minimum_span:
            mid = 0.5 * (dhi + dlo)
            half = 0.5 * minimum_span
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

    def _decoration(self, key, kind, coords, **options):
        item = self.decor.get(key)
        if item is None:
            item = getattr(self.c, "create_" + kind)(*coords, tags="dyn", **options)
            self.decor[key] = item
        elif self.decor_cache.get(key) != (coords, options) or key in self.hidden_decor:
            self.c.coords(item, *coords)
            self.c.itemconfigure(item, state="normal", **options)
        self.decor_cache[key] = (coords, options)
        self.hidden_decor.discard(key)
        self.used_decor.add(key)

    def update(self, t, ys):
        self._last_t, self._last_ys = t, ys
        if not self.render_enabled:
            return
        c = self.c
        L, R = (100 if self.hex_digits == 8 else self.left_margin), 18
        T, B = (28, 22) if self.compact_header else (54, 26)
        pw, ph = self.w - L - R, self.h - T - B
        if pw < 10 or ph < 10:
            return
        t0, t1 = self.time_range or ((t[0], max(t[0] + .05, t[-1])) if t else (0., 10.))
        range_series = list(ys) + [[self.reference_value]] if self.reference_value is not None else ys
        lo, hi, ystep = self._y_range(range_series)
        if self.compact_header:
            ystep = self._nice_step(hi - lo, max(1, int(ph / 24)))
        if self.hex_digits:
            lo, hi = max(0., lo), min(float((1 << (4 * self.hex_digits)) - 1), hi)
            ystep = max(1., ystep)
        t0, t1, lo, hi = self.navigation.configure((t0, t1, lo, hi), (L, T, pw, ph))
        if self.navigation.view:
            ystep = max(1., self._nice_step(hi-lo)) if self.hex_digits else self._nice_step(hi-lo)
        self._bounds = (L, T, pw, ph, t0, t1)
        sx = pw / (t1 - t0)
        sy = ph / (hi - lo)
        # 画顺序：先参考虚线（在底层），再实线（压在上面）
        order = [k for k in range(len(ys)) if k in self.dash] + \
                [k for k in range(len(ys)) if k not in self.dash]
        for k in order:
            y = ys[k]
            first = max(0, bisect_left(t, t0)-1) if self.navigation.view else 0
            last = bisect_right(t, t1)+1 if self.navigation.view else len(t)
            segs = plot_segments(t[first:last], y[first:last], pw, wrap=self.wrap_angles)
            segs = [clip_points(step_points(segment) if self.stepped else segment, t0, t1) for segment in segs]
            segs = [segment for segment in segs if segment]
            segs = [clipped for segment in segs for clipped in clip_polyline(segment, t0, t1, lo, hi)]
            for si, segment in enumerate(segs):
                pts = []
                for tt, vv in segment:
                    pts.extend((L + (tt - t0) * sx, T + ph - (vv - lo) * sy))
                it = self._item(k, si)
                c.coords(it, *pts)
                c.itemconfigure(it, state="normal")
                c.tag_raise(it)
            for (series, si), it in self.pool.items():
                if series == k and si >= len(segs):
                    c.itemconfigure(it, state="hidden")
        # 网格与刻度（用 tag 统一擦除/置底）
        self.used_decor = set()
        d = self._decoration
        if self.reference_value is not None and lo <= self.reference_value <= hi:
            reference_y = T + ph - (self.reference_value - lo) * sy
            d("reference", "line", (L, reference_y, L + pw, reference_y), fill="#929daf", dash=(4, 3))
        k = int(math.ceil(lo / ystep))
        tick_index = 0
        while k * ystep <= hi + 1e-9:
            v = k * ystep
            yy = T + ph - (v - lo) * sy
            d(("yline", tick_index), "line", (L, yy, L + pw, yy), fill="#e8edf4")
            axis_text = self.value_text(v) if self.hex_digits else self._fmt(v, ystep)
            d(("ylabel", tick_index), "text", (L - 8, yy), text=axis_text, anchor="e", fill="#65758b",
              font=("Consolas", 9))
            k += 1
            tick_index += 1
        # 时间轴刻度：从 0.5 s 到 5 min 自适应
        xstep = self._nice_step(t1-t0, 5) if self.navigation.view else self._time_step(t1 - t0)
        k0 = int(math.ceil(t0 / xstep))
        while k0 * xstep <= t1 + 1e-9:
            v = k0 * xstep
            xx = L + (v - t0) * sx
            # Keys follow tick positions on screen, so scrolling does not grow the pool.
            key = k0 - int(math.ceil(t0 / xstep))
            d(("xline", key), "line", (xx, T, xx, T + ph), fill="#f0f3f8")
            d(("xlabel", key), "text", (xx, T + ph + 5),
              text=self.time_axis.tick_text(v, xstep) if self.time_axis else self._fmt(v, xstep) + " s",
              anchor="n", fill="#65758b", font=("Consolas", 9))
            k0 += 1
        d("title", "text", (16, 12), text=self.title, anchor="w",
          fill=self.title_color, font=("Microsoft YaHei", 9 if self.compact_header else 11, "bold"))
        unit_text = (self.value_text(ys[0][-1]) + " " + self.ylabel) if self.compact_header and ys and ys[0] else self.ylabel
        d("unit", "text", (self.w - 18, 12), text=unit_text, anchor="e",
          fill="#65758b", font=("Consolas", 10))
        spacing = (self.w - 28) / max(1, len(self.series))
        for k, name in enumerate([] if self.compact_header else self.series):
            xx = 16 + spacing * k
            options = dict(fill=self.cols[k % len(self.cols)], width=2)
            if k in self.dash:
                options["dash"] = (5, 3)
            d(("legendline", k), "line", (xx, 35, xx + 16, 35), **options)
            value = ys[k][-1] if k < len(ys) and ys[k] else float("nan")
            label = name + (" " + self.value_text(value) if self.legend_values and spacing >= 110 else "")
            d(("legendlabel", k), "text", (xx + 21, 35), text=label, anchor="w",
              fill="#44536b", font=("Microsoft YaHei", 9))
        if not t:
            d("empty", "text", (L + pw / 2, T + ph / 2), text="等待 IMU 数据",
              fill="#8997a9", font=("Microsoft YaHei", 11))
        for key, item in self.decor.items():
            if key not in self.used_decor and key not in self.hidden_decor:
                c.itemconfigure(item, state="hidden")
                self.hidden_decor.add(key)
        c.tag_lower("dyn")


class IntegrityPanel(PlotPanel):
    """Two aligned interval traces, with cumulative integrity counts in their headers."""

    def __init__(self, canvas):
        import tkinter as tk
        PlotPanel.__init__(self, canvas, "数据完整性监控", "", [])
        self.metrics = {}
        self.time_canvas = tk.Canvas(canvas, bg="white", highlightthickness=0)
        self.counter_canvas = tk.Canvas(canvas, bg="white", highlightthickness=0)
        self.time_plot = PlotPanel(self.time_canvas, "采样时间间隔", "ms", ["Δt"], colors=("#3c70d6",), min_span=.5)
        self.counter_plot = PlotPanel(self.counter_canvas, "IMU 计数间隔", "count", ["Δcount"], colors=("#278b81",), min_span=.5)
        self.counter_plot.integer_values = True
        self.counter_plot.reference_value = 1
        self.time_plot.value_decimals = 3
        for panel in (self.time_plot, self.counter_plot):
            panel.compact_header = True
        self.resize(100, 100)

    def resize(self, width, height):
        PlotPanel.resize(self, width, height)
        half = self.h // 2
        self.time_canvas.place(x=0, y=0, width=self.w, height=half - 2)
        self.counter_canvas.place(x=0, y=half + 2, width=self.w, height=self.h - half - 2)
        self.time_plot.resize(self.w, half - 2)
        self.counter_plot.resize(self.w, self.h - half - 2)

    def bind_cursor(self, callback):
        self.time_plot.bind_cursor(callback)
        self.counter_plot.bind_cursor(callback)

    def bind_navigation(self, owner=None):
        self.time_plot.bind_navigation(owner)
        self.counter_plot.bind_navigation(owner)

    def _redraw_cached(self):
        if self.metrics:
            self.update(self._last_t, self.metrics)

    def show_cursor(self, value):
        self.time_plot.show_cursor(value)
        self.counter_plot.show_cursor(value)

    def update(self, t, metrics):
        self._last_t, self._last_ys, self.metrics = t, [], dict(metrics)
        self.time_plot.title = "时间间隔 · CRC %d / 异常 %d" % (metrics['crc_errors'], metrics['interval_anomalies'])
        self.counter_plot.title = "计数间隔 · 跳变 %d / 重复 %d" % (metrics['counter_jumps'], metrics['counter_duplicates'])
        self.time_plot.title_color = "#b43b3b" if metrics['crc_errors'] or metrics['interval_anomalies'] else "#172b49"
        self.counter_plot.title_color = "#b43b3b" if metrics['counter_jumps'] or metrics['counter_duplicates'] else "#172b49"
        self.time_plot.reference_value = metrics['expected_ms']
        for panel in (self.time_plot, self.counter_plot):
            panel.time_range = self.time_range
            panel.time_axis = self.time_axis
            panel.render_enabled = self.render_enabled
            if not t:
                panel.cur = None
        self.time_plot.update(t, [metrics['time_interval_ms']])
        self.counter_plot.update(t, [metrics['counter_interval']])


class StatusPanel(PlotPanel):
    """Named fault lanes following the project's rawimu_status.m convention."""

    def __init__(self, canvas):
        PlotPanel.__init__(self, canvas, "IMU 故障位", "bit 8–15", [],
                           colors=("#3b78aa",) * 8)
        self.left_margin = 116
        self.navigation.horizontal_only = True

    def update(self, t, ys):
        self._last_t, self._last_ys = t, ys
        if not self.render_enabled:
            return
        words = ys[0] if ys else []
        left, top, width, height = self.left_margin, 54, self.w - self.left_margin - 18, self.h - 80
        if width < 10 or height < 10:
            return
        t0, t1 = self.time_range or ((t[0], max(t[0] + .05, t[-1])) if t else (0., 10.))
        t0, t1 = self.navigation.configure((t0, t1, 0., 1.), (left, top, width, height))[:2]
        self._bounds = (left, top, width, height, t0, t1)
        sx = width / (t1 - t0)
        lane_height = height / 8.
        self.used_decor = set()
        d = self._decoration
        d("title", "text", (16, 12), text="IMU 故障位", anchor="w",
          fill="#172b49", font=("Microsoft YaHei", 11, "bold"))
        byte = (words[-1] >> 8) & 255 if words else None
        text = ("StatusByte 0x%02X   1=异常 / 0=正常" % byte) if byte is not None else "StatusByte —   等待 IMU 数据"
        d("legend", "text", (16, 35), text=text, anchor="w", fill="#536780", font=("Microsoft YaHei", 9))
        for row in range(8):
            bit = 8 + row
            y0 = top + row * lane_height
            label = "%s [B%d] %s" % (IMU_STATUS_FAULT_NAMES[row], bit,
                                     "—" if byte is None else str((words[-1] >> bit) & 1))
            label_color = "#b43b3b" if byte is not None and byte & (1 << row) else "#44536b"
            d(("band", row), "rectangle", (left, y0, left + width, y0 + lane_height),
              fill="#f1f5fa" if row % 2 == 0 else "#ffffff", outline="")
            d(("bitlabel", row), "text", (left - 8, y0 + lane_height / 2), text=label, anchor="e",
              fill=label_color, font=("Consolas", 9 if lane_height >= 16 else 8 if lane_height >= 13 else 7))
            values = [(word >> bit) & 1 for word in words]
            first = max(0, bisect_left(t, t0)-1) if self.navigation.view else 0
            last = bisect_right(t, t1)+1 if self.navigation.view else len(t)
            segments = plot_segments(t[first:last], values[first:last], width)
            segments = [clip_points(step_points(segment), t0, t1) for segment in segments]
            segments = [segment for segment in segments if segment]
            for si, segment in enumerate(segments):
                coords = []
                for tt, value in segment:
                    coords.extend((left + (tt - t0) * sx,
                                   y0 + lane_height * (.23 if value else .77)))
                item = self._item(row, si)
                self.c.coords(item, *coords)
                self.c.itemconfigure(item, state="normal", width=1.5)
                self.c.tag_raise(item)
            for (series, si), item in self.pool.items():
                if series == row and si >= len(segments):
                    self.c.itemconfigure(item, state="hidden")
        step = self._nice_step(t1-t0, 5) if self.navigation.view else self._time_step(t1 - t0)
        tick = math.ceil(t0 / step) * step
        index = 0
        while tick <= t1 + 1e-9:
            x = left + (tick - t0) * sx
            d(("xline", index), "line", (x, top, x, top + height), fill="#dde5ef")
            d(("xlabel", index), "text", (x, top + height + 5),
              text=self.time_axis.tick_text(tick, step) if self.time_axis else "%g s" % tick,
              anchor="n", fill="#65758b", font=("Consolas", 9))
            tick += step
            index += 1
        for key, item in self.decor.items():
            if key not in self.used_decor and key not in self.hidden_decor:
                self.c.itemconfigure(item, state="hidden")
                self.hidden_decor.add(key)
        self.c.tag_lower("dyn")


# ---------------------------------------------------------------- 主界面
class App(object):
    BG = "#eef2f7"
    INK = "#172b49"
    MUTED = "#63748b"
    COLORS = ("#d34c4c", "#20956b", "#3c70d6")

    def __init__(self, root, cfg, replay=None, autostart=False, models=None, formats=None):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk, self.root, self.cfg = tk, ttk, root, cfg
        self.replay = replay
        self.models = models or load_imu_models()
        self.formats = formats or FormatRegistry()
        self.sess = Session(cfg, self.formats)
        self.worker = None
        self._pending_replay_seek = None
        self._live_calibration = (cfg["auto_bias"], cfg["auto_accel_scale"])
        self._closing = False
        self._after = None
        self._dirty = True
        self._drawn = -1
        self._integrity_drawn = None
        self.details_window = None
        self.axes_window = None
        self.performance_window = None
        self.frozen = False
        self._status_view = None
        self._display_data = None
        self._cursor_time = None
        self.time_axis_var = tk.StringVar(master=root, value=next(
            (name for name, mode in AXIS_MODES.items() if mode == cfg.get('time_axis')), '相对时间 (s)'))
        self.chart_time_axis = TimeAxis()
        self.measurement_var = tk.StringVar(master=root, value='原始物理量')
        root.title("RAWIMU Workbench · IMU 监测与状态分析")
        root.geometry("1600x960")
        root.minsize(1100, 720)
        root.configure(bg=self.BG)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TButton", font=("Microsoft YaHei", 10), padding=(10, 5))
        style.configure("TButton", background="#f4f6f9", bordercolor="#c4cedb")
        style.configure("TCombobox", font=("Microsoft YaHei", 10), padding=3)
        style.configure("Accent.TButton", background="#2563b9", foreground="white")
        style.map("Accent.TButton", background=[("active", "#1d519a")])
        style.configure("TProgressbar", background="#20956b", troughcolor="#e5ebf3")
        style.configure("TLabel", background="white")
        style.configure("TLabelframe", background="white", bordercolor="#cfd8e4")
        style.configure("TLabelframe.Label", background="white", foreground=self.INK,
                        font=("Microsoft YaHei", 10, "bold"))
        style.configure("TNotebook", background=self.BG, borderwidth=0)
        style.configure("TNotebook.Tab", background="#e6edf5", padding=(16, 6), font=("Microsoft YaHei", 10))
        style.map("TNotebook.Tab", background=[("selected", "#ffffff")], foreground=[("selected", "#1b5b97")])

        header = tk.Frame(root, bg=self.INK)
        header.pack(fill="x")
        tk.Label(header, text="RAWIMU WORKBENCH", bg=self.INK, fg="white",
                 font=("Consolas", 16, "bold")).pack(side="left", padx=18, pady=7)
        tk.Label(header, text="IMU 监测 / 姿态 / 状态分析", bg=self.INK, fg="#b9cbe4",
                 font=("Microsoft YaHei", 11)).pack(side="left")
        self.state_var = tk.StringVar(value="未连接")
        self.source_badge = tk.Label(header, textvariable=self.state_var, bg="#34465f", fg="white",
                                     font=("Microsoft YaHei", 10, "bold"), padx=16, pady=5)
        self.source_badge.pack(side="right", padx=18)

        tools = tk.Frame(root, bg="white", padx=14, pady=7)
        tools.pack(fill="x", padx=12, pady=(10, 6))
        row = tk.Frame(tools, bg="white")
        row.pack(fill="x", pady=(0, 5))
        self._label(row, "数据源", bold=True)
        self.source_var = tk.StringVar(master=root, value=next(
            (name for name, mode in NETWORK_MODES.items() if mode == cfg.get('source')), '串口'))
        self._last_source = self.source_var.get()
        self.source_box = ttk.Combobox(row, textvariable=self.source_var, width=11,
                                      state='readonly', values=('串口',) + tuple(NETWORK_MODES))
        self.source_box.pack(side='left', padx=(0, 8))
        self.source_box.bind('<<ComboboxSelected>>', self.on_source_changed)
        self.serial_controls = tk.Frame(row, bg='white')
        self.serial_controls.pack(side='left')
        self._label(self.serial_controls, "串口")
        self.port_var = tk.StringVar(value=cfg["port"])
        self.port_box = ttk.Combobox(self.serial_controls, textvariable=self.port_var, width=9)
        self.port_box.pack(side="left", padx=(0, 4))
        self.btn_ports = ttk.Button(self.serial_controls, text="刷新", width=5, command=self.refresh_ports)
        self.btn_ports.pack(side="left", padx=(0, 10))
        self._label(self.serial_controls, "波特率")
        self.baud_var = tk.StringVar(value=str(cfg["baud"]))
        self.baud_box = ttk.Combobox(self.serial_controls, textvariable=self.baud_var, width=8,
                                     values=("921600", "460800", "230400", "115200"))
        self.baud_box.pack(side="left", padx=(0, 8))
        self.network_controls = tk.Frame(row, bg='white')
        self.host_label = self._label(self.network_controls, '目标 IP')
        self.host_var = tk.StringVar(master=root, value=cfg.get('network_host', '127.0.0.1'))
        self.host_entry = ttk.Entry(self.network_controls, textvariable=self.host_var, width=15)
        self.host_entry.pack(side='left', padx=(0, 8))
        self._label(self.network_controls, '端口')
        self.network_port_var = tk.StringVar(master=root, value=str(cfg.get('network_port', 3001)))
        self.network_port_entry = ttk.Entry(self.network_controls, textvariable=self.network_port_var, width=6)
        self.network_port_entry.pack(side='left', padx=(0, 8))
        self.btn_open = ttk.Button(row, text="连接串口", style="Accent.TButton", command=self.toggle_port)
        self.btn_open.pack(side="left", padx=(0, 16))
        self.btn_replay = ttk.Button(row, text="打开回放…", command=self.open_replay)
        self.btn_replay.pack(side="left", padx=(0, 8))
        self._label(row, "回放速度")
        self.speed_var = tk.StringVar(value="%g" % cfg["replay_speed"])
        self.speed_box = ttk.Combobox(row, textvariable=self.speed_var, width=5, state="readonly",
                                      values=("0.25", "0.5", "1", "2", "4", "0"))
        self.speed_box.pack(side="left", padx=(0, 5))
        self._label(row, "倍  ·  0 = 极速")
        self.btn_raw = ttk.Button(row, text="保存原始 DAT…", command=self.toggle_raw_record)
        self.btn_raw.pack(side="left", padx=(12, 0))

        row = tk.Frame(tools, bg="white")
        row.pack(fill="x")
        self._label(row, "姿态解算", bold=True)
        self._label(row, "IMU")
        self.imu_var = tk.StringVar(value=cfg.get("imu_model", ""))
        self.imu_box = ttk.Combobox(row, textvariable=self.imu_var, width=9, state="readonly",
                                    values=sorted(self.models))
        self.imu_box.pack(side="left", padx=(0, 9))
        self.imu_box.bind("<<ComboboxSelected>>", lambda e: self.on_imu())
        self.mode_var = tk.StringVar(value=cfg["mode"])
        for label, value in (("陀螺 + 重力", "complementary"), ("纯陀螺", "gyro")):
            tk.Radiobutton(row, text=label, value=value, variable=self.mode_var, command=self.on_mode,
                           bg="white", activebackground="white", font=("Microsoft YaHei", 10)).pack(side="left")
        self._label(row, "修正 τ(s)")
        self.tau_var = tk.StringVar(value="%.1f" % cfg["level_tau"])
        self.tau_box = ttk.Combobox(row, textvariable=self.tau_var, width=4, state="readonly",
                                    values=("0.5", "1.0", "2.0", "5.0"))
        self.tau_box.pack(side="left", padx=(0, 10))
        self.tau_box.bind("<<ComboboxSelected>>", lambda e: self.on_tau())
        ttk.Button(row, text="姿态归零", command=self.on_reset).pack(side="left", padx=3)
        ttk.Button(row, text="重新标定", command=self.on_recal).pack(side="left", padx=3)
        self.btn_rec = ttk.Button(row, text="开始 CSV…", command=self.toggle_record)
        self.btn_rec.pack(side="left", padx=(10, 3))
        from replay_timeline import ReplayTimeline
        self.replay_timeline = ReplayTimeline(tools, self)
        self.refresh_ports()

        # Bottom status remains visible regardless of the drawing area's dimensions.
        footer = tk.Frame(root, bg="white", padx=14, pady=8)
        footer.pack(side="bottom", fill="x", padx=12, pady=(0, 10))
        ttk.Button(footer, text="诊断详情", command=self.open_details).pack(side="right", padx=(12, 0))
        self.metrics_var = tk.StringVar(value="采样率 —    温度 —    IMU 帧 0    CRC 错误 0")
        tk.Label(footer, textvariable=self.metrics_var, bg="white", fg=self.INK,
                 font=("Microsoft YaHei", 10), anchor="w").pack(fill="x")
        calibration = tk.Frame(footer, bg="white")
        calibration.pack(fill="x", pady=(5, 0))
        self.cal_var = tk.StringVar(value="标定：等待数据")
        tk.Label(calibration, textvariable=self.cal_var, bg="white", fg=self.MUTED,
                 font=("Microsoft YaHei", 9)).pack(side="left")
        self.cal_progress = ttk.Progressbar(calibration, maximum=100, length=110)
        self.cal_progress.pack(side="left", padx=10)
        self.record_var = tk.StringVar(value="记录：未开始")
        self.record_label = tk.Label(calibration, textvariable=self.record_var, bg="white", fg=self.MUTED,
                                     font=("Microsoft YaHei", 9), anchor="w")
        self.record_label.pack(side="left", fill="x", expand=True)
        self.message_var = tk.StringVar(value="选择串口并连接，或打开 .dat 文件回放。")
        self.message_label = tk.Label(footer, textvariable=self.message_var, bg="white", fg=self.MUTED,
                                      font=("Microsoft YaHei", 9), anchor="w", justify="left")
        self.message_label.pack(fill="x", pady=(5, 0))
        self.cursor_var = tk.StringVar(value="游标：在任意图表内移动鼠标，读取同步样本")
        tk.Label(footer, textvariable=self.cursor_var, bg="white", fg="#315c86",
                 font=("Consolas", 9), anchor="w").pack(fill="x", pady=(3, 0))
        footer.bind("<Configure>", lambda e: self.message_label.configure(wraplength=max(100, e.width - 30)))

        body = tk.Frame(root, bg=self.BG)
        body.pack(fill="both", expand=True, padx=12, pady=(0, 8))
        self.main_split = ttk.Panedwindow(body, orient="horizontal")
        self.main_split.pack(fill="both", expand=True)
        self._layout_initialized = False
        self.main_split.bind("<Configure>", self._initial_layout)
        left = tk.Frame(self.main_split, bg="white")
        self.main_split.add(left, weight=1)
        top = tk.Frame(left, bg="white")
        top.pack(fill="x", padx=14, pady=(10, 4))
        tk.Label(top, text="三维姿态 · F-22", bg="white", fg=self.INK,
                 font=("Microsoft YaHei", 12, "bold")).pack(side="left")
        ttk.Button(top, text="恢复视角", width=8, command=self.reset_view).pack(side="right")
        cards = tk.Frame(left, bg="white")
        cards.pack(fill="x", padx=10, pady=6)
        from tkinter.font import Font
        self._angle_font = Font(root=root, family="Consolas", size=19, weight="bold")
        cards.bind("<Configure>", self.on_angles_resize)
        self.angle_vars = [tk.StringVar(value="—") for _ in range(3)]
        for k, label in enumerate(("横滚 Roll", "俯仰 Pitch", "航向 Yaw")):
            cards.columnconfigure(k, weight=1, uniform="angle")
            card = tk.Frame(cards, bg="#f4f7fb", padx=5, pady=8)
            card.grid(row=0, column=k, sticky="ew", padx=3)
            tk.Label(card, text=label, bg="#f4f7fb", fg=self.MUTED,
                     font=("Microsoft YaHei", 10)).pack()
            tk.Label(card, textvariable=self.angle_vars[k], bg="#f4f7fb", fg=self.COLORS[k],
                     font=self._angle_font).pack(pady=(4, 0))
            tk.Label(card, text="度 °", bg="#f4f7fb", fg=self.MUTED, font=("Microsoft YaHei", 9)).pack()
        current = ttk.LabelFrame(left, text="最新样本 · IMU 字段", padding=(10, 5))
        self.current_group = current
        current.pack(fill="x", padx=12, pady=(6, 0))
        self.status_word_var = tk.StringVar(value="—")
        self.status_byte_var = tk.StringVar(value="—")
        self.counter_var = tk.StringVar(value="—")
        self.status_temp_var = tk.StringVar(value="—")
        for k, (label, var) in enumerate((("Status (u32)", self.status_word_var),
                                          ("StatusByte [15:8]", self.status_byte_var),
                                          ("Counter [7:0]", self.counter_var),
                                          ("Temperature", self.status_temp_var))):
            ttk.Label(current, text=label, font=("Consolas", 10), foreground=self.MUTED).grid(row=k, column=0, sticky="w", pady=2)
            ttk.Label(current, textvariable=var, font=("Consolas", 11, "bold"), foreground=self.INK).grid(row=k, column=1, sticky="e", padx=(10, 0))
        current.columnconfigure(1, weight=1)
        self.cv3d = tk.Canvas(left, bg="#edf3fb", width=1, height=1, highlightthickness=0)
        self.cv3d.pack(fill="both", expand=True, padx=12, pady=(6, 0))
        self.scene = Scene3D(self.cv3d, cfg)
        tk.Label(left, text="机体坐标：X 前 · Y 右 · Z 下\n航向由陀螺积分，随时间可能漂移",
                 bg="white", fg=self.MUTED, font=("Microsoft YaHei", 9), pady=8).pack(fill="x")

        right = tk.Frame(self.main_split, bg=self.BG)
        self.main_split.add(right, weight=3)
        viewbar = tk.Frame(right, bg=self.BG)
        viewbar.pack(fill="x", padx=6, pady=(0, 5))
        self._label(viewbar, "时间窗口", background=self.BG)
        self.window_var = tk.StringVar(value="%g" % cfg["window_s"])
        self.window_box = ttk.Combobox(viewbar, textvariable=self.window_var, width=4,
                                       state="readonly", values=("10", "30", "60"))
        self.window_box.pack(side="left")
        self.window_box.bind("<<ComboboxSelected>>", lambda e: self.on_window())
        self._label(viewbar, "秒", background=self.BG)
        self.freeze_text = tk.StringVar(value="冻结显示")
        ttk.Button(viewbar, textvariable=self.freeze_text, command=self.toggle_freeze).pack(side="left", padx=6)
        ttk.Button(viewbar, text='清空数据', command=self.clear_data).pack(side='left', padx=(0, 6))
        ttk.Button(viewbar, text="恢复布局", command=self.restore_layout).pack(side="left")
        ttk.Button(viewbar, text="六轴独立分析…", command=self.open_axes_window).pack(side="left", padx=6)
        ttk.Button(viewbar, text="静态性能分析…", command=self.open_performance_window).pack(side="left")
        axisbar = tk.Frame(right, bg=self.BG)
        axisbar.pack(fill='x', padx=8, pady=(0, 4))
        self._label(axisbar, '横轴', background=self.BG)
        axisbox = ttk.Combobox(axisbar, textvariable=self.time_axis_var, values=tuple(AXIS_MODES),
                               state='readonly', width=18)
        axisbox.pack(side='left', padx=(0, 8))
        axisbox.bind('<<ComboboxSelected>>', self.on_axis_changed)
        measurement_box = ttk.Combobox(axisbar, textvariable=self.measurement_var,
                                      values=('原始物理量', '解算补偿量'), state='readonly', width=12)
        measurement_box.pack(side='left', padx=(0, 8))
        measurement_box.bind('<<ComboboxSelected>>', self.on_measurement_changed)
        tk.Label(axisbar, text="冻结：滚轮缩放 · 拖动平移 · 双击恢复",
                 bg=self.BG, fg=self.MUTED, anchor="w", font=("Microsoft YaHei", 9)).pack(side='left')
        self.notebook = ttk.Notebook(right)
        self.notebook.pack(fill="both", expand=True, padx=(6, 0))
        overview = tk.Frame(self.notebook, bg=self.BG)
        status_page = tk.Frame(self.notebook, bg=self.BG)
        self.overview_page, self.status_page = overview, status_page
        self.notebook.add(overview, text="监控总览")
        self.notebook.add(status_page, text="IMU 状态分析")
        self.gpimu_page = tk.Frame(self.notebook, bg=self.BG)
        self.notebook.add(self.gpimu_page, text='GPIMU 分析')
        from gpimu_view import GPIMUView
        self.gpimu_view = GPIMUView(self.gpimu_page, self, PlotPanel)
        self.command_page = tk.Frame(self.notebook, bg=self.BG)
        self.notebook.add(self.command_page, text='命令发送')
        from command_view import CommandView
        self.command_view = CommandView(self.command_page, self)
        self._gp_auto_selected_worker = None
        self.notebook.bind("<<NotebookTabChanged>>", lambda e: self._mark_dirty())

        def make_grid(parent):
            for k in range(2):
                parent.columnconfigure(k, weight=1, uniform="plotcol")
                parent.rowconfigure(k, weight=1, uniform="plotrow")
            result = []
            for k in range(4):
                canvas = tk.Canvas(parent, bg="white", width=100, height=100, highlightthickness=1,
                                   highlightbackground="#cfd8e4")
                canvas.grid(row=k // 2, column=k % 2, sticky="nsew", padx=3, pady=3)
                result.append(canvas)
            return result

        self.cv_eul, self.cv_gyr, self.cv_acc, self.cv_integrity = make_grid(overview)
        self.p_eul = PlotPanel(self.cv_eul, "姿态角", "deg", ["Roll", "Pitch", "Yaw", "R参", "P参"],
                               colors=self.COLORS, dashed=(3, 4), min_span=2.)
        self.p_eul.wrap_angles = True
        self.p_gyr = PlotPanel(self.cv_gyr, "角速度 · 去零偏", "deg/s", ["X", "Y", "Z"],
                               colors=self.COLORS, min_span=.1)
        self.p_acc = PlotPanel(self.cv_acc, "加速度", "m/s²", ["X", "Y", "Z"],
                               colors=self.COLORS, min_span=2.)
        self.p_integrity = IntegrityPanel(self.cv_integrity)
        raw_cv, self.cv_status, temp_cv, count_cv = make_grid(status_page)
        self.p_bits = StatusPanel(self.cv_status)
        self.p_status_raw = PlotPanel(raw_cv, "原始 IMU_Status", "u32 / hex", ["Status"], colors=("#596bc2",), min_span=16.)
        self.p_status_raw.hex_digits, self.p_status_raw.stepped = 8, True
        self.p_temperature = PlotPanel(temp_cv, "温度 · bit 16–31 换算", "°C", ["Temperature"], colors=("#b47b24",), min_span=.2)
        self.p_temperature.left_margin = 100
        self.p_counter = PlotPanel(count_cv, "采样计数器 · bit 0–7", "count", ["Counter"], colors=("#278b81",), ylim=(0, 255))
        self.p_counter.stepped, self.p_counter.integer_values = True, True
        self._panels = ((self.cv_eul, self.p_eul), (self.cv_gyr, self.p_gyr),
                        (self.cv_acc, self.p_acc), (self.cv_integrity, self.p_integrity), (self.cv_status, self.p_bits),
                        (raw_cv, self.p_status_raw),
                        (temp_cv, self.p_temperature), (count_cv, self.p_counter))
        self._build_menu()
        self.on_source_changed()
        root.after_idle(self.restore_layout)

        self.cv3d.bind("<Configure>", self.on_3d_resize)
        for cv, panel in self._panels:
            cv.bind("<Configure>", lambda e, p=panel: self.on_plot_resize(e, p))
            panel.bind_cursor(self.inspect_time)
            panel.bind_navigation(self)
        root.bind("<Escape>", lambda e: self.on_close())
        root.bind("<space>", self._space_reset)
        self.refresh()
        self._schedule()
        if autostart:
            self.start_replay(replay) if replay else self.toggle_port()

    def _label(self, parent, text, bold=False, background="white"):
        label = self.tk.Label(parent, text=text, bg=background, fg=self.INK if bold else self.MUTED,
                              font=("Microsoft YaHei", 10, "bold" if bold else "normal"))
        label.pack(side="left", padx=(0, 8))
        return label

    def _mark_dirty(self):
        self._dirty = True

    def axis_mode(self):
        return AXIS_MODES.get(self.time_axis_var.get(), 'elapsed')

    def on_measurement_changed(self, event=None):
        for _, panel in self._plot_panels():
            panel.cur = None
        self._reset_chart_navigation()
        self._dirty = True
        self.refresh()

    def on_axis_changed(self, event=None):
        self.cfg['time_axis'] = self.axis_mode()
        self.gpimu_view._mark_dirty()
        self._dirty = True
        self.refresh()
        if self.performance_window is not None:
            self.performance_window._redraw()

    def on_source_changed(self, event=None):
        source = self.source_var.get()
        serial = source == '串口'
        self.serial_controls.pack_forget()
        self.network_controls.pack_forget()
        (self.serial_controls if serial else self.network_controls).pack(side='left', before=self.btn_open)
        self.host_label.configure(text='目标 IP' if source == 'TCP 客户端' else '本机 IP')
        if source != self._last_source:
            if source == 'TCP 客户端' and self.host_var.get() == '0.0.0.0':
                self.host_var.set('127.0.0.1')
            elif not serial and source != 'TCP 客户端' and self.host_var.get() == '127.0.0.1':
                self.host_var.set('0.0.0.0')
        self._last_source = source
        if not (self.worker and self.worker.is_alive()):
            self.btn_open.configure(text='连接串口' if serial else '连接网口' if source == 'TCP 客户端' else '开始监听')

    def _build_menu(self):
        menu = self.tk.Menu(self.root)
        file_menu = self.tk.Menu(menu, tearoff=False)
        file_menu.add_command(label="打开数据回放…", accelerator="Ctrl+O", command=self.open_replay)
        file_menu.add_command(label="CSV 开始 / 停止记录…", command=self.toggle_record)
        file_menu.add_command(label="DAT 开始 / 停止原始记录…", command=self.toggle_raw_record)
        file_menu.add_separator()
        file_menu.add_command(label="退出", command=self.on_close)
        menu.add_cascade(label="文件", menu=file_menu)
        connection = self.tk.Menu(menu, tearoff=False)
        connection.add_command(label="连接 / 断开", command=self.toggle_port)
        connection.add_command(label="刷新串口", command=self.refresh_ports)
        menu.add_cascade(label="连接", menu=connection)
        view = self.tk.Menu(menu, tearoff=False)
        view.add_command(label="监控总览", command=lambda: self.notebook.select(0))
        view.add_command(label="IMU 状态分析", command=lambda: self.notebook.select(1))
        view.add_command(label='GPIMU 分析', command=lambda: self.notebook.select(self.gpimu_page))
        view.add_command(label='命令发送', command=lambda: self.notebook.select(self.command_page))
        view.add_command(label="六轴独立分析…", command=self.open_axes_window)
        view.add_command(label="静态性能分析…", command=self.open_performance_window)
        view.add_separator()
        view.add_command(label="冻结 / 恢复显示", accelerator="F6", command=self.toggle_freeze)
        view.add_command(label='清空数据并重新显示', command=self.clear_data)
        view.add_command(label="恢复布局", command=self.restore_layout)
        view.add_command(label="诊断详情", command=self.open_details)
        menu.add_cascade(label="视图", menu=view)
        tools = self.tk.Menu(menu, tearoff=False)
        tools.add_command(label="姿态归零", command=self.on_reset)
        tools.add_command(label="重新标定", command=self.on_recal)
        menu.add_cascade(label="工具", menu=tools)
        help_menu = self.tk.Menu(menu, tearoff=False)
        help_menu.add_command(label="F-22 模型来源与许可", command=self.show_model_info)
        menu.add_cascade(label="帮助", menu=help_menu)
        self.root.configure(menu=menu)
        self.root.bind("<Control-o>", lambda e: self.open_replay())
        self.root.bind("<F6>", lambda e: self.toggle_freeze())

    def restore_layout(self):
        if self._closing:
            return
        width = self.main_split.winfo_width()
        if width > 600:
            self.main_split.sashpos(0, max(310, min(450, int(width * .29))))
        self._dirty = True

    def show_model_info(self):
        from tkinter import messagebox
        messagebox.showinfo("F-22 模型来源与许可",
                            "精细模型：AeroFab / Fabrizio Fracaroli，FlightGear / FGUK 贡献者\n"
                            "来源：https://github.com/FGMEMBERS/F-22\n"
                            "模型与纹理：GPL-2.0；原始源文件、作者说明和许可位于 assets/f22。\n"
                            "模型无担保，可按其许可再分发。模型许可与程序代码许可分别保留。", parent=self.root)

    def save_snapshot(self, path):
        """PrintWindow alone cannot capture a hardware OpenGL child; read it back explicitly."""
        import ctypes
        from ctypes import wintypes
        self.root.update_idletasks()
        frame = self.scene.gl_view
        overlay = frame.capture() if frame is not None and frame.context_created and not frame.failed else None
        hwnd = int(self.root.wm_frame(), 16)
        ok = grab_window(self.root.title(), path, hwnd)
        if ok and overlay is not None:
            from PIL import Image
            rect = wintypes.RECT()
            ctypes.windll.user32.GetWindowRect(hwnd, ctypes.byref(rect))
            with Image.open(path) as source:
                combined = source.convert("RGB")
            combined.paste(overlay, (frame.winfo_rootx() - rect.left, frame.winfo_rooty() - rect.top))
            combined.save(path)
        return ok

    def _initial_layout(self, event):
        if not self._layout_initialized and event.width > 600:
            self._layout_initialized = True
            self.restore_layout()

    def open_performance_window(self):
        if self.performance_window is None:
            from performance_window import PerformanceWindow
            self.performance_window = PerformanceWindow(self, PlotPanel)
        self.performance_window.window.deiconify()
        self.performance_window.window.lift()

    def open_axes_window(self):
        if self.axes_window is None:
            from axis_analysis import AxisAnalysisWindow
            self.axes_window = AxisAnalysisWindow(self, PlotPanel)
        self.axes_window.window.deiconify()
        self.axes_window.window.lift()
        self._dirty = True
        self.refresh()

    def _plot_panels(self):
        return self._panels + (self.axes_window.panels if self.axes_window else ())

    def toggle_freeze(self):
        self.frozen = not self.frozen
        if not self.frozen:
            self._reset_chart_navigation()
        self.freeze_text.set("恢复实时显示" if self.frozen else "冻结显示")
        self._dirty = True
        self.refresh()

    def _reset_chart_navigation(self):
        for _, panel in self._plot_panels() + self.gpimu_view.panels:
            children = (panel.time_plot, panel.counter_plot) if isinstance(panel, IntegrityPanel) else (panel,)
            for child in children:
                had_view = child.navigation.view is not None
                child.navigation.view = child.navigation.drag = None
                if had_view:
                    child._redraw_cached()

    def inspect_time(self, target):
        data = self._status_view
        if not data or not data["time"]:
            return
        times = data["time"]
        index = min(len(times) - 1, bisect_left(times, target))
        if index and abs(times[index - 1] - target) <= abs(times[index] - target):
            index -= 1
        self._cursor_time = times[index]
        angles = self.p_eul._last_ys
        attitude = "  R/P/Y %+.2f/%+.2f/%+.2f°" % tuple(y[index] for y in angles[:3]) if angles else ""
        self.cursor_var.set(self.chart_time_axis.cursor_text(times[index]) +
                           "  Status=0x%08X  Byte=0x%02X  Count=%d  T=%.2f°C%s"
                            % (data["status"][index], data["stat_byte"][index],
                               data["counter"][index], data["temperature"][index], attitude))
        for _, panel in self._plot_panels():
            panel.show_cursor(times[index])
        if self.axes_window is not None:
            self.axes_window.inspect(index)

    def _schedule(self):
        if not self._closing:
            self._after = self.root.after(max(10, int(1000 / max(1, self.cfg["refresh_hz"]))), self.tick)

    def _space_reset(self, event):
        if event.widget.winfo_class() not in ("Entry", "TEntry", "TCombobox", "Text"):
            self.on_reset()
            return "break"

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        self.port_box["values"] = ports
        # Keep an explicit --port selection, even if the device is currently absent.
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def _new_session(self, replay=False):
        if self.worker and self.worker.is_alive():
            return False
        if self.performance_window is not None and self.performance_window._job_session is self.sess:
            self.performance_window.cancel_work()
        self.sess.stop_record()
        self.sess.stop_raw_record()
        self.sess.close_analysis()
        self.cfg["auto_bias"], self.cfg["auto_accel_scale"] = ((False, False) if replay else self._live_calibration)
        self.sess = Session(self.cfg, self.formats)
        self.gpimu_view.on_session_changed()
        if self.performance_window is not None:
            self.performance_window.on_session_changed()
        self.frozen, self._display_data, self._status_view, self._cursor_time = False, None, None, None
        self._reset_chart_navigation()
        self.freeze_text.set("冻结显示")
        self.cursor_var.set("游标：在任意图表内移动鼠标，读取同步样本")
        for _, panel in self._plot_panels():
            panel.cur = None
            panel.show_cursor(None)
        if self.axes_window is not None:
            self.axes_window.reset_cursor()
        self._dirty, self._drawn = True, -1
        return True

    def toggle_port(self):
        self._pending_replay_seek = None
        if self.worker and self.worker.is_alive():
            self.worker.stop()
            self.sess.state = "stopping"
            self.refresh()
            return
        network = self.source_var.get() != '串口'
        try:
            if network:
                mode, host, port = validate_endpoint(NETWORK_MODES.get(self.source_var.get()),
                                                     self.host_var.get(), self.network_port_var.get())
            else:
                baud = int(self.baud_var.get())
                if baud <= 0:
                    raise ValueError('波特率必须为正整数')
        except ValueError as error:
            self.sess.error, self.sess.state = str(error) or "波特率必须为正整数", "error"
            self.refresh()
            return
        if network:
            self.cfg.update(source=mode, network_host=host, network_port=port)
        else:
            self.cfg.update(source='serial', port=self.port_var.get().strip(), baud=baud)
        if self._new_session():
            self.worker = NetworkWorker(self.sess, mode, host, port) if network else SerialWorker(self.sess)
            self.sess.state = "connecting"
            self.worker.start()
            self.refresh()

    def open_replay(self):
        from tkinter import filedialog
        path = filedialog.askopenfilename(parent=self.root, title="打开 RAWIMU 数据回放",
                                          filetypes=(("RAWIMU 数据", "*.dat *.bin *.log"), ("所有文件", "*.*")))
        if path:
            self.start_replay(path)

    def start_replay(self, path, speed=None, index=None, start_position=0., paused=False):
        self._pending_replay_seek = None
        try:
            speed = float(self.speed_var.get()) if speed is None else float(speed)
            if not math.isfinite(speed) or speed < 0:
                raise ValueError()
        except ValueError:
            self.sess.error, self.sess.state = "回放速度必须为非负有限数值", "error"
            self.refresh()
            return
        if self._new_session(replay=True):
            self.replay = path
            self.cfg["replay_speed"] = speed
            self.speed_var.set("%g" % speed)
            self.worker = ReplayWorker(self.sess, path, speed, index, start_position, paused)
            self.sess.state = "connecting"
            self.worker.start()
            self.refresh()

    def toggle_replay_pause(self):
        worker = self.worker
        if not isinstance(worker, ReplayWorker):
            return
        if not worker.is_alive():
            self.seek_replay(0., resume=True)
        else:
            worker.set_paused(not worker.timeline()['paused'])
        self.refresh()

    def seek_replay(self, position, resume=False):
        worker = self.worker
        if not isinstance(worker, ReplayWorker):
            return
        if self.sess.recording:
            self.sess.status = 'CSV录制中禁止回放跳转；请先停止CSV记录。仍可暂停/继续回放。'
            return
        info = worker.timeline()
        if not info['ready'] or not math.isfinite(position):
            return
        position = max(0., min(info['duration'], position))
        self._pending_replay_seek = (worker.path, worker.speed, worker.index, position,
                                    False if resume else info['paused'] or not worker.is_alive())
        worker.stop()
        self.sess.state = 'stopping'
        self.sess.status = '准备跳转到 %.3f s；正在停止当前回放…' % position
        self.tick_replay_seek()

    def tick_replay_seek(self):
        if self._closing or self._pending_replay_seek is None or (self.worker and self.worker.is_alive()):
            return
        path, speed, index, position, paused = self._pending_replay_seek
        self._pending_replay_seek = None
        self.start_replay(path, speed, index, position, paused)

    def on_imu(self):
        if self.sess.set_model(self.models, self.imu_var.get()):
            self._dirty = True

    def on_tau(self):
        with self.sess.lock:
            self.cfg["level_tau"] = float(self.tau_var.get())

    def on_mode(self):
        with self.sess.lock:
            self.cfg["mode"] = self.mode_var.get()
        self._dirty = True

    def on_window(self):
        self.cfg["window_s"] = float(self.window_var.get())
        self._dirty = True

    def on_reset(self):
        self.sess.reset_attitude()
        self._dirty = True
        self.refresh()

    def clear_data(self):
        self.sess.clear_data()
        self.frozen = False
        self.freeze_text.set('冻结显示')
        self._display_data = self._status_view = self._cursor_time = None
        self._drawn, self._integrity_drawn, self._dirty = -1, None, True
        for _, panel in self._plot_panels():
            children = (panel.time_plot, panel.counter_plot) if isinstance(panel, IntegrityPanel) else (panel,)
            for child in children:
                child.clear_plot()
        self._reset_chart_navigation()
        for _, panel in self._plot_panels():
            panel.cur = None
            panel.show_cursor(None)
        self.cursor_var.set('数据已清空：等待后续样本，连接和文件录制继续')
        if self.axes_window is not None:
            self.axes_window.reset_cursor()
        self.gpimu_view.on_session_changed()
        if self.performance_window is not None:
            self.performance_window.on_session_changed()
        self.refresh()
        # Hidden pages must not retain old cached traces when mapped later.
        for _, panel in self._plot_panels():
            children = (panel.time_plot, panel.counter_plot) if isinstance(panel, IntegrityPanel) else (panel,)
            for child in children:
                child.render_enabled = True
                child._redraw_cached()

    def on_recal(self):
        self.sess.reset_calibration()
        self._dirty = True

    def _csv_replay_ready(self):
        if not isinstance(self.worker, ReplayWorker):
            return True
        info = self.worker.timeline()
        return self._pending_replay_seek is None and info['ready'] and not info['seeking']

    def toggle_record(self):
        from tkinter import filedialog
        if self.sess.recording:
            self.sess.stop_record()
        else:
            if not self._csv_replay_ready():
                self.sess.record_error = '时间轴准备或定位重算中，请完成后再开始CSV录制。'
                self.refresh()
                return
            sess = self.sess
            name = time.strftime("rawimu_%Y%m%d_%H%M%S.csv")
            path = filedialog.asksaveasfilename(parent=self.root, title="保存 IMU 记录", initialfile=name,
                                               defaultextension=".csv", filetypes=(("CSV 记录", "*.csv"),))
            if path:
                if self.sess is sess and self._csv_replay_ready():
                    sess.start_record(path)
                else:
                    self.sess.record_error = '数据源已变化或正在定位，请重新选择CSV记录。'
        self.refresh()

    def toggle_raw_record(self):
        from tkinter import filedialog
        sess = self.sess
        if sess.raw_recording:
            sess.stop_raw_record()
        elif not self._live_ready(sess):
            sess.raw_error = "请先连接串口或网口，再开始原始 DAT 记录"
        else:
            path = filedialog.asksaveasfilename(
                parent=self.root, title="保存接收的原始数据", initialfile=time.strftime("rawimu_%Y%m%d_%H%M%S.dat"),
                defaultextension=".dat", filetypes=(("原始数据 DAT", "*.dat"),))
            if path:
                with sess.raw_lock:
                    if self._live_ready(sess):
                        sess.start_raw_record(path)
                    else:
                        sess.raw_error = "连接已断开，未开始 DAT 记录"
        self.refresh()

    def _serial_ready(self, sess):
        return (self.sess is sess and isinstance(self.worker, SerialWorker)
                and self.worker.is_alive() and sess.running and sess.state == "serial"
                and not self.worker.stop_event.is_set())

    def _live_ready(self, sess):
        return (self.sess is sess and isinstance(self.worker, (SerialWorker, NetworkWorker))
                and self.worker.is_alive() and sess.running and sess.state in ('serial', 'network', 'listening')
                and not self.worker.stop_event.is_set())

    def open_details(self):
        if self.details_window is not None and self.details_window.winfo_exists():
            self.details_window.lift()
            return
        from tkinter.scrolledtext import ScrolledText
        self.details_window = self.tk.Toplevel(self.root)
        self.details_window.title("RAWIMU · 诊断详情")
        self.details_window.geometry("660x520")
        self.details_text = ScrolledText(self.details_window, font=("Consolas", 11), padx=15, pady=12,
                                         bg="white", fg=self.INK, wrap="word")
        self.details_text.pack(fill="both", expand=True)
        self._details_last = None
        self.refresh()

    def _update_details(self, att):
        if self.details_window is None or not self.details_window.winfo_exists():
            return
        sess, parser = self.sess, self.sess.parser
        smp = sess.smp or {}
        lines = [sess.status or "未连接", "",
                 "3D model   : " + (self.scene.gl_view.renderer if self.scene.gl_view is not None and self.scene.gl_view.context_created else "Canvas compatibility"),
                 "3D detail  : " + (self.scene.gpu_error or "—"),
                 "Format     : " + fmt_text(parser.fmt_counts.copy()),
                 "Frames     : %d (binary) / %d (ASCII)" % (parser.n_frame, parser.n_text),
                 "IMU        : %d" % sess.n_imu,
                 "CRC errors : %d (binary) / %d (ASCII)" % (parser.n_crc, parser.n_text_crc_bad),
                 "ShortHdr   : %d" % parser.n_short,
                 "SkipBytes  : %d" % parser.n_drop,
                 "Bytes      : %.1f kB" % (parser.n_bytes / 1024.),
                 "Status     : 0x%08X" % smp.get("status", 0),
                 "StatByte   : 0x%02X" % smp.get("stat_byte", 0),
                 "Faults     : " + ((", ".join(imu_fault_names(smp["status"])) or "无异常") if smp else "—"),
                 "Counter    : %d" % smp.get("counter", 0),
                 "Bias dps   : %.5f %.5f %.5f" % tuple(att.bias),
                 "IntAngle   : %.2f %.2f %.2f deg" % tuple(att.ang_acc),
                 "", "记录路径   : " + (sess.csv_name or "—"),
                 "DAT path   : " + (sess.raw_name or "—"),
                 "DAT bytes  : %d" % sess.raw_bytes,
                 "", "标定日志："] + att.log[-8:]
        text = "\n".join(lines)
        if text != self._details_last:
            self.details_text.configure(state="normal")
            self.details_text.delete("1.0", "end")
            self.details_text.insert("1.0", text)
            self.details_text.configure(state="disabled")
            self._details_last = text

    def reset_view(self):
        self.cfg["view_az"], self.cfg["view_el"] = DEFAULT_CFG["view_az"], DEFAULT_CFG["view_el"]
        self.scene.resize(self.cv3d.winfo_width(), self.cv3d.winfo_height())
        self._dirty = True

    def on_3d_resize(self, event):
        self.scene.resize(event.width, event.height)
        self._dirty = True

    def on_plot_resize(self, event, panel):
        panel.resize(event.width, event.height)
        self._dirty = True

    def on_angles_resize(self, event):
        available = max(30, event.width / 3. - 22)
        for size in (19, 18, 16, 14, 12):
            self._angle_font.configure(size=size)
            if self._angle_font.measure("-180.00") <= available:
                break

    def on_close(self):
        if self._closing:
            return
        self._closing = True
        self._pending_replay_seek = None
        if self.performance_window is not None:
            self.performance_window.cancel_work()
        if self._after:
            self.root.after_cancel(self._after)
        if self.worker:
            self.worker.stop()
        self._close_deadline = time.monotonic() + 2.
        self._finish_close()

    def _finish_close(self):
        if self.worker and self.worker.is_alive() and time.monotonic() < self._close_deadline:
            self.root.after(25, self._finish_close)
        else:
            self.sess.stop_record()
            self.sess.stop_raw_record()
            self.sess.close_analysis()
            self.root.destroy()

    def tick(self):
        if self._closing:
            return
        try:
            self.tick_replay_seek()
            self.refresh()
        except Exception:
            traceback.print_exc()
        self._schedule()

    def refresh(self):
        if self._closing:
            return
        self.command_view.refresh()
        if self.performance_window is not None:
            self.performance_window.refresh_source()
        sess = self.sess
        alive = bool(self.worker and self.worker.is_alive())
        state = sess.state
        if isinstance(self.worker, ReplayWorker) and self.worker.index is not None and self.worker.index.gpimu_only and self._gp_auto_selected_worker is not self.worker:
            self._gp_auto_selected_worker = self.worker
            self.notebook.select(self.gpimu_page)
        self.replay_timeline.render(self.worker if isinstance(self.worker, ReplayWorker) else None, sess.recording)
        labels = {"idle": "未连接", "connecting": "正在连接", "stopping": "正在停止",
                  "disconnected": "已断开", "finished": "回放结束", "error": "连接 / 数据错误",
                  "indexing": "建立回放时间轴", "seeking": "回放定位中", "paused": "回放已暂停",
                  "listening": "网口监听中 · 等待设备"}
        label = labels.get(state, "")
        if state in ("serial", "network"):
            last_wall = max(sess.last_sample_wall or 0., sess.gpimu.last_wall or 0.)
            stale = not last_wall or time.monotonic() - last_wall > 1.5
            label = "已连接 · 等待 IMU 数据" if stale else "实时接收中"
        elif state == "replay":
            label = "回放中 · %g 倍" % self.cfg["replay_speed"]
        self.state_var.set(label)
        color = "#b44747" if sess.error else "#20956b" if state in ("serial", "network", "replay") else "#51647e"
        self.source_badge.configure(bg=color)
        idle_text = ('连接串口' if self.source_var.get() == '串口' else
                     '连接网口' if self.source_var.get() == 'TCP 客户端' else '开始监听')
        self.btn_open.configure(text=("停止回放" if isinstance(self.worker, ReplayWorker) else "断开连接") if alive else idle_text,
                                state="disabled" if state == "stopping" and alive else "normal")
        for widget in (self.port_box, self.baud_box, self.btn_ports, self.btn_replay):
            widget.configure(state="disabled" if alive else "normal")
        self.source_box.configure(state='disabled' if alive else 'readonly')
        for widget in (self.host_entry, self.network_port_entry):
            widget.configure(state='disabled' if alive else 'normal')
        self.speed_box.configure(state="disabled" if alive else "readonly")
        self.btn_rec.configure(text="停止 CSV" if sess.recording else "开始 CSV…",
                               state='normal' if sess.recording or self._csv_replay_ready() else 'disabled')
        serial_active = self._live_ready(sess)
        self.btn_raw.configure(text="停止 DAT" if sess.raw_recording else "保存原始 DAT…",
                               state="normal" if sess.raw_recording or serial_active else "disabled")
        recording = ("CSV 记录中：" + os.path.basename(sess.csv_name)) if sess.recording else ("CSV 已保存" if sess.csv_name else "CSV 未开始")
        raw_status = ("DAT 记录中" if sess.raw_recording else "DAT 保存失败" if sess.raw_error else "DAT 已保存" if sess.raw_name else "DAT 未开始")
        if sess.raw_name:
            raw_status += "：%s (%d B)" % (os.path.basename(sess.raw_name), sess.raw_bytes)
        self.record_var.set(recording + "  |  " + raw_status)
        self.record_label.configure(fg="#b43b3b" if sess.raw_error else "#17805c" if sess.recording or sess.raw_recording else self.MUTED)
        message = sess.error or sess.raw_error or sess.record_error or sess.status or "选择串口 / TCP / UDP 数据源，或打开 .dat 文件回放。"
        if self.frozen:
            message = "[显示已冻结，采样 / 记录继续] " + message
        self.current_group.configure(text="冻结样本 · IMU 字段" if self.frozen else "最新样本 · IMU 字段")
        self.message_var.set(message)
        self.message_label.configure(fg="#b43b3b" if sess.error or sess.raw_error or sess.record_error else self.MUTED)
        with sess.lock:
            att = copy.deepcopy(sess.att)
            count, temp, rate = sess.n_imu, sess.temp, sess.rate_hz
            mean_acc = sess.acc_norm_sum / max(1, sess.acc_norm_n)
            mean_dt = 1000 * sess.dt_sum / max(1, sess.dt_n)
        temp_text = "%.2f °C" % temp if math.isfinite(temp) else "—"
        self.metrics_var.set("采样率 %.1f Hz    间隔 %.2f ms    温度 %s    |a| %.3f m/s²    IMU 帧 %d    CRC 错误 %d"
                             % (rate, mean_dt, temp_text, mean_acc, count,
                                sess.integrity_snapshot()['crc_errors']))
        if sess.gpimu.total:
            self.metrics_var.set(self.metrics_var.get() + '    GPIMU 帧 %d' % sess.gpimu.total)
        need = max(10, int(round(self.cfg["bias_sec"] * self.cfg["freq"])))
        enabled = self.cfg["auto_bias"] or self.cfg["auto_accel_scale"]
        progress = min(100., 100. * min(att.bias_n if self.cfg["auto_bias"] else need,
                                       att.acc_n if self.cfg["auto_accel_scale"] else need) / need)
        self.cal_var.set("标定：回放时关闭" if not enabled else "标定完成" if progress >= 100 else
                         "标定：请保持静止 %d%%" % progress if count else "标定：等待数据")
        self.cal_progress["value"] = progress if enabled else 0
        self._update_details(att)
        integrity_signature = (count, sess.parser.n_crc, sess.parser.n_text_crc_bad, sess.gpimu.total,
                               sess.parser.n_gpimu_crc_bad, sess.parser.n_gpimu_malformed)
        if ((count == self._drawn and integrity_signature == self._integrity_drawn) or self.frozen) and not self._dirty:
            return
        if not self.frozen or self._display_data is None:
            with sess.lock:
                self._display_data = (sess.snapshot(), sess.status_snapshot(), sess.integrity_snapshot(), sess.gpimu_snapshot())
        (t, ch, roll_a, pitch_a, att, smp), status_data, integrity_data, gpimu_data = self._display_data
        self._drawn, self._dirty = count, False
        self._integrity_drawn = integrity_signature
        angles = [math.degrees(v) for v in (att.roll, att.pitch, att.yaw)]
        for var, value in zip(self.angle_vars, angles):
            var.set("%+.2f" % value if t else "—")
        self.scene.update(quat_to_dcm(att.q), *angles,
                          text="X 前 / Y 右 / Z 下" if t else "等待数据 · 姿态示意")
        start = 0
        if t:
            start = max(0, bisect_left(t, t[-1] - self.cfg["window_s"]) - 1)
        times = t[start:]
        self._status_view = {key: values[start:] for key, values in status_data.items()}
        status = self._status_view
        self.chart_time_axis = TimeAxis(self.axis_mode(), times, status['sow'], status['week'])
        self.status_word_var.set("0x%08X" % status["status"][-1] if times else "—")
        self.status_byte_var.set("0x%02X" % status["stat_byte"][-1] if times else "—")
        self.counter_var.set(str(status["counter"][-1]) if times else "—")
        self.status_temp_var.set("%.2f °C" % status["temperature"][-1] if times else "—")
        window = self.cfg["window_s"]
        time_range = (max(0., t[-1] - window), max(window, t[-1])) if t else (0., window)
        for _, panel in self._panels:
            panel.time_range = time_range
            panel.time_axis = self.chart_time_axis
            panel.render_enabled = bool(panel.c.winfo_ismapped())
        self.p_eul.update(times, [ch[0][start:], ch[1][start:], ch[2][start:], roll_a[start:], pitch_a[start:]])
        raw_view = self.measurement_var.get() == '原始物理量'
        display_channels = ch[:3] + [status_data[key] for key in ('gx','gy','gz','ax','ay','az')] if raw_view else ch
        self.p_gyr.title = '角速度 · 原始物理量' if raw_view else '角速度 · 解算去零偏'
        self.p_acc.title = '加速度 · 原始物理量' if raw_view else '加速度 · 解算标度'
        self.p_gyr.update(times, [display_channels[k][start:] for k in (3, 4, 5)])
        self.p_acc.update(times, [display_channels[k][start:] for k in (6, 7, 8)])
        self.p_integrity.update(times, dict(integrity_data, time_interval_ms=status['time_interval_ms'],
                                          counter_interval=status['counter_interval']))
        self.p_bits.update(times, [status["status"]])
        self.p_status_raw.update(times, [status["status"]])
        self.p_temperature.update(times, [status["temperature"]])
        self.p_counter.update(times, [status["counter"]])
        self.gpimu_view.update(gpimu_data, window)
        if self.axes_window is not None:
            self.axes_window.update(t, display_channels, status, start, time_range)
        if self._cursor_time is not None:
            self.inspect_time(self._cursor_time)


# ---------------------------------------------------------------- 窗口截图
def grab_window(title, out_path, hwnd=None):
    """把窗口自身渲染成图片（PrintWindow，锁屏/遮挡也能取到）"""
    import ctypes
    from ctypes import wintypes
    user32 = ctypes.windll.user32
    gdi32 = ctypes.windll.gdi32
    if hwnd is None:
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
    previous_bitmap = gdi32.SelectObject(mdc, bmp)
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
    gdi32.SelectObject(mdc, previous_bitmap)
    gdi32.DeleteObject(bmp)
    gdi32.DeleteDC(mdc)
    user32.ReleaseDC(hwnd, hdc)
    return ok

# ---------------------------------------------------------------- 自检 / 无界面
def run_selftest(cfg, path, truth=None, dump_unknown=False):
    import csv
    from contextlib import ExitStack

    cfg = dict(cfg, auto_bias=False, auto_accel_scale=False)
    sess = Session(cfg)
    errors = [0.0, 0.0, 0.0]
    compared = [0]
    t0 = time.monotonic()
    try:
        with ExitStack() as stack:
            source = stack.enter_context(open(path, "rb"))
            if truth:
                truth_file = stack.enter_context(open(truth, newline="", encoding="utf-8-sig"))
                rows = csv.reader(truth_file)
                next(rows, None)  # header
                initial = next(rows, None)  # attitude immediately before the first integration
                if initial is None or len(initial) < 4:
                    raise ValueError("真值文件缺少初始姿态和逐帧数据")
                initial_values = [float(value) for value in initial[:4]]
                if not all(math.isfinite(value) for value in initial_values):
                    raise ValueError("真值初始行包含非有限数值")
                truth_time = [initial_values[0]]
                missing_final = [False]

                def compare(spec, body):
                    before = sess.n_imu
                    sess._on_frame(spec, body)
                    if sess.n_imu == before:
                        return
                    if missing_final[0]:
                        raise ValueError("真值数据不足，缺少多帧对应姿态")
                    row = next(rows, None)
                    if row is None:
                        # Existing fixture stores t=0..19.99 while 2000 integrations end at 20s.
                        missing_final[0] = True
                        return
                    if len(row) < 4:
                        raise ValueError("真值数据列不足，IMU 帧 %d 无对应姿态" % sess.n_imu)
                    values = [float(v) for v in row[:4]]
                    if not all(math.isfinite(v) for v in values):
                        raise ValueError("真值包含非有限数值")
                    if values[0] <= truth_time[0]:
                        raise ValueError("真值时间必须严格递增")
                    if abs(values[0] - initial_values[0] - sess.dt_sum) > 0.0005:
                        raise ValueError("真值时间与 IMU 帧 %d 不匹配" % sess.n_imu)
                    truth_time[0] = values[0]
                    for j, angle in enumerate((sess.att.roll, sess.att.pitch, sess.att.yaw)):
                        delta = (math.degrees(angle) - values[j + 1] + 180) % 360 - 180
                        errors[j] += abs(delta)
                    compared[0] += 1

                # Binary callback exceptions propagate; ASCII parser normally isolates callbacks.
                # Store failures explicitly so a malformed truth row cannot be swallowed there.
                compare_errors = []
                def checked_compare(spec, body):
                    if compare_errors:
                        return
                    try:
                        compare(spec, body)
                    except (ValueError, TypeError) as exc:
                        compare_errors.append(exc)
                sess.parser.on_frame = checked_compare
            while True:
                chunk = source.read(65536)
                if not chunk:
                    break
                sess.parser.feed(chunk)
                if truth and compare_errors:
                    raise compare_errors[0]
            if truth and next(rows, None) is not None:
                raise ValueError("真值行数多于 IMU 帧，无法逐帧对应")
    except (OSError, ValueError, csv.Error) as exc:
        print("自检失败: %s" % exc)
        return 2
    dt = time.monotonic() - t0
    parser = sess.parser
    print("二进制帧 %d（其它 %d / 短头 %d / CRC错 %d）  ASCII 行 %d  IMU 帧 %d  丢弃字节 %d"
          % (parser.n_frame, parser.n_other, parser.n_short, parser.n_crc,
             parser.n_text, sess.n_imu, parser.n_drop))
    print("报文格式统计: %s   ASCII 行 %d（CRC错 %d）"
          % (fmt_text(parser.fmt_counts), parser.n_text, parser.n_text_crc_bad))
    if parser.text_unknown:
        print("未识别 ASCII 前缀: %s" % ", ".join(parser.text_unknown))
    if dump_unknown:
        for mid, hlen, length, hx in parser.unknown:
            print("  id=0x%04X(%d) 头长=%d 体长=%d 前56字节=%s" % (mid, mid, hlen, length, hx))
    elif parser.unknown:
        print("未识别 ID: %s" % ", ".join("0x%04X" % u[0] for u in parser.unknown))
    print("解析耗时 %.2f s（%.1f MB/s）" % (dt, parser.n_bytes / 1048576.0 / max(1e-6, dt)))
    if not sess.n_imu:
        print("自检失败: 未解析到有效 IMU 数据")
        return 2
    print("时长 %.1f s  平均帧间隔 %.3f ms  温度 %.2f C  平均|a| %.3f m/s2"
          % (sess.t_last, 1000 * sess.dt_sum / max(1, sess.dt_n), sess.temp,
             sess.acc_norm_sum / max(1, sess.acc_norm_n)))
    att = sess.att
    print("末姿态 roll %.2f pitch %.2f yaw %.2f" % tuple(
        math.degrees(v) for v in (att.roll, att.pitch, att.yaw)))
    print("累计转角 %.1f %.1f %.1f  deg" % tuple(att.ang_acc))
    if truth:
        if not compared[0]:
            print("自检失败: 没有可比较的真值")
            return 2
        if missing_final[0]:
            print("真值仅缺少末帧，沿用样例初始行对齐约定；比较其余全部帧")
        print("对比真值 %d 帧 meanerr roll=%.5f pitch=%.5f yaw=%.5f deg" % (
            compared[0], errors[0] / compared[0], errors[1] / compared[0], errors[2] / compared[0]))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="RAWIMU 实时姿态显示（支持自定义 IMU 型号与报文格式）")
    ap.add_argument("--port")
    ap.add_argument('--source', choices=('serial', 'tcp_client', 'tcp_server', 'udp'), help='实时数据源')
    ap.add_argument('--host', help='IPv4：TCP客户端填设备IP；TCP服务端/UDP填本机监听IP')
    ap.add_argument('--network-port', type=int, help='网口端口，1～65535')
    ap.add_argument('--time-axis', choices=('elapsed', 'sow'), help='时序图横轴：相对时间 / GPS周内秒')
    ap.add_argument("--baud", type=int)
    ap.add_argument("--replay")
    ap.add_argument("--speed", type=float)
    ap.add_argument("--imu", default=None, help="IMU 型号名（见 --list-models）")
    ap.add_argument("--mode", choices=("complementary", "gyro"))
    ap.add_argument("--tau", type=float, help="重力修正时间常数(秒)，越小跟得越快")
    ap.add_argument("--config")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--truth")
    ap.add_argument("--no-autostart", action="store_true")
    ap.add_argument("--snapshot", help="截图保存路径（调试/留证）")
    ap.add_argument("--snapshot-delay", type=float, default=3.0)
    ap.add_argument("--renderer", choices=("auto", "canvas"), help="auto 使用高精度 OpenGL 模型；canvas 使用兼容模式")
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
        print("%-12s %-9s %-6s %-6s %s" % ("名称", "标识", "头长", "体长", "备注"))
        for name, s in sorted(formats.specs.items()):
            if s.get("kind") == "ascii":
                print("%-12s %-9s %-6s %-6s %s" % (
                    name, "ASCII", "-", "-", s.get("note", "")))
            else:
                print("%-12s 0x%04X    %-6d %-6d %s" % (
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
    if a.source:
        cfg['source'] = a.source
        if a.source in ('tcp_server', 'udp') and a.host is None:
            cfg['network_host'] = '0.0.0.0'
    if a.host is not None:
        cfg['network_host'] = a.host
    if a.network_port is not None:
        cfg['network_port'] = a.network_port
    if a.time_axis is not None:
        cfg['time_axis'] = a.time_axis
    if cfg.get('source', 'serial') != 'serial':
        try:
            validate_endpoint(cfg['source'], cfg['network_host'], cfg['network_port'])
        except ValueError as error:
            ap.error(str(error))
    if a.mode:
        cfg["mode"] = a.mode
    if a.tau:
        cfg["level_tau"] = a.tau
    if a.speed is not None:
        if not math.isfinite(a.speed) or a.speed < 0:
            ap.error("--speed 必须为非负有限数值；0 表示极速回放")
        cfg["replay_speed"] = a.speed
    if a.renderer:
        cfg["renderer"] = a.renderer

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
            root.update_idletasks()
            ok = app.save_snapshot(a.snapshot)
            print("snapshot", "ok" if ok else "failed", a.snapshot)
            root.after(200, app.on_close)
        root.after(int(a.snapshot_delay * 1000), _shot)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
