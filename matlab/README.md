# MATLAB 版（与本项目 Python 版功能等价）

用于已经在 MATLAB 环境里做后续分析的同学；算法、协议、量纲与 `src/rawimu_live.py` 完全一致。

| 文件 | 作用 |
|---|---|
| `rawimu_live.m` | 实时解析 + 三维姿态显示（串口或回放 `.dat`） |
| `rawimu_make_testfile.m` | 生成「已知姿态」的 RAWIMU 二进制流，用于无硬件自检 |
| `novatel_crc32.m` | 设备用的 CRC32（初值 0、末尾不取反） |

## 用法

```matlab
cd matlab
rawimu_live('Port','COM49','Baud',460800)                       % 实时
rawimu_live('Source','file','File','..\examples\rawimu_test.dat')% 回放
rawimu_make_testfile('rawimu_test.dat', 20);                     % 造测试数据
```

运行中：`ESC` 退出、`r` 姿态归零、`z` 重新估零偏。

可配置项见 `rawimu_live.m` 顶部的 `default_config()`（端口、波特率、IMU 预设、
视角、`AttitudeMode = 'gyro' | 'complementary'` 等）。

## 与 Python 版的对照

- 同一份 `examples/rawimu_test.dat`，两边末姿态、累计转角、|a|、温度完全一致，
  对真值误差均为 `0.00000°`；
- Python 版的 `imu_models.json` 对应 MATLAB 里的 `imu_preset()` 函数，加型号需改函数
  （Python 版是改 JSON，不用动代码）；
- 三维场景：MATLAB 用 `patch` + 正交相机，Python 用 Tk Canvas 自绘正交投影，观感略有差异。

## 注意

- 需要 MATLAB R2019b 及以上（`tiledlayout`、`exportgraphics`）；
- 串口用的是 Instrument Control Toolbox 的 `serialport`；
- 依赖 `novatel_crc32.m`，三个文件放同一目录（或在 path 上）。
