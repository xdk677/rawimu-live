# IMU Status Monitor Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans for inline implementation with test-first behavior changes and one final independent review.

**Goal:** 状态字段完整绘图，并以工程监测软件的布局和交互统一界面。

**Architecture:** 保持 Python 3.7 / Tkinter 与现有解析、姿态、CSV 约定。Session 增加同步状态历史，状态位采用数字阶梯波形；Notebook 区分监测总览和状态分析，所有图使用同一时间窗和同步游标。

**Authorization:** 用户要求直接优化，前轮已选择直接实施；本轮说明没有指定参考软件。采用既有架构与工程监测惯例，不声称复制某一产品。保留前轮未提交改动和未知来源图片。

## Constraints and decisions

- IMU_Status 位 0–7 为计数器，8–15 为厂商定义状态字节，16–31 为有符号温度原始值。未定义位不解释为故障。
- 原始 32 位状态、状态字节、温度和计数器均保留历史并可绘图；已知协议布局以 docs/protocol.md 为准。
- 总览 2×2 图表包含姿态/角速度/加速度/状态位；状态分析 2×2 图表包含原始状态/状态字节/温度/计数器。
- 分隔条可拖动；视窗时间一致；游标显示最近样本时间、原始状态和各通道值。冻结显示不停止采样或记录。
- 现有 CLI、六元组 Session.snapshot() 和 CSV 列不变。新状态历史随型号频率扩容并随数据源重置。
- 数字波形采用阶梯而非斜线；按时间像素分桶保留高低状态，稀疏采样、计数器回绕和数据源切换不得错位。

## Tasks

- [x] 数据：增加状态历史和 status_snapshot()，回归验证字段、时间对齐、缓冲扩容和数字阶梯/回绕。
- [x] 绘图：扩展 PlotPanel 阶梯/十六进制坐标、固定时间窗和游标；StatusPanel 显示 8 条状态位数字波形及当前十六进制状态。
- [x] 界面：统一工具栏与菜单、可拖动分区、总览/状态分析页、当前字段读数、冻结显示和同步游标；真实 Tk 验证空闲/回放/缩放。
- [x] 交付：更新 README 和协议显示说明、生成状态变化演示截图；全部回归、样例自检、独立审查。

## Validation

`python -m unittest discover -s tests -v`；二进制/ASCII 既有自检；`python -m py_compile src/rawimu_live.py`；`git diff --check`。状态演示截图使用明确标记的合成数据，不将合成状态解释为设备实测故障。

## Completion evidence

- 35 项测试通过，包含真实 Tk、状态历史对齐、缓冲扩容、有符号温度、计数回绕、游标读取、冻结显示与窄窗口数字完整显示。
- 二进制 2000 帧 / ASCII 797 帧保持基线，均无 CRC 错误；既有真值约定下比较 1999 帧，三轴平均误差均打印为 0.00000 度。
- 独立审查发现的数字波形左边界保持问题、旧源游标残留均先写失败回归，再修复并通过审查复验。
- 截图已检查 1600×960 与 1100×720 布局；README 中保留总览和状态分析两张合成状态演示截图。
- 语法编译、diff whitespace 检查通过；未连接串口硬件、未安装新依赖、未提交或推送。保留此前全部优化与未知来源图片。
