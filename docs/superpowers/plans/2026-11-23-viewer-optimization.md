# Viewer Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 修复后台问题并交付直观、流畅的 RAWIMU 桌面界面。

**Architecture:** 保留现有单文件入口与协议/姿态实现，以同步的 Session 和可停止 Worker 管理数据，Tk 主线程负责所有控件。图形只绘制视窗所需数据，数据计算与记录不降采样。

**Tech Stack:** Python 3.7、Tkinter/Tk 8.6、pyserial、标准库 unittest/zlib。

**Spec:** `docs/superpowers/specs/2026-11-23-viewer-optimization-design.md`

## Global Constraints

- 保留 Python 3.7 兼容，不增加第三方运行依赖。
- 保留 CLI、JSON 格式配置、CSV 列和打包入口；不改变姿态融合算法。
- 用户已选择直接实施，当前目录直接修改，不创建需额外确认的 worktree；不替用户提交或发布。

## Review Focus

- 停止请求发生在串口打开期间：线程退出并关闭句柄，不重新激活。
- 记录文件写入失败：展示记录错误，采样继续。
- 空文件/真值不足/超过 6000 帧：明确结果，流式误差不能错位。
- NaN 断线/±180 度跳变/瞬时峰值：减点不歪曲曲线。
- 无数据和较小窗口：状态、控件及占位显示可用。

### Task 1: CRC、可停止数据源、同步 Session

**Files:** Modify `src/rawimu_live.py`; Create `tests/test_rawimu_live.py`.

**Interfaces:** `SerialWorker.stop()`、`ReplayWorker.stop()` 使用事件；`Session.snapshot()` 保留六元组但姿态为独立副本；Session 提供同步 reset 和 model 操作。

- [x] 写 CRC 已知向量/随机分块回归、回放模拟时钟及停止、CSV 换行/失败隔离/关闭同步、snapshot 隔离测试。
- [x] 运行 `python -m unittest discover -s tests -v` 确认新行为在原代码上失败。
- [x] 实现 zlib CRC、可中断 monotonic 回放与 Worker 生命周期；同步姿态和记录操作。
- [x] 运行上述测试及两个现有样例自检。

### Task 2: 流式自检与图形数据

**Files:** Modify `src/rawimu_live.py`, `tests/test_rawimu_live.py`.

**Interfaces:** `run_selftest()` 返回有效退出码，逐帧累计真值误差；`plot_segments(t, y, width, wrap=False)` 返回按时间排序的有限曲线段，最多每像素桶四个极值/边界点。

- [x] 写长文件/空文件/短真值测试，以及峰值、NaN 间隙和角度跳变测试；运行确认失败。
- [x] 实现分块自检、真值流式比较；实现按像素保极值减点。
- [x] 运行测试和样例回归确认数值保持一致。

### Task 3: Tkinter 布局与直观状态

**Files:** Modify `src/rawimu_live.py`; Create `tests/test_gui.py`.

**Interfaces:** App 保留串口操作与 CLI 自动回放；增加文件回放、速度、窗口长度、姿态数值与状态标签，PlotPanel 使用图形数据接口。

- [x] 增加实际 Tk 的空闲/回放/结束/缩放/归零/关闭烟测，确认旧界面缺少功能。
- [x] 两排控件、左侧姿态与三维、右侧三组曲线、持续状态区；中文状态和图例；主线程非阻塞停止。
- [x] 缓存网格和曲线对象；无新帧时跳过重绘但仍更新生命周期状态；重绘响应 resize。
- [x] 运行实际 Tk 烟测，生成并检查截图，按发现修正。

### Task 4: 文档与最终验证

**Files:** Modify `README.md`; add optimized UI screenshot under `docs/images/`.

- [x] 更新操作和验证说明、修正已知限制中的 ASCII 描述。
- [x] 全部 unittest、二进制真值、ASCII 自检、语法编译、`git diff --check`。
- [x] 检查整文件解析与 CRC 微基准，独立代码审查并处理问题。
- [x] 更新本计划的实际完成状态；报告截图、测试和硬件验证边界。
