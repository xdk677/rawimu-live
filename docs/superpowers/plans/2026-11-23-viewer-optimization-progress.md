# Progress — 2026-11-23-viewer-optimization.md

- 用户选择“按这个方案直接实施”，跳过后续设计讨论。
- Ruling: 在用户指定目录的 `optimize/viewer-performance-ui` 分支工作，保留未提交改动 — 直接实施已授权，不再增加 worktree 确认 — 原始 master 提交保持不变。
- Task 1: complete — CRC 标准库实现、可停止 Worker、Session 同步、独立快照与记录错误隔离。已观察原版回归失败，再修复并通过。
- Task 2: complete — 大文件流式自检/真值对齐、空文件失败、按时间像素分桶保极值和断线。8000 帧独立已知航向真值通过。
- Task 3: complete — 分组控件、姿态卡片、三张趋势图、独立图例/当前值、状态与标定/记录提示、诊断窗口。实际 Tk 烟测及 1480×900 / 1100×720 截图检查通过。
- Task 4: complete — README、截图和可复现性能工具已更新；最终命令见下。
- Review: 独立审查发现真值时间未校验、高频型号切换未扩容，均先补失败测试再修复；二次检查发现不均匀采样时点数分桶失真，改为时间/像素分桶并补回归。审查者已确认三项修复。
- 最终验证：26 项 unittest 全通过；二进制 2000 帧 / ASCII 797 帧，均无 CRC 错误；二进制 1999 帧真值比较三轴平均误差 0.00000 度；语法编译与 diff whitespace 检查通过。
- 截图验证补充：复现同名窗口截错对象的问题后，修复 grab_window 忽略显式 hwnd 的错误；同名双窗口回归通过，交付截图已重新生成并查看。
- 保留未知来源的根目录 `live_new.png`、`new_ui.png`，未修改或删除。
- Ruling: 保留现有样例的初始行偏移和最后一帧缺失真值约定，但明确输出比较帧数 — 兼容仓库自检数据 — 不将末帧未比较伪装成全帧比较。
- Ruling: 小数据绘图额外开销与新图例/断线保护一并接受并公开报告 — 大量数据显示明显提速且峰值不能丢失 — 6000 点基准约多 0.9 ms。
- 硬件边界：没有连接设备；串口打开失败/打开时停止等使用模拟串口验证。

## 最终检查

```text
python -m unittest discover -s tests -v
python src/rawimu_live.py --selftest --replay examples/rawimu_test.dat --truth examples/rawimu_truth.csv
python src/rawimu_live.py --selftest --replay examples/rawimua_ascii.dat
python -m py_compile src/rawimu_live.py tests/test_rawimu_live.py tests/test_gui.py tests/benchmark_viewer.py
git diff --check
```

性能重跑：`python tests/benchmark_viewer.py --baseline c7dce6e --plots`，具体同机基准值见 README。最终交付没有安装新依赖、创建提交、合并或推送。
