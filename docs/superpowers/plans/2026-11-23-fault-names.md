# Fault name synchronization

用户提供 main_of_desay.m 及故障位截图，随后明确范围为“把图中的故障位名称同步到实时软件”。只修改 Python viewer 与其文档、测试和截图，未修改 MATLAB 分析文件。

- 名称依据上层 rawimu_status.m 的位操作及 raw/desay_imu_status_details.m，stat bit0..7 / word bit8..15 依次为 GyroX、GyroY、GyroZ、AccX、AccY、AccZ、Temp、FUSA。
- 总览故障位从上到下按此顺序，保留原始位号和当前0/1；1异常标红、0正常。诊断详情列出当前异常名。
- 仅同步故障定义，不复制 MATLAB 中的温度换算系数或修改状态原始数据、CSV、DAT。
- 新增单位置位、多位组合以及排除温度/计数器误判的测试；真实Tk验证名称、位号、数值和排列。先观察原版失败，再实现通过。
- 全部46项测试通过，语法编译和diff whitespace检查通过；独立审查确认位映射和显示顺序一致、无发现。
- 更新并查看1600×960和1100×720演示截图；其中故障位变化明确使用合成演示，不是硬件故障实测。
