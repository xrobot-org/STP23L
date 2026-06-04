# STP23L

STP23L 测距模块的 LibXR 封装，采用与 `xrobot-org/BMI088` 一致的模块结构：

- 后台线程读取 UART 字节流
- 用固定帧头和校验和解析完整一帧点云数据
- 发布完整 `STP23L::Frame` Topic
- 在 `ramfs/bin/stp23l` 提供状态命令

## Required Hardware

- `uart5` 或等价别名
- `ramfs`

## Constructor Arguments

- `topic_name`
- `task_stack_depth`
- `uart_name`
- `frame_timeout_ms`

## Published Topic

`topic_name` 的 payload 为 `STP23L::Frame`，包含：
- 12 个点的原始距离/噪声/强度/置信度/积分次数
- 传感器时间戳
- 平均距离、最小距离、最大距离
- 有效点数量
