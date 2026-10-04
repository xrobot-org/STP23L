# STP23L

LDROBOT STP-23L 激光测距传感器驱动模块 / Driver Module for the LDROBOT STP-23L laser ranging sensor

## 1. 模块作用 / Purpose

构造时，STP23L 创建线程 `stp23l_thread`（`REALTIME` 优先级，栈深 `task_stack_depth`）并向 `ramfs` 的 `bin` 添加命令 `stp23l`。线程读取 UART 字节流，在固定的 10 字节帧头（`AA AA AA AA 00 02 00 00 B8 00`）上同步，读取 184 字节载荷（12 个点和传感器时间戳）并校验 8 位累加和。校验失败的帧被计数并丢弃，每个校验通过的帧发布一次。UART 的波特率由 BSP 按传感器的设置配置。

Upon construction, STP23L creates the thread `stp23l_thread` (`REALTIME` priority, stack depth `task_stack_depth`) and adds the command `stp23l` to `bin` of `ramfs`. The thread reads the UART byte stream, synchronizes on the fixed 10-byte frame header (`AA AA AA AA 00 02 00 00 B8 00`), reads the 184-byte payload (12 points and the sensor timestamp) and verifies the 8-bit sum checksum. Frames with a bad checksum are counted and dropped, and every good frame is published once. The UART baud rate is configured by the BSP to match the sensor.

## 2. Shell 命令 / Shell Command

- `bin/stp23l` 或 `bin/stp23l status`：打印校验通过与失败的帧计数，以及最近一帧的平均距离、有效点数、最小距离和最大距离。

- `bin/stp23l` or `bin/stp23l status`: prints the counters of good and bad frames, and the average distance, valid point count, minimum distance and maximum distance of the last frame.

## 3. 构造接口 / Constructor

```cpp
STP23L(LibXR::UART& uart, LibXR::RamFS& ramfs,
       const char* topic_name = "stp23l_frame",
       size_t task_stack_depth = 2048,
       uint32_t frame_timeout_ms = 200);
```

依赖：

- `uart`：连接 STP-23L 的 UART，由 BSP 配置波特率。
- `ramfs`：接收 `stp23l` 命令的 RamFS。

配置参数：

- `topic_name`：发布的 Topic 名称，默认 `stp23l_frame`。
- `task_stack_depth`：接收线程栈深，单位字节，默认 2048。
- `frame_timeout_ms`：接收线程每次等待 UART 数据的超时，单位 ms，默认 200；超时后重新等待。

Dependencies:

- `uart`: the UART connected to the STP-23L, with the baud rate configured by the BSP.
- `ramfs`: the RamFS that receives the `stp23l` command.

Configuration parameters:

- `topic_name`: name of the published Topic, default `stp23l_frame`.
- `task_stack_depth`: stack depth of the receive thread in bytes, default 2048.
- `frame_timeout_ms`: timeout of each wait for UART data in the receive thread, in ms, default 200; a timeout restarts the wait.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `topic_name`（默认 `stp23l_frame`） | 发布 | `STP23L::Frame` | 一帧测距结果，字段见下表 |

| 字段 | 含义 |
| --- | --- |
| `points[12]` | 原始点：`distance_mm`、`noise`、`peak`、`confidence`、`integration`、`reference_tof` |
| `sensor_timestamp` | 帧中的时间戳字段 |
| `average_distance_m` | 12 个点 `distance_mm` 的平均值，单位 m，包含无效点 |
| `min_distance_mm`、`max_distance_mm` | 12 个点 `distance_mm` 的最小值与最大值 |
| `valid_points` | `distance_mm > 0` 且 `confidence > 0` 的点数 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `topic_name` (default `stp23l_frame`) | Publish | `STP23L::Frame` | One ranging frame, fields in the table below |

| Field | Meaning |
| --- | --- |
| `points[12]` | raw points: `distance_mm`, `noise`, `peak`, `confidence`, `integration`, `reference_tof` |
| `sensor_timestamp` | timestamp field of the frame |
| `average_distance_m` | mean `distance_mm` of the 12 points in m, invalid points included |
| `min_distance_mm`, `max_distance_mm` | minimum and maximum `distance_mm` of the 12 points |
| `valid_points` | number of points with `distance_mm > 0` and `confidence > 0` |

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/STP23L` 写入的实例，`uart` 与 `ramfs` 填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/STP23L`, with `uart` and `ramfs` set to names registered by the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/STP23L
    id: stp23l_0
    args:
      - uart: usart6
      - ramfs: ramfs
      - topic_name: "stp23l_frame"
      - task_stack_depth: 2048
      - frame_timeout_ms: 200
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一台 LDROBOT STP-23L 激光测距传感器，通过 UART 连接。

Dependencies: LibXR.

Hardware: one LDROBOT STP-23L laser ranging sensor on a UART.
