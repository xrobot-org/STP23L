# STP23L

XRobot Module for the LDROBOT STP-23L UART laser ranging sensor.

The `stp23l_thread` thread (`REALTIME` priority) reads the UART byte stream,
synchronizes on the fixed 10-byte frame header
(`AA AA AA AA 00 02 00 00 B8 00`), reads the 184-byte payload (12 points and a
sensor timestamp) and verifies the 8-bit sum checksum. Frames with a bad
checksum are counted and dropped; every good frame is published. The module
does not configure the UART; the BSP must set the baud rate the sensor uses.

## Published topic

`topic_name` (default `stp23l_frame`), type `STP23L::Frame`:

| Field | Meaning |
| --- | --- |
| `points[12]` | raw points: `distance_mm`, `noise`, `peak`, `confidence`, `integration`, `reference_tof` |
| `sensor_timestamp` | timestamp field of the frame |
| `average_distance_m` | mean `distance_mm` of all 12 points, in m (invalid points included) |
| `min_distance_mm`, `max_distance_mm` | min / max `distance_mm` of all 12 points |
| `valid_points` | number of points with `distance_mm > 0` and `confidence > 0` |

## Shell command

The module adds the command `stp23l` to `bin` in `ramfs`.

- `stp23l` or `stp23l status`: prints the good and bad frame counters and the
  last average distance, valid point count, min and max distance.

## Dependencies

No other Modules; LibXR only.

## Constructor

```cpp
STP23L(LibXR::UART& uart, LibXR::RamFS& ramfs,
       const char* topic_name = "stp23l_frame",
       size_t task_stack_depth = 2048,
       uint32_t frame_timeout_ms = 200);
```

Dependencies:

- `uart`: the UART connected to the STP-23L, already configured by the BSP.
- `ramfs`: RamFS that receives the `stp23l` command.

Configuration:

- `topic_name`: name of the published topic, default `stp23l_frame`.
- `task_stack_depth`: stack size of the receive thread, default 2048.
- `frame_timeout_ms`: timeout of each wait for UART data in the receive thread,
  ms, default 200 (a timeout only restarts the wait).

## Use

```sh
xrobot module add xrobot-org/STP23L
xrobot setup
xrobot instance add xrobot-org/STP23L
```

`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of
objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/STP23L
    id: stp23l_0
    args:
      - uart: usart6
      - ramfs: ramfs
      - topic_name: '"stp23l_frame"'
      - task_stack_depth: '2048'
      - frame_timeout_ms: '200'
```

BSP side:

```cpp
XR_REGISTER(usart6, LibXR::UART);
XR_REGISTER(ramfs, LibXR::RamFS);
```

Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/STP23L` in a BSP, prints the current
constructor.
