# STP23L

## Static assembly source line

This source line uses explicit C++ constructor dependencies and ordered instance
arguments. Inspect the current primary header with `xrobot_mod_parser --path .`;
its declarations, not old manifest/config examples, define the interface.
Historical HardwareContainer/ApplicationManager examples below apply only to the
older dynamic source tags. Device/protocol descriptions remain relevant.
See the XRobot [migration guide](https://github.com/xrobot-org/XRobot/blob/dev/MIGRATION.md).
Compilation is not hardware validation; retain version-specific board evidence.


LDROBOT STP-23L UART laser ranging sensor module for XRobot.

This module reads the STP-23L UART byte stream in a background thread, syncs on
the fixed frame header, verifies checksum, publishes complete 12-point ranging
frames, and exposes a RamFS shell command for status output.

The UART name is a constructor argument, so projects may use other hardware
aliases if needed.

## Required Hardware

- `stp23l_uart`
- `ramfs`

## Constructor Arguments

- `topic_name`: default `"stp23l_frame"`
- `task_stack_depth`: default `2048`
- `uart_name`: default `"stp23l_uart"`
- `frame_timeout_ms`: default `200`

## Published Topics

- `topic_name`: `STP23L::Frame`, including 12 raw points, sensor timestamp, average distance, min / max distance, and valid point count

## Shell Commands

The module registers `bin/stp23l` in `RamFS`.

- `bin/stp23l` or `bin/stp23l status`: print frame counters and latest distance statistics

## XRobot Configuration Example

```yaml
- id: rangefinder
  name: STP23L
  constructor_args:
    topic_name: "stp23l_frame"
    task_stack_depth: 2048
    uart_name: "stp23l_uart"
    frame_timeout_ms: 200
```
