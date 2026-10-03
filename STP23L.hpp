#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: LDROBOT STP-23L 激光测距传感器驱动模块 / Driver module for the LDROBOT STP-23L laser ranging sensor
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>

#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "uart.hpp"

/**
 * @brief STP-23L 激光测距传感器驱动，解析 UART 帧并发布 12 点测距结果。
 *        Driver for the STP-23L laser ranging sensor; parses the UART frames and
 *        publishes the 12-point ranging result.
 */
class STP23L
{
 public:
#pragma pack(push, 1)
  /**
   * @brief 传感器上报的单点测距数据。
   *        One ranging point reported by the sensor.
   */
  struct Point
  {
    int16_t distance_mm;    ///< 距离，mm Distance, mm
    uint16_t noise;         ///< 噪声 Noise
    uint32_t peak;          ///< 接收峰值 Receive peak
    uint8_t confidence;     ///< 置信度 Confidence
    uint32_t integration;   ///< 积分次数 Integration count
    int16_t reference_tof;  ///< 参考飞行时间 Reference time of flight
  };

  /**
   * @brief 一帧测距结果，由 Topic 发布。
   *        One ranging frame published on the Topic.
   */
  struct Frame
  {
    Point points[12];           ///< 12 个原始点 The 12 raw points
    uint32_t sensor_timestamp;  ///< 帧中的时间戳字段 Timestamp field of the frame
    float average_distance_m;   ///< 平均距离，m Mean distance, m
    int16_t min_distance_mm;    ///< 最小距离，mm Minimum distance, mm
    int16_t max_distance_mm;    ///< 最大距离，mm Maximum distance, mm
    uint8_t valid_points;       ///< 有效点数 Valid point count
  };
#pragma pack(pop)

  /**
   * @brief 构造 STP23L：向 ramfs 的 bin 注册 `stp23l` 命令并创建接收线程。
   *        Construct STP23L: register the `stp23l` command in bin of ramfs and create
   *        the receive thread.
   *
   * @param uart 连接 STP-23L 的 UART，波特率由 BSP 配置。
   *             UART connected to the STP-23L, with the baud rate configured by the BSP.
   * @param ramfs 接收 `stp23l` 命令的 RamFS。
   *              RamFS that receives the `stp23l` command.
   * @param topic_name 测距帧 Topic 名称。
   *                   Name of the ranging frame Topic.
   * @param task_stack_depth 接收线程栈深。
   *                         Stack depth of the receive thread.
   * @param frame_timeout_ms 接收线程每次等待 UART 数据的超时，单位 ms。
   *                         Timeout of each wait for UART data, in ms.
   */
  STP23L(
      LibXR::UART& uart,
      LibXR::RamFS& ramfs,
      const char* topic_name = "stp23l_frame",
      size_t task_stack_depth = 2048,
      uint32_t frame_timeout_ms = 200)
      : frame_timeout_ms_(frame_timeout_ms),
        topic_(LibXR::Topic::CreateTopic<Frame>(topic_name)),
        uart_(std::addressof(uart)),
        cmd_file_(LibXR::RamFS::CreateCommand("stp23l", CommandFunc, this))
  {
    ramfs.bin_.Add(cmd_file_);

    thread_.Create(this, ThreadFunc, "stp23l_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
  }

 private:
#pragma pack(push, 1)
  struct RawFrame
  {
    Point points[12];
    uint32_t timestamp;
  };
#pragma pack(pop)

  static constexpr auto FRAME_HEAD = std::to_array<uint8_t>(
      {0xAA, 0xAA, 0xAA, 0xAA, 0x00, 0x02, 0x00, 0x00, 0xB8, 0x00});

  static int CommandFunc(STP23L* self, int argc, char** argv)
  {
    if (argc == 1 || (argc == 2 && std::strcmp(argv[1], "status") == 0))
    {
      LibXR::STDIO::Printf<"frames=%u bad=%u avg=%.3f valid=%u min=%d max=%d\r\n">(
          self->frame_count_, self->bad_frame_count_, self->frame_.average_distance_m,
          static_cast<unsigned int>(self->frame_.valid_points),
          static_cast<int>(self->frame_.min_distance_mm),
          static_cast<int>(self->frame_.max_distance_mm));
      return 0;
    }

    LibXR::STDIO::Printf<"usage: stp23l [status]\r\n">();
    return -1;
  }

  static void ThreadFunc(STP23L* self)
  {
    uint8_t buffer[64] = {};

    while (true)
    {
      LibXR::ReadOperation ready_op(self->sem_uart_, self->frame_timeout_ms_);
      if (self->uart_->Read({nullptr, 0}, ready_op) != LibXR::ErrorCode::OK)
      {
        continue;
      }

      while (self->uart_->read_port_->Size() > 0)
      {
        const size_t size = std::min(self->uart_->read_port_->Size(), sizeof(buffer));
        LibXR::ReadOperation read_now;
        if (self->uart_->Read({buffer, size}, read_now) != LibXR::ErrorCode::OK)
        {
          break;
        }

        for (size_t i = 0; i < size; ++i)
        {
          self->ParseByte(buffer[i]);
        }
      }
    }
  }

  void ParseByte(uint8_t byte)
  {
    switch (state_)
    {
      case ParseState::SYNC_HEAD:
        if (byte == FRAME_HEAD[head_index_])
        {
          ++head_index_;
          if (head_index_ == FRAME_HEAD.size())
          {
            head_index_ = 0;
            payload_index_ = 0;
            checksum_ = 0xBA;
            state_ = ParseState::READ_PAYLOAD;
          }
        }
        else
        {
          head_index_ = (byte == FRAME_HEAD[0]) ? 1U : 0U;
        }
        break;

      case ParseState::READ_PAYLOAD:
        payload_buffer_[payload_index_++] = byte;
        checksum_ = static_cast<uint8_t>(checksum_ + byte);
        if (payload_index_ == payload_buffer_.size())
        {
          state_ = ParseState::READ_CHECKSUM;
        }
        break;

      case ParseState::READ_CHECKSUM:
        if (byte == checksum_)
        {
          PublishPayload();
        }
        else
        {
          ++bad_frame_count_;
        }
        payload_index_ = 0;
        checksum_ = 0;
        state_ = ParseState::SYNC_HEAD;
        break;
    }
  }

  void PublishPayload()
  {
    RawFrame raw{};
    std::memcpy(&raw, payload_buffer_.data(), sizeof(raw));

    frame_.sensor_timestamp = raw.timestamp;
    frame_.valid_points = 0;
    frame_.min_distance_mm = 0;
    frame_.max_distance_mm = 0;

    int32_t distance_sum = 0;
    for (size_t i = 0; i < 12; ++i)
    {
      frame_.points[i] = raw.points[i];
      const int16_t distance = raw.points[i].distance_mm;
      distance_sum += distance;

      if (i == 0 || distance < frame_.min_distance_mm)
      {
        frame_.min_distance_mm = distance;
      }
      if (i == 0 || distance > frame_.max_distance_mm)
      {
        frame_.max_distance_mm = distance;
      }
      if (distance > 0 && raw.points[i].confidence > 0)
      {
        ++frame_.valid_points;
      }
    }

    frame_.average_distance_m = static_cast<float>(distance_sum) / 12.0f / 1000.0f;

    ++frame_count_;
    topic_.Publish(frame_);
  }

  enum class ParseState : uint8_t
  {
    SYNC_HEAD,
    READ_PAYLOAD,
    READ_CHECKSUM,
  };

  uint32_t frame_timeout_ms_ = 200;
  uint32_t frame_count_ = 0;
  uint32_t bad_frame_count_ = 0;

  ParseState state_ = ParseState::SYNC_HEAD;
  size_t head_index_ = 0;
  size_t payload_index_ = 0;
  uint8_t checksum_ = 0;

  Frame frame_{};
  std::array<uint8_t, sizeof(RawFrame)> payload_buffer_{};

  LibXR::Topic topic_;
  LibXR::UART* uart_;
  LibXR::Semaphore sem_uart_;
  LibXR::Thread thread_;
  LibXR::RamFS::File cmd_file_;
};
