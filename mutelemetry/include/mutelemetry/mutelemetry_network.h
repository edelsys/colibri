/*
 * Copyright (c) 2020, EDEL LLC <http://www.edelsys.com>
 * All Rights Reserved
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * https://opensource.org/licenses/MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <atomic>
#include <functional>

#include <muroute/concqueue.h>
#include <muroute/mavlink2/common/mavlink.h>
#include <muroute/subsystem.h>

#include "mutelemetry_tools.h"

//#define CHECK_PARSE_VALIDITY_STREAMER

namespace mutelemetry_network {

class MutelemetryStreamer {
 private:
  enum class StreamerState : uint8_t {
    STATE_INIT = 0,
    STATE_CONNECTED,
    STATE_SEND_DEF,
    STATE_RESEND_DEF,
    STATE_ACK_RECV,
    STATE_ACK_WAIT,
    STATE_RUNNING,
    STATE_STOPPED,
    STATE_UNKNOWN
  };

  friend std::ostream &operator<<(std::ostream &o, const StreamerState &s) {
    static const std::string states[] = {"INIT",       "CONNECTED", "SEND_DEF",
                                         "RESEND_DEF", "ACK_RECV",  "ACK_WAIT",
                                         "RUNNING",    "STOPPED",   "UNKNOWN"};
    if (s < StreamerState::STATE_UNKNOWN)
      o << states[uint8_t(s)];
    else
      o << states[uint8_t(StreamerState::STATE_UNKNOWN)];
    return o;
  }

 public:
  static constexpr uint32_t port = 7788;
  static uint32_t get_port() { return port; }

 public:
  MutelemetryStreamer()
      : discarding_(false),
        running_(false),
        roster_(nullptr),
        data_queue_(nullptr),
        state_(StreamerState::STATE_INIT),
        seq_(0),
        target_system_(0),
        target_component_(0),
        sync_timeout_(0),
        send_cntr_(0),
        skip_cntr_(0) {}

  MutelemetryStreamer(const MutelemetryStreamer &) = delete;
  MutelemetryStreamer &operator=(const MutelemetryStreamer &) = delete;

  virtual ~MutelemetryStreamer() { release(); }

 public:
  // TODO: make start/stop interface
  void run(bool rt);
  bool init(fflow::RouteSystemPtr,
            ConcQueue<mutelemetry_tools::SerializedDataPtr> *);
  void release() {}

 private:
  void sync_loop();
  void discard_loop();
  void main_loop();

 private:
  std::atomic<bool> discarding_;
  std::atomic<bool> running_;
  fflow::RouteSystemPtr roster_;
  ConcQueue<mutelemetry_tools::SerializedDataPtr> *data_queue_;
  std::atomic<StreamerState> state_;
  size_t seq_;
  std::vector<mutelemetry_tools::SerializedDataPtr> definitions_;
#ifdef CHECK_PARSE_VALIDITY_STREAMER
  // only for debug since ULog parser is not thread-safe
  std::mutex parser_mutex_;
#endif

  // protocol definition
 private:
  fflow::pointprec_t proto_command_handler(uint8_t *, size_t,
                                           fflow::SparseAddress,
                                           fflow::BaseComponentPtr cc);
  fflow::pointprec_t proto_logging_ack_handler(uint8_t *, size_t,
                                               fflow::SparseAddress,
                                               fflow::BaseComponentPtr cc);

  uint8_t target_system_;
  uint8_t target_component_;
  uint16_t sync_timeout_;
  std::atomic<uint64_t> send_cntr_;
  std::atomic<uint64_t> skip_cntr_;

  static constexpr size_t proto_table_len = 2;

  fflow::message_handler_note_t proto_table[proto_table_len] = {
      {MAVLINK_MSG_ID_COMMAND_LONG /* #76 */,
       std::bind(&MutelemetryStreamer::proto_command_handler, this,
                 std::placeholders::_1, std::placeholders::_2,
                 std::placeholders::_3, std::placeholders::_4)},
      {MAVLINK_MSG_ID_LOGGING_ACK /* #268 */,
       [this](uint8_t *a, size_t b, fflow::SparseAddress c,
              fflow::BaseComponentPtr d) -> fflow::pointprec_t {
         return proto_logging_ack_handler(a, b, c, d);
       }},
  };

  bool send_ulog(const uint8_t *, size_t);
  bool send_ulog_ack(const uint8_t *, size_t, uint16_t);
  inline bool set_state(StreamerState old_state, StreamerState new_state) {
    return state_.compare_exchange_strong(old_state, new_state);
  }
};

}  // namespace mutelemetry_network
