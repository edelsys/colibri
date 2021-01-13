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

#include "mutelemetry/mutelemetry_network.h"

#include <glog/logging.h>
#include <muqueue/erqperiodic.h>
#include <muqueue/scheduler.h>

#include <cassert>
#include <chrono>
#include <thread>

using namespace std;
using namespace fflow;
using namespace mutelemetry_network;
using namespace mutelemetry_tools;

fflow::pointprec_t MutelemetryStreamer::proto_command_handler(
    uint8_t *payload, size_t len, fflow::SparseAddress sa,
    BaseComponentPtr cc) {
  (void)len;
  (void)cc;
  mavlink_message_t *rxmsg = MAVPAYLOAD_TO_MAVMSG(payload);
  mavlink_command_long_t lcmd;
  mavlink_msg_command_long_decode(rxmsg, &lcmd);

#if 1
  uint32_t targetMcastId = sa.group_id;
  uint32_t targetCompId = sa.instance_id;
#else
  uint32_t targetMcastId = lcmd.target_system;
  uint32_t targetCompId = lcmd.target_component;
#endif

  // skip all except start/stop commands
  if (!(lcmd.command == MAV_CMD_LOGGING_START ||
        lcmd.command == MAV_CMD_LOGGING_STOP))
    return 1.0;

  StreamerState state = state_, new_state = StreamerState::STATE_UNKNOWN;

  bool confirm = lcmd.confirmation;

  // check if current state corresponds with the command
  if (lcmd.command == MAV_CMD_LOGGING_START) {
    LOG(INFO) << "Received MAV_CMD_LOGGING_START";
    if (state == StreamerState::STATE_INIT) {
      new_state = StreamerState::STATE_CONNECTED;
      target_system_ = targetMcastId;
      target_component_ = targetCompId;
    } else if (state == StreamerState::STATE_STOPPED) {
      if (target_system_ == targetMcastId &&
          target_component_ == targetCompId) {
        new_state = StreamerState::STATE_RUNNING;
      }
    }
  } else {
    LOG(INFO) << "Received MAV_CMD_LOGGING_STOP" << uint32_t(target_system_)
              << "==" << uint32_t(targetMcastId) << " ; "
              << uint32_t(target_component_) << "==" << uint32_t(targetCompId);
    if (state == StreamerState::STATE_RUNNING) {
      if (target_system_ == targetMcastId &&
          target_component_ == targetCompId) {
        new_state = StreamerState::STATE_STOPPED;
        LOG(INFO) << "new_state = StreamerState::STATE_STOPPED";
      }
    }
  }

  if (new_state != StreamerState::STATE_UNKNOWN) {
    __UNUSED__ bool result = set_state(state, new_state);
    LOG(INFO) << "State changed from " << state << " to " << state_;
    assert(result);
  }

  if (confirm) {
    mavlink_message_t msg;

    mavlink_msg_command_ack_pack(
        uint8_t(lcmd.target_system) /*system_id*/,
        uint8_t(lcmd.target_component) /* our own component_id*/, &msg /*msg*/,
        lcmd.command /*command*/,
        (new_state != StreamerState::STATE_UNKNOWN) /*result*/
            ? MAV_RESULT_ACCEPTED
            : MAV_RESULT_FAILED,
        0 /*progress*/, MutelemetryStreamer::get_port() /*result_param2*/,
        uint8_t(sa.group_id) /*target_system*/,
        sa.instance_id /* target_component */);

    roster_->sendmavmsg(msg,
                        {fflow::SparseAddress(sa.group_id, sa.instance_id, 0)});
  }

  return 1.0;
}

fflow::pointprec_t MutelemetryStreamer::proto_logging_ack_handler(
    uint8_t *payload, size_t /*len*/, SparseAddress /*sa*/,
    BaseComponentPtr /*cc*/) {
  mavlink_message_t *rxmsg = MAVPAYLOAD_TO_MAVMSG(payload);
  mavlink_logging_ack_t logging_ack;
  mavlink_msg_logging_ack_decode(rxmsg, &logging_ack);

  StreamerState state = state_.load();

  switch (state) {
    case StreamerState::STATE_ACK_WAIT: {
      StreamerState new_state =
          logging_ack.sequence == static_cast<uint16_t>(seq_)
              ? StreamerState::STATE_ACK_RECV
              : StreamerState::STATE_RESEND_DEF;
      bool result = set_state(state, new_state);
      if (!result) {
        LOG(ERROR) << "Another thread has changed the state: "
                   << "Was => " << state << " Now => " << state_.load()
                   << " Must be => " << new_state;
        assert(0);
      }
    } break;

    default:
      LOG(INFO) << "Receiving logging ack message in " << state << " state";
      break;
  }

  return 1.0;
}

bool MutelemetryStreamer::init(RouteSystemPtr roster,
                               ConcQueue<SerializedDataPtr> *data_queue) {
  if (running_ || roster_ != nullptr || data_queue_ != nullptr) return false;
  if (roster == nullptr || data_queue == nullptr) return false;

  roster->add_protocol2(proto_table, proto_table_len);

  roster_ = roster;
  data_queue_ = data_queue;
  return true;
}

bool MutelemetryStreamer::send_ulog(const uint8_t *data, size_t data_len) {
  if (data_len > MAVLINK_MSG_LOGGING_DATA_FIELD_DATA_LEN) {
    LOG(ERROR) << "Message size is too big: " << data_len
               << " bytes while max possible is "
               << MAVLINK_MSG_LOGGING_DATA_FIELD_DATA_LEN << " bytes";
    return false;
  }

  std::shared_ptr<mavlink_message_t> msg =
      std::make_shared<mavlink_message_t>();

  ///< @bug workaround of mavlink_msg_logging_data_pack() which copies
  /// exactly 249 bytes of data into ulog payload. We'r forced to fit
  /// data buffer to 249 byets as well even if its passed as 'const'
  uint8_t tempbuf[MAVLINK_MSG_LOGGING_DATA_FIELD_DATA_LEN];
  memcpy(tempbuf, data, data_len);

  /* uint16_t msg_len = */ mavlink_msg_logging_data_pack(
      roster_->getMcastId(), roster_->getMcompId(), &(*msg), target_system_,
      target_component_, 0, uint8_t(data_len), 255, tempbuf);

  fflow::post_function<void>([msg, this](void) -> void {
    roster_->sendmavmsg(
        *msg, {fflow::SparseAddress(target_system_, target_component_, 0)});
  });

  return true;
}

bool MutelemetryStreamer::send_ulog_ack(const uint8_t *data, size_t data_len,
                                        uint16_t seq) {
  if (data_len > MAVLINK_MSG_LOGGING_DATA_ACKED_FIELD_DATA_LEN) {
    LOG(ERROR) << "Message size is too big: " << data_len
               << " bytes while max possible is "
               << MAVLINK_MSG_LOGGING_DATA_ACKED_FIELD_DATA_LEN << " bytes";
    return false;
  }

  LOG(INFO) << "Send ULog definition message of size=" << data_len
            << " with seq=" << uint32_t(seq);

  mavlink_message_t msg;
  uint16_t msg_len = mavlink_msg_logging_data_acked_pack(
      roster_->getMcastId(), roster_->getMcompId(), &msg, target_system_,
      target_component_, seq, uint8_t(data_len), 255, data);
  (void)msg_len;
  roster_->sendmavmsg(
      msg, {fflow::SparseAddress(target_system_, target_component_, 0)});

  return true;
}

void MutelemetryStreamer::sync_loop() {
  LOG(INFO) << "Sync loop started";
  assert(running_ == false);

  while (true) {
    if (data_queue_->empty()) {
      this_thread::sleep_for(chrono::milliseconds(50));
      continue;
    }

    auto dp = data_queue_->front();
    const uint8_t *buffer = dp->data();

#ifdef CHECK_PARSE_VALIDITY_STREAMER
    if (!check_ulog_valid(buffer)) {
      LOG(ERROR) << "Validity check failed for definitions";
      assert(0);
    }
#endif

    if (check_ulog_data_begin(buffer)) {
      // start discarding ulog data until all definitions are sent,
      // this is needed to keep up to time data on client side
      LOG(INFO) << "Definitions section size = " << definitions_.size();
      discarding_ = true;
      break;
    }

    data_queue_->dequeue();
    definitions_.emplace_back(dp);
  }

  size_t defs_sz = definitions_.size();

  while (seq_ < defs_sz) {
    StreamerState state = state_.load();

    //    LOG(INFO) << "State is " << state << " seq:" << seq_
    //              << " seqsize: " << defs_sz;

    switch (state) {
      case StreamerState::STATE_CONNECTED:
        assert(target_system_ != 0);
        assert(target_component_ != 0);
      case StreamerState::STATE_RESEND_DEF:
      case StreamerState::STATE_SEND_DEF: {
        SerializedDataPtr dp = definitions_[seq_];
        if (!send_ulog_ack(dp->data(), dp->size(), uint16_t(seq_))) {
          LOG(ERROR) << "Failed to send ULog definition message";
          assert(0);
          break;
        }
        sync_timeout_ = 0;
        StreamerState new_state = StreamerState::STATE_ACK_WAIT;
        bool result = set_state(state, new_state);
        if (!result) {
          LOG(ERROR) << "Another thread has changed the state: "
                     << "Was => " << state << " Now => " << state_
                     << " Must be => " << new_state;
          assert(0);
        }
      } break;

      case StreamerState::STATE_ACK_RECV: {
        StreamerState new_state = StreamerState::STATE_SEND_DEF;
        bool result = set_state(state, new_state);
        if (!result) {
          LOG(ERROR) << "Another thread has changed the state: "
                     << "Was => " << state << " Now => " << state_
                     << " Must be => " << new_state;
          assert(0);
        }
        seq_++;
      } break;

      case StreamerState::STATE_STOPPED:
        this_thread::sleep_for(chrono::milliseconds(100));
      case StreamerState::STATE_INIT:
        this_thread::sleep_for(chrono::milliseconds(100));
        break;

      case StreamerState::STATE_ACK_WAIT:
        this_thread::sleep_for(chrono::milliseconds(50));
        sync_timeout_ += 50;
        if (sync_timeout_ == 1000) {
          // sync timeout (approx. 1s) reached, resend the last message
          StreamerState new_state = StreamerState::STATE_RESEND_DEF;
          bool result = set_state(state, new_state);
          if (!result) {
            LOG(ERROR) << "Another thread has changed the state: "
                       << "Was => " << state << " Now => " << state_
                       << " Must be => " << new_state;
            assert(0);
          }
        }
        break;

      default:
        LOG(ERROR) << state << " state";
        assert(0);
    }
  }

  set_state(state_.load(), StreamerState::STATE_RUNNING);
  running_ = true;
  LOG(INFO) << "Sync loop exited";
}

void MutelemetryStreamer::discard_loop() {
  while (!discarding_) this_thread::sleep_for(chrono::milliseconds(50));
  LOG(INFO) << "Discard loop started";
  while (!running_) {
    data_queue_->dequeue();
    ++skip_cntr_;
  }
  LOG(INFO) << "Discard loop exited";
}

void MutelemetryStreamer::main_loop() {
  if (!running_) return;

  StreamerState st = state_.load();
  VLOG(5) << "State is " << st;
  assert(st == StreamerState::STATE_RUNNING ||
         st == StreamerState::STATE_STOPPED);

  while (!data_queue_->empty()) {
    auto dp = data_queue_->dequeue();
    assert(dp != nullptr);

    if (st == StreamerState::STATE_STOPPED) {
      ++skip_cntr_;
      VLOG(5) << "Skipping " << skip_cntr_ << " ULog packet";
      continue;
    }

    const uint8_t *buffer = dp->data();
#ifdef CHECK_PARSE_VALIDITY_STREAMER
    {
      std::lock_guard<std::mutex> lock(parser_mutex_);
      if (!check_ulog_valid(buffer)) {
        LOG(ERROR) << "Validity check failed for data";
        assert(0);
      }
    }
#endif

    // send ULog data without acknowledgement
    if (!send_ulog(buffer, dp->size())) {
      ++skip_cntr_;
      VLOG(4) << "ULog packet " << skip_cntr_
              << " won't fit into mavlink buffer, skipping";
    } else {
      ++send_cntr_;
      VLOG(4) << "ULog packet " << send_cntr_ << " sent";
    }
  }
}

void MutelemetryStreamer::run(bool rt) {
  post_function<void>([&](void) -> void { sync_loop(); });
  if (rt) post_function<void>([&](void) -> void { discard_loop(); });
  add_periodic<void>(([&](void) -> void { main_loop(); }), 0.000001, 0.1);
}
