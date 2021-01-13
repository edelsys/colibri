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

#include "muroute/mavparamproto.h"

#include <cstddef>
#include <thread>

using namespace std;
using namespace fflow;

static size_t mem_copy(void *dest, size_t dsize, const void *src, size_t ssize,
                       size_t cnt) {
  if (dest == NULL || src == NULL) return 0;
  if (dsize == 0 || ssize == 0 || cnt == 0) return 0;
  size_t ncopy = min(max(dsize, ssize), cnt);
  memmove(dest, src, ncopy);
  return ncopy;
}

void MavParamProto::send_mavlink_message(mavlink_message_t &msg,
                                         int src_comp_id, int dst_sys_id,
                                         int dst_comp_id) {
  uint8_t buffer[MAX_MAVLINK_MESSAGE_SIZE];
  uint8_t *data = buffer;
  size_t len = mavlink_msg_to_send_buffer(data, &msg);
  uint32_t src_sys_id = getRoster()->getMcastId();

  if (len > 0) {
    LOG(INFO) << "Sending from <" << src_sys_id << ":" << src_comp_id << "> to "
              << "<" << dst_sys_id << ":" << dst_comp_id << ">";
    const auto src = SparseAddress(src_sys_id, src_comp_id, 0);
    auto dst = fflow::SparseAddress(dst_sys_id, dst_comp_id, 0);
    getRoster()->__send(msg, src, {dst});
  } else {
    LOG(ERROR) << "Failed to send message from <" << src_sys_id << ":"
               << src_comp_id << "> to "
               << "<" << dst_sys_id << ":" << dst_comp_id << ">";
  }
}

void MavParamProto::send_parameter_ext(int param_index, int src_comp_id,
                                       int dst_sys_id, int dst_comp_id) {
  RouteSystemPtr roster = getRoster();
  BaseComponentPtr comp = roster->getCBus().get_component(src_comp_id);
  assert(comp);

  send_parameter_ext(comp->getParameterId(param_index).c_str(), src_comp_id,
                     dst_sys_id, dst_comp_id);
}

void MavParamProto::send_parameter_ext(const char *param_id, int src_comp_id,
                                       int dst_sys_id, int dst_comp_id) {
  if (param_id && param_id[0] != '\0') {
    mavlink_message_t msg;
    int mssleep = 20;

    RouteSystemPtr roster = getRoster();
    BaseComponentPtr comp = roster->getCBus().get_component(src_comp_id);
    assert(comp);

    string param_id_str =
        MavParams::toStr(param_id, MavParams::max_param_id_len);

    bool result = true;
    int param_index = comp->getParameterIdx(param_id_str);
    if (param_index < 0) result = false;

    int type = comp->getParameterType(param_id_str);
    if (type <= 0 || type >= MAV_PARAM_EXT_TYPE_ENUM_END) result = false;

    string value = comp->getParameterValue(param_id_str);
    if (value.empty()) result = false;

    if (result) {
      mavlink_param_ext_value_t param_value;

      param_value.param_count = 1;
      param_value.param_index = static_cast<uint16_t>(param_index);
      param_value.param_type = static_cast<uint8_t>(type);

      mem_copy(param_value.param_id, sizeof(param_value.param_id),
               param_id_str.c_str(), param_id_str.size() + 1,
               sizeof(param_value.param_id));
      mem_copy(param_value.param_value, sizeof(param_value.param_value),
               value.data(), value.size(), sizeof(param_value.param_value));

      mavlink_msg_param_ext_value_encode(roster->getMcastId(), src_comp_id,
                                         &msg, &param_value);
    } else {
      mavlink_param_ext_ack_t param_ext_ack;

      param_ext_ack.param_type = static_cast<uint8_t>(type);
      param_ext_ack.param_result = PARAM_ACK_FAILED;

      mem_copy(param_ext_ack.param_id, sizeof(param_ext_ack.param_id), param_id,
               MavParams::max_param_id_len, sizeof(param_ext_ack.param_id));

      mavlink_msg_param_ext_ack_encode(roster->getMcastId(), src_comp_id, &msg,
                                       &param_ext_ack);
    }

    this_thread::sleep_for(chrono::milliseconds(mssleep));
    send_mavlink_message(msg, src_comp_id, dst_sys_id, dst_comp_id);
  }
}

void MavParamProto::send_parameters_ext(int src_comp_id, int dst_sys_id,
                                        int dst_comp_id) {
  RouteSystemPtr roster = getRoster();
  BaseComponentPtr comp = roster->getCBus().get_component(src_comp_id);
  assert(comp);

  LOG(INFO) << "REQUESTED ALL PARAMETERS EXT FOR COMPONENT WITH ID="
            << static_cast<int>(comp->getId());

  const auto &param_id_to_value = comp->getParameterList();

  if (param_id_to_value.size()) {
    mavlink_param_ext_value_t param_ext_value;
    int mssleep = 20;
    param_ext_value.param_count = param_id_to_value.size();
    assert(param_ext_value.param_count == comp->getParameterCount());

    for (auto &x : param_id_to_value) {
      string param_id = MavParams::toStr(x.first.c_str(), x.first.size());
      mavlink_message_t msg;

      param_ext_value.param_index = comp->getParameterIdx(param_id);
      assert(param_ext_value.param_index >= 0 &&
             param_ext_value.param_index < param_ext_value.param_count);

      mem_copy(param_ext_value.param_id, sizeof(param_ext_value.param_id),
               x.first.c_str(), x.first.size() + 1,
               sizeof(param_ext_value.param_id));
      mem_copy(param_ext_value.param_value, sizeof(param_ext_value.param_value),
               x.second.data(), x.second.size(),
               sizeof(param_ext_value.param_id));

      param_ext_value.param_type = comp->getParameterType(param_id);
      assert(param_ext_value.param_type > 0 &&
             param_ext_value.param_type < MAV_PARAM_EXT_TYPE_ENUM_END);

      mavlink_msg_param_ext_value_encode(roster->getMcastId(), src_comp_id,
                                         &msg, &param_ext_value);

      this_thread::sleep_for(chrono::milliseconds(mssleep));
      send_mavlink_message(msg, src_comp_id, dst_sys_id, dst_comp_id);
    }

    LOG(INFO) << "COMPONENT WITH ID=" << static_cast<int>(comp->getId())
              << " SENT " << param_id_to_value.size() << " EXT PARAMETERS";
  } else {
    LOG(INFO) << "COMPONENT WITH ID=" << static_cast<int>(comp->getId())
              << " HAS NO REGISTERED EXT PARAMETERS";
  }
}

pointprec_t MavParamProto::param_request_list_handler_ext(
    uint8_t *payload, size_t len, SparseAddress sa, BaseComponentPtr comp) {
  (void)len;
  (void)comp;

  mavlink_message_t *msg = MAVPAYLOAD_TO_MAVMSG(payload);
  mavlink_param_ext_request_list_t param_list_ext;
  mavlink_msg_param_ext_request_list_decode(msg, &param_list_ext);

  RouteSystemPtr roster = getRoster();
  assert(roster);
  assert(param_list_ext.target_system == roster->getMcastId());

  if (param_list_ext.target_component == MAV_COMP_ID_ALL) {
    LOG(INFO) << " BROADCAST REQUEST EXT OF ALL PARAMETERS";

    for (auto it = roster->getCBus().begin(); it != roster->getCBus().end();
         ++it) {
      // broadcast according to protocol
      send_parameters_ext(it->first, sa.group_id /*, sa.instance_id*/);
    }
  } else {
    // normally, must not enter this branch
    int comp_id = static_cast<int>(param_list_ext.target_component);

    if (!roster->getCBus().has_component(comp_id)) {
      LOG(INFO) << "NO COMPONENT WITH ID=" << comp_id;
      return 1.0;
    }

    send_parameters_ext(comp_id, sa.group_id /*, sa.instance_id*/);
  }

  return 1.0;
}

pointprec_t MavParamProto::param_request_read_handler_ext(
    uint8_t *payload, size_t len, SparseAddress sa, BaseComponentPtr comp) {
  (void)len;
  (void)comp;
  mavlink_message_t *imsg = MAVPAYLOAD_TO_MAVMSG(payload);

  mavlink_param_ext_request_read_t pread_ext;
  mavlink_msg_param_ext_request_read_decode(imsg, &pread_ext);

  RouteSystemPtr roster = getRoster();
  assert(roster);
  assert(pread_ext.target_system == roster->getMcastId());

  LOG(INFO) << " REQUESTED PARAMETER READ EXT FOR COMPONENT WITH ID= "
            << static_cast<int>(pread_ext.target_component);

  int comp_id = static_cast<int>(pread_ext.target_component);
  if (!roster->getCBus().has_component(comp_id)) {
    LOG(INFO) << "NO COMPONENT WITH ID=" << comp_id;
    return 1.0;
  }

  // send broadcast
  if (pread_ext.param_index >= 0)
    send_parameter_ext(pread_ext.param_index, comp_id,
                       sa.group_id /*, sa.instance_id*/);
  else
    send_parameter_ext(pread_ext.param_id, comp_id,
                       sa.group_id /*, sa.instance_id*/);

  return 1.0;
}

pointprec_t MavParamProto::param_set_handler_ext(uint8_t *payload, size_t len,
                                                 SparseAddress sa,
                                                 BaseComponentPtr comp) {
  (void)len;
  (void)comp;
  mavlink_message_t *imsg = MAVPAYLOAD_TO_MAVMSG(payload);
  mavlink_message_t omsg;
  int mssleep = 20;

  mavlink_param_ext_set_t pset_ext;
  mavlink_msg_param_ext_set_decode(imsg, &pset_ext);

  string param_id =
      MavParams::toStr(pset_ext.param_id, MavParams::max_param_id_len);
  if (param_id.empty()) {
    assert(0);
    return 1.0;
  }

  int comp_id = static_cast<int>(pset_ext.target_component);
  LOG(INFO) << "SET PARAMETER EXT: " << param_id << " = "
            << pset_ext.param_value << " FOR COMPONENT WITH ID=" << comp_id;

  RouteSystemPtr roster = getRoster();
  assert(roster);
  assert(pset_ext.target_system == roster->getMcastId());

  BaseComponentPtr c = roster->getCBus().get_component(comp_id);
  if (!c) {
    LOG(INFO) << "NO COMPONENT WITH ID=" << comp_id;
    return 1.0;
  }

  MavParams::ParamUnion old_value;
  MavParams::toParamUnion(c->getParameterValue(param_id),
                          c->getParameterType(param_id), old_value);
  assert(pset_ext.param_type == old_value.type);

  MavParams::ParamUnion new_value;
  new_value.type = pset_ext.param_type;
  mem_copy(new_value.bytes, sizeof(new_value.bytes), pset_ext.param_value,
           sizeof(pset_ext.param_value), sizeof(new_value.bytes));

  bool result = c->updateParameter(param_id, new_value);

  mavlink_param_ext_ack_t param_ack;
  mem_copy(param_ack.param_id, sizeof(param_ack.param_id), pset_ext.param_id,
           sizeof(pset_ext.param_id), sizeof(param_ack.param_id));
  param_ack.param_type = pset_ext.param_type;

  if (result) {
    mem_copy(param_ack.param_value, sizeof(param_ack.param_value),
             pset_ext.param_value, sizeof(pset_ext.param_value),
             sizeof(param_ack.param_value));
    param_ack.param_result = PARAM_ACK_ACCEPTED;
  } else {
    MavParams::ParamUnion current_value;
    MavParams::toParamUnion(c->getParameterValue(param_id),
                            c->getParameterType(param_id), current_value);
    mem_copy(param_ack.param_value, sizeof(param_ack.param_value),
             current_value.bytes, sizeof(current_value.bytes),
             sizeof(param_ack.param_value));
    param_ack.param_result = PARAM_ACK_FAILED;
  }

  mavlink_msg_param_ext_ack_encode(roster->getMcastId(), comp_id, &omsg,
                                   &param_ack);

  this_thread::sleep_for(chrono::milliseconds(mssleep));
  // broadcast
  send_mavlink_message(omsg, comp_id, sa.group_id /*, sa.instance_id*/);

  return 1.0;
}

void MavParamProto::send_parameter(int param_index, int src_comp_id,
                                   int dst_sys_id, int dst_comp_id) {
  RouteSystemPtr roster = getRoster();
  BaseComponentPtr comp = roster->getCBus().get_component(src_comp_id);
  assert(comp);

  send_parameter(comp->getParameterId(param_index).c_str(), src_comp_id,
                 dst_sys_id, dst_comp_id);
}

void MavParamProto::send_parameter(const char *param_id, int src_comp_id,
                                   int dst_sys_id, int dst_comp_id) {
  if (param_id && param_id[0] != '\0') {
    mavlink_param_value_t param_value;
    mavlink_message_t msg;
    int mssleep = 20;

    RouteSystemPtr roster = getRoster();
    BaseComponentPtr comp = roster->getCBus().get_component(src_comp_id);
    assert(comp);

    string param_id_str =
        MavParams::toStr(param_id, MavParams::max_param_id_len);

    param_value.param_count = 1;
    int param_index = comp->getParameterIdx(param_id_str);
    if (param_index < 0) return;

    param_value.param_index = static_cast<uint16_t>(param_index);

    mem_copy(param_value.param_id, sizeof(param_value.param_id),
             param_id_str.c_str(), param_id_str.size() + 1,
             sizeof(param_value.param_id));

    int type = comp->getParameterType(param_id_str);
    if (type > 0 && type < MAV_PARAM_TYPE_ENUM_END) {
      param_value.param_type = static_cast<uint8_t>(type);

      string value = comp->getParameterValue(param_id_str);
      MavParams::ParamUnion u;
      MavParams::toParamUnion(value, type, u);

      if (decodeParameterValue(param_value, type, u)) {
        mavlink_msg_param_value_encode(roster->getMcastId(), src_comp_id, &msg,
                                       &param_value);

        this_thread::sleep_for(chrono::milliseconds(mssleep));
        send_mavlink_message(msg, src_comp_id, dst_sys_id, dst_comp_id);
      }
    }
  }
}

void MavParamProto::send_parameters(int src_comp_id, int dst_sys_id,
                                    int dst_comp_id) {
  RouteSystemPtr roster = getRoster();
  BaseComponentPtr comp = roster->getCBus().get_component(src_comp_id);
  assert(comp);

  LOG(INFO) << "REQUESTED ALL PARAMETERS FOR COMPONENT WITH ID="
            << static_cast<int>(comp->getId());

  const auto &param_id_to_value = comp->getParameterList();

  if (param_id_to_value.size()) {
    mavlink_param_value_t param_value;
    int mssleep = 20, send_cntr = 0;

    param_value.param_count = param_id_to_value.size();
    assert(param_value.param_count == comp->getParameterCount());

    for (auto &x : param_id_to_value) {
      string param_id = MavParams::toStr(x.first.c_str(), x.first.size());
      mavlink_message_t msg;

      int param_index = comp->getParameterIdx(param_id);
      param_value.param_index = static_cast<uint16_t>(param_index);
      assert(param_index >= 0 && param_index < param_value.param_count);

      mem_copy(param_value.param_id, sizeof(param_value.param_id),
               x.first.c_str(), x.first.size() + 1,
               sizeof(param_value.param_id));

      int type = comp->getParameterType(param_id);
      if (type > 0 && type < MAV_PARAM_TYPE_ENUM_END) {
        param_value.param_type = static_cast<uint8_t>(type);

        string value = x.second;
        MavParams::ParamUnion u;
        MavParams::toParamUnion(value, type, u);

        bool result = decodeParameterValue(param_value, type, u);
        if (result) {
          mavlink_msg_param_value_encode(roster->getMcastId(), src_comp_id,
                                         &msg, &param_value);

          this_thread::sleep_for(chrono::milliseconds(mssleep));
          send_mavlink_message(msg, src_comp_id, dst_sys_id, dst_comp_id);
          ++send_cntr;
        }
      }
    }

    LOG(INFO) << "COMPONENT WITH ID=" << static_cast<int>(comp->getId())
              << " SENT " << send_cntr << " PARAMETERS OF TOTAL "
              << param_id_to_value.size();
  } else {
    LOG(INFO) << "COMPONENT WITH ID=" << static_cast<int>(comp->getId())
              << " HAS NO REGISTERED PARAMETERS";
  }
}

pointprec_t MavParamProto::param_request_list_handler(uint8_t *payload,
                                                      size_t len,
                                                      SparseAddress sa,
                                                      BaseComponentPtr comp) {
  (void)len;
  (void)comp;

  mavlink_message_t *msg = MAVPAYLOAD_TO_MAVMSG(payload);
  mavlink_param_request_list_t param_list;
  mavlink_msg_param_request_list_decode(msg, &param_list);

  RouteSystemPtr roster = getRoster();
  assert(roster);
  assert(param_list.target_system == roster->getMcastId());

  if (param_list.target_component == MAV_COMP_ID_ALL) {
    LOG(INFO) << "BROADCAST REQUEST OF ALL PARAMETERS";

    for (auto it = roster->getCBus().begin(); it != roster->getCBus().end();
         ++it) {
      // broadcast according to protocol
      send_parameters(it->first, sa.group_id /*, sa.instance_id*/);
    }
  } else {
    // must not enter this branch normally
    int comp_id = static_cast<int>(param_list.target_component);

    if (!roster->getCBus().has_component(comp_id)) {
      LOG(INFO) << "NO COMPONENT WITH ID=" << comp_id;
      return 1.0;
    }

    send_parameters(comp_id, sa.group_id /*, sa.instance_id*/);
  }

  return 1.0;
}

pointprec_t MavParamProto::param_request_read_handler(uint8_t *payload,
                                                      size_t len,
                                                      SparseAddress sa,
                                                      BaseComponentPtr comp) {
  (void)len;
  (void)comp;
  mavlink_message_t *msg = MAVPAYLOAD_TO_MAVMSG(payload);

  mavlink_param_request_read_t pread;
  mavlink_msg_param_request_read_decode(msg, &pread);

  RouteSystemPtr roster = getRoster();
  assert(roster);
  assert(pread.target_system == roster->getMcastId());

  LOG(INFO) << "REQUESTED PARAMETER READ FOR COMPONENT WITH ID="
            << static_cast<int>(pread.target_component);

  int comp_id = static_cast<int>(pread.target_component);
  if (!roster->getCBus().has_component(comp_id)) {
    LOG(INFO) << "NO COMPONENT WITH ID=" << comp_id;
    return 1.0;
  }

  // send broadcast
  if (pread.param_index >= 0)
    send_parameter(pread.param_index, comp_id,
                   sa.group_id /*, sa.instance_id*/);
  else
    send_parameter(pread.param_id, comp_id, sa.group_id /*, sa.instance_id*/);

  return 1.0;
}

pointprec_t MavParamProto::param_set_handler(uint8_t *payload, size_t len,
                                             SparseAddress sa,
                                             BaseComponentPtr comp) {
  (void)len;
  (void)comp;
  mavlink_message_t *imsg = MAVPAYLOAD_TO_MAVMSG(payload);

  mavlink_param_set_t pset;
  mavlink_msg_param_set_decode(imsg, &pset);

  string param_id =
      MavParams::toStr(pset.param_id, MavParams::max_param_id_len);
  if (param_id.empty()) {
    assert(0);
    return 1.0;
  }

  int comp_id = static_cast<int>(pset.target_component);
  LOG(INFO) << "SET PARAMETER: " << param_id << " = " << pset.param_value
            << " FOR COMPONENT WITH ID=" << comp_id;

  RouteSystemPtr roster = getRoster();
  assert(roster);
  assert(pset.target_system == roster->getMcastId());

  BaseComponentPtr c = roster->getCBus().get_component(comp_id);
  if (!c) {
    LOG(INFO) << "NO COMPONENT WITH ID=" << comp_id;
    return 1.0;
  }

  MavParams::ParamUnion old_value;
  MavParams::toParamUnion(c->getParameterValue(param_id),
                          c->getParameterType(param_id), old_value);

  MavParams::ParamUnion new_value;
  new_value.param_float = pset.param_value;
  new_value.type = pset.param_type;
  assert(pset.param_type == old_value.type);

  bool result = c->updateParameter(param_id, new_value);
  if (!result) {
    // just a check
    MavParams::ParamUnion current_value;
    MavParams::toParamUnion(c->getParameterValue(param_id),
                            c->getParameterType(param_id), current_value);
    assert(current_value.param_float == old_value.param_float);
  }

  // send updated value
  send_parameter(param_id.c_str(), comp_id, sa.group_id /*, sa.instance_id*/);

  return 1.0;
}

bool MavParamProto::decodeParameterValue(mavlink_param_value_t &param_value,
                                         int param_type,
                                         const MavParams::ParamUnion &u) {
  bool result = true;

  switch (static_cast<MAV_PARAM_TYPE>(param_type)) {
    case MAV_PARAM_TYPE_UINT8:
      param_value.param_value = u.param_uint8;
      break;
    case MAV_PARAM_TYPE_INT8:
      param_value.param_value = u.param_int8;
      break;
    case MAV_PARAM_TYPE_UINT16:
      param_value.param_value = u.param_uint16;
      break;
    case MAV_PARAM_TYPE_INT16:
      param_value.param_value = u.param_int16;
      break;
    case MAV_PARAM_TYPE_UINT32:
      param_value.param_value = u.param_uint32;
      break;
    case MAV_PARAM_TYPE_INT32:
      param_value.param_value = u.param_int32;
      break;
    case MAV_PARAM_TYPE_REAL32:
      param_value.param_value = u.param_float;
      break;
    case MAV_PARAM_TYPE_REAL64:
    case MAV_PARAM_TYPE_UINT64:
    case MAV_PARAM_TYPE_INT64:
    default:
      LOG(ERROR) << "PARAMETER TYPE=" << param_type
                 << " IS NOT SUPPORTED UNDER BASE PARAMETER PROTOCOL";
      result = false;
  }

  return result;
}
