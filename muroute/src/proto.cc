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

#include "muroute/proto.h"

#include <glog/logging.h>

#include "muqueue/scheduler.h"

namespace fflow {
AbstractEdgeInterface::AbstractEdgeInterface() : recv_cb(nullptr) {}

/*virtual*/ AbstractEdgeInterface::~AbstractEdgeInterface() {}

/* public */ AbstractEdgeInterface::AcceptState
AbstractEdgeInterface::accept_msg(std::vector<uint8_t> &pbuf) const {
#if 0
  if (Log::get_max_level() >= Log::Level::TRACE) {
    log_trace(
        "AbstractEdgeInterface [%d]%s: got message %u to %d/%d from %u/%u", fd,
        _name.c_str(), pbuf->curr.msg_id, pbuf->curr.target_sysid,
        pbuf->curr.target_compid, pbuf->curr.src_sysid, pbuf->curr.src_compid);
    log_trace("\tKnown components:");
    for (const auto &id : _sys_comp_ids) {
      log_trace("\t\t%u/%u", (id >> 8), id & 0xff);
    }
  }

  // This AbstractEdgeInterface sent the message, we don't want to send it back
  // over the same channel to avoid loops: reject
  if (has_sys_comp_id(pbuf->curr.src_sysid, pbuf->curr.src_compid)) {
    return AbstractEdgeInterface::AcceptState::Rejected;
  }

  // If filter is defined and message is not in the set: discard it
  if (pbuf->curr.msg_id != UINT32_MAX && !_allowed_outgoing_msg_ids.empty() &&
      !vector_contains(_allowed_outgoing_msg_ids, pbuf->curr.msg_id)) {
    return AbstractEdgeInterface::AcceptState::Filtered;
  }

  // If filter is defined and message is in the set: discard it
  if (pbuf->curr.msg_id != UINT32_MAX && !_blocked_outgoing_msg_ids.empty() &&
      vector_contains(_blocked_outgoing_msg_ids, pbuf->curr.msg_id)) {
    return AbstractEdgeInterface::AcceptState::Filtered;
  }

  // If filter is defined and message is not in the set: discard it
  if (pbuf->curr.msg_id != UINT32_MAX && !_allowed_outgoing_src_comps.empty() &&
      !vector_contains(_allowed_outgoing_src_comps, pbuf->curr.src_compid)) {
    return AbstractEdgeInterface::AcceptState::Filtered;
  }

  // If filter is defined and message is in the set: discard it
  if (pbuf->curr.msg_id != UINT32_MAX && !_blocked_outgoing_src_comps.empty() &&
      vector_contains(_blocked_outgoing_src_comps, pbuf->curr.src_compid)) {
    return AbstractEdgeInterface::AcceptState::Filtered;
  }

  // If filter is defined and message is not in the set: discard it
  if (pbuf->curr.msg_id != UINT32_MAX &&
      !_allowed_outgoing_src_systems.empty() &&
      !vector_contains(_allowed_outgoing_src_systems, pbuf->curr.src_sysid)) {
    return AbstractEdgeInterface::AcceptState::Filtered;
  }

  // If filter is defined and message is in the set: discard it
  if (pbuf->curr.msg_id != UINT32_MAX &&
      !_blocked_outgoing_src_systems.empty() &&
      vector_contains(_blocked_outgoing_src_systems, pbuf->curr.src_sysid)) {
    return AbstractEdgeInterface::AcceptState::Filtered;
  }

  // Message is broadcast on sysid or sysid is non-existent: accept msg
  if (pbuf->curr.target_sysid == 0 || pbuf->curr.target_sysid == -1) {
    return AbstractEdgeInterface::AcceptState::Accepted;
  }

  // This AbstractEdgeInterface has the target of message (sys and comp id):
  // accept
  if (pbuf->curr.target_compid > 0 &&
      has_sys_comp_id(pbuf->curr.target_sysid, pbuf->curr.target_compid)) {
    return AbstractEdgeInterface::AcceptState::Accepted;
  }

  // This AbstractEdgeInterface has the target of message (sysid, but compid is
  // broadcast or non-existent): accept
  if ((pbuf->curr.target_compid == 0 || pbuf->curr.target_compid == -1) &&
      has_sys_id(pbuf->curr.target_sysid)) {
    return AbstractEdgeInterface::AcceptState::Accepted;
  }
  // This AbstractEdgeInterface has the sniffer_sysid: accept
  if ((sniffer_sysid != 0) && has_sys_id(sniffer_sysid)) {
    return AbstractEdgeInterface::AcceptState::Accepted;
  }

#endif
  // Reject everything else
  return AbstractEdgeInterface::AcceptState::Rejected;
}

bool AbstractEdgeInterface::allowed_by_dedup(std::vector<uint8_t> &buf) const {
  // return Mainloop::get_instance().dedup_check_msg(buf);
  return (true);
}

bool AbstractEdgeInterface::allowed_by_incoming_filters(
    std::vector<uint8_t> &buf) const {
#if 0

  // If filter is defined and message is not in the set: discard it
  if (buf->curr.msg_id != UINT32_MAX && !_allowed_incoming_msg_ids.empty() &&
      !vector_contains(_allowed_incoming_msg_ids, buf->curr.msg_id)) {
    return false;
  }

  // If filter is defined and message is in the set: discard it
  if (buf->curr.msg_id != UINT32_MAX && !_blocked_incoming_msg_ids.empty() &&
      vector_contains(_blocked_incoming_msg_ids, buf->curr.msg_id)) {
    return false;
  }

  // If filter is defined and message is not in the set: discard it
  if (!_allowed_incoming_src_comps.empty() &&
      !vector_contains(_allowed_incoming_src_comps, buf->curr.src_compid)) {
    return false;
  }

  // If filter is defined and message is in the set: discard it
  if (!_blocked_incoming_src_comps.empty() &&
      vector_contains(_blocked_incoming_src_comps, buf->curr.src_compid)) {
    return false;
  }

  // If filter is defined and message is not in the set: discard it
  if (!_allowed_incoming_src_systems.empty() &&
      !vector_contains(_allowed_incoming_src_systems, buf->curr.src_sysid)) {
    return false;
  }

  // If filter is defined and message is in the set: discard it
  if (!_blocked_incoming_src_systems.empty() &&
      vector_contains(_blocked_incoming_src_systems, buf->curr.src_sysid)) {
    return false;
  }

#endif

  // everything else seems to be allowed
  return true;
}

}  // namespace fflow

namespace fflow {

LoopBackInterface::LoopBackInterface() {
  AbstractEdgeInterface::name = "EdgeLoopback";
}

/*virtual*/ LoopBackInterface::~LoopBackInterface() {}

/*virtual*/ void LoopBackInterface::sendtoraw(std::vector<uint8_t> &msg,
                                              const native_addr_t & /*to8*/,
                                              const native_addr_t & /*from*/
                                              ,
                                              uint32_t from_edge_id) {
  if (from_edge_id == edge_id) {
    VLOG(6) << "Message Loop DETECTED";
    return;
  }

  if (recv_cb) {
    // copy message
    const std::vector<uint8_t> msgcopy = msg;
    uint32_t edge_id_ = edge_id;
    recv_cb_func_t recv_cb_ = recv_cb;
    native_addr_t naddr;
    naddr.typ = FLOWADDR_UNICAST;

    // bool res = fflow::post_function<void>(
    //     [this, msgcopy, edge_id_, naddr](void) -> void {
    recv_cb(edge_id_, msgcopy, naddr);
    // },
    // 0);
  }
}

/*virtual*/ bool LoopBackInterface::open(const std::string & /*iface*/,
                                         uint32_t /*port*/) {
  return true;
}

}  // namespace fflow
