/*
 * Copyright 2019-2020, EDEL LLC <http://www.edelsys.com>
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

#include "muroute/subsystem.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <utility>

#include "muqueue/erqperiodic.h"
#include "muqueue/scheduler.h"
#include "muroute/mavlink2/mavlink_types.h"
#include "muroute/mavlink2/standard/mavlink.h"
#include "muroute/mavparamproto.h"
#include "muroute/muroute.h"

using namespace std;

namespace fflow {

/// ************************************************************************************************

void BaseMavlinkProtocol::send_ack(int cmd, bool result, int src_comp_id,
                                   int dst_sys_id, int dst_comp_id) {
  mavlink_message_t msg;

  if (nullptr == roster_) {
    assert(0);
    return;
  }

  mavlink_msg_command_ack_pack(roster_->getMcastId(), src_comp_id, &msg, cmd,
                               result ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED,
                               0, 0, dst_sys_id, dst_comp_id);

  send_mavlink_message(msg, src_comp_id, dst_sys_id, dst_sys_id);
}

void BaseMavlinkProtocol::send_mavlink_message(mavlink_message_t &msg,
                                               int src_comp_id, int dst_sys_id,
                                               int dst_comp_id) {
  uint8_t buffer[MAX_MAVLINK_MESSAGE_SIZE];
  uint8_t *data = buffer;

  if (nullptr == roster_) {
    assert(0);
    return;
  }

  size_t len = mavlink_msg_to_send_buffer(data, &msg);
  uint32_t src_sys_id = roster_->getMcastId();

  if (len > 0) {
    LOG(INFO) << "Sending from <" << src_sys_id << ":" << src_comp_id << "> to "
              << "<" << dst_sys_id << ":" << dst_comp_id << ">";
    const auto src = SparseAddress(src_sys_id, src_comp_id, 0);
    auto dst = fflow::SparseAddress(dst_sys_id, dst_comp_id, 0);
    roster_->__send(msg, src, {dst});
  } else {
    LOG(ERROR) << "Failed to send message from <" << src_sys_id << ":"
               << src_comp_id << "> to "
               << "<" << dst_sys_id << ":" << dst_comp_id << ">";
  }
}

const char RouteSystem::PARAMETER_FORWARDING[] = "Forwarding";

/// ************************************************
/// \brief RouteSystem::RouteSystem
/// ************************************************
RouteSystem::RouteSystem()
    : BaseComponent::BaseComponent(MAV_COMP_ID_ONBOARD_COMPUTER),
      nextEdgeId(0),
      forwarding_(false) {
  route_table.clear();
  edges.clear();
  protoqueue.clear();

  // setup default protocols
  param_proto = make_unique<MavParamProto>();
  param_proto->setRoster(shared_ptr<RouteSystem>(this));
  add_protocol2(param_proto->get_table(), param_proto->get_table_len());

  shared_ptr<fflow::AbstractEdgeInterface> lb =
      make_shared<LoopBackInterface>();
  loopBackEdgeId = add_edge_transport(lb);

  setMcompId();

  setParameterValue(PARAMETER_FORWARDING, getForwarding());

  addComponent(this);
  addComponent(new BridgeComponent());
}

/*virtual*/ RouteSystem::~RouteSystem() {
  BridgeComponent *bptr =
      dynamic_cast<BridgeComponent *>(getComponent(MAV_COMP_ID_PATHPLANNER));
  cbus.clear();
  if (bptr != nullptr) delete bptr;
}

/// ************************************************
/// \brief RouteSystem::add_protocol
/// \param proto
/// \return
/// ************************************************
int RouteSystem::add_protocol(f_protocol_handler proto) {
  lock_guard<mutex> lock(routelock);
  protoqueue.push_back(proto);
  return 0;
}

/// ************************************************
/// \brief RouteSystem::add_protocol2
/// \param proto
/// \param len
/// \return
/// ************************************************
int RouteSystem::add_protocol2(const message_handler_note_t *proto,
                               size_t len) {
  for (size_t i = 0; i < len; i++) {
    auto note = proto[i];
    all_msg_handlers[note.id].emplace_back(note.handler);
  }
  return 0;
}

/// ************************************************
/// \brief RouteSystem::printRouteTable
/// ************************************************
void RouteSystem::printRouteTable() const {
  for (const auto &c : route_table) {
    cout << "|-----------------------\n";
    cout << "|" << c.first << ": " << get<0>(c.second) << " * "
         << get<1>(c.second).typ << " * " << get<1>(c.second).len << " * "
         << get<2>(c.second).get();
    cout << "|-----------------------\n";
  }
}

/// ************************************************************************************************

///
/// \brief RouteSystem::__is_bcast_addr
/// \param addr
/// \return
///
bool RouteSystem::__is_bcast_addr(const SparseAddress &addr) const {
  if (addr.group_id == 0 || addr.instance_id == MAV_COMP_ID_ALL) {
    return true;
  }
  return false;
}

/// ************************************************
/// \brief __is_local_addr
/// \param addr
/// \return
/// ************************************************
bool RouteSystem::__is_local_addr(const SparseAddress &addr) const {
  if ((addr.group_id == getMcastId() || addr.group_id == 0 /* BCAST */) &&
      (cbus.has_component(addr.instance_id) ||
       addr.instance_id == MAV_COMP_ID_ALL)) {
    return true;
  }
  return false;
}

/// ************************************************
/// \brief RouteSystem::__is_forward_addr
/// \param addr
/// \return
/// ************************************************
bool RouteSystem::__is_forward_addr(const SparseAddress &addr) const {
  if (addr.group_id == 0) return true;
  if (addr.group_id !=
      getMcastId() /** @todo CHECK if target systems was observed before*/)
    return true;
  if (addr.group_id == getMcastId() &&
      !cbus.has_component(addr.instance_id)) /** @todo CHECK if component was
                       observed before. As soon as we employ loopback interface
                       there is no need to check locality of component */
    return true;
  return false;
}

#define SPARSE_ADDR_TO_UINT64(x, y) \
  ((uint64_t(x) & 0xffffffff) | ((uint64_t(y) & 0xffffffff) << 32))

#define SYSID_FROM_UINT64_ADDR(a) \
  ((uint64_t(x) & 0xffffffff) | ((uint64_t(y) & 0xffffffff) << 32))

#define COMPID_FROM_UINT64_ADDR(a) \
  ((uint64_t(x) & 0xffffffff) | ((uint64_t(y) & 0xffffffff) << 32))

/*
#define UINT64_TO_SPARSE_ADDR(x, y) \
 ((uint64_t(x) & 0xffffffff) | ((uint64_t(y) & 0xffffffff) << 32))
*/

/// ************************************************
/// \brief RouteSystem::__send
/// \param msg
/// \param srcAddr
/// \param dst
/// \param forwarded
/// ************************************************
void RouteSystem::__send(mavlink_message_t &msg, const SparseAddress &srcAddr,
                         const list<fflow::SparseAddress> &dst,
                         bool forwarded) const {
  bool has_bcast = false;
  for (const auto &d : dst) {
    if (__is_bcast_addr(d)) {
      has_bcast = true;
      break;
    }
  }

  //--------------- GET native addr of source -------------
  uint64_t fromaddr64 =
      SPARSE_ADDR_TO_UINT64(srcAddr.group_id, srcAddr.instance_id);

  // identify source of a message
  native_addr_t srcna = {FLOWADDR_NONE, 0, {}};
  uint32_t src_edge_id = -1;

  unordered_map<uint64_t, route_row_t>::iterator route;
  {
    lock_guard<mutex> lock(routelock);
    route = route_table.find(fromaddr64);

    if (route != route_table.end()) {
      auto ifc = get<2>(route->second);
      if (!ifc) {
        VLOG(2) << "Route table corrupted! Exiting...";
        printRouteTable();
        assert(false);
      }

      auto ifc_id = get<1>(route->second);
      src_edge_id = ifc->edge_id;
      srcna = ifc_id;
    }
  }

  // prepare message
  uint8_t crc_extra =
      mavlink_get_crc_extra(&msg);  //< get extra crc for given message
  uint8_t lmin =
      mavlink_min_message_length(&msg);  //< get min length for given message

  if (has_bcast) {
    native_addr_t na = {FLOWADDR_BROADCAST, 0, {}};
    VLOG_IF(4, true) << " *** ---------------";

    //------------------
    // make copy for safety
    unordered_map<uint32_t, shared_ptr<AbstractEdgeInterface>> _edges;

    {
      //      _edges.reserve(edges.size());
      lock_guard<mutex> lock(edgeslock);
      for (const auto &edg : edges) _edges.emplace(edg);
    }

    for (const auto &edgei : _edges) {
      VLOG_IF(4, true) << "Interface is in use when broadcasting: "
                       << edgei.second->name << edgei.second->edge_id << " ** "
                       << srcAddr.group_id << ":" << srcAddr.instance_id
                       << " ## " << dst.front().group_id << ":"
                       << dst.front().instance_id;

      // messages length control variable
      uint16_t finlen = 0;

      // allocate buffer which is knowingly bigger
      vector<uint8_t> rawbuf(sizeof(mavlink_message_t));

      // if message is not forwarded will do required finalization
      if (!forwarded)
        finlen = mavlink_finalize_message_chan(
            &msg, srcAddr.group_id, srcAddr.instance_id, edgei.second->edge_id,
            lmin, msg.len, crc_extra);

      // serialize into raw buffer
      finlen = mavlink_msg_to_send_buffer(&rawbuf[0], &msg);

      assert(finlen <= rawbuf.size());

      // trim buffer to actual message size on wire
      rawbuf.resize(finlen);

      // send to underlyling transport
      edgei.second->sendtoraw(rawbuf, na, srcna, src_edge_id);

      VLOG(6) << "Sent broadcast packet " << msg.seq;
    }
  } else
    // iterate over recipients
    for (const auto &dstaddr : dst) {
      unordered_map<uint64_t, route_row_t>::iterator route = route_table.end();
      uint64_t dstaddr64 =
          SPARSE_ADDR_TO_UINT64(dstaddr.group_id, dstaddr.instance_id);

      {
        lock_guard<mutex> lock(routelock);
        route = route_table.find(dstaddr64);
      }

      if (route == route_table.end()) {
        VLOG(4) << "Destination unreachable... " << dstaddr.group_id << ":"
                << dstaddr.instance_id << " continue";
        continue;
      }
      native_addr_t na = get<1>(route->second);

      // get edge interface (a.k.a transport) pointer to map to channel
      auto edgei = get<2>(route->second);

      // messages length control variable
      uint16_t finlen = 0;

      // allocate buffer which is knowingly bigger
      vector<uint8_t> rawbuf(sizeof(mavlink_message_t));

      // if message is not forwarded will do required finalization
      if (!forwarded)
        finlen = mavlink_finalize_message_chan(
            &msg, srcAddr.group_id, srcAddr.instance_id, edgei->edge_id, lmin,
            msg.len, crc_extra);

      // serialize into raw buffer
      finlen = mavlink_msg_to_send_buffer(&rawbuf[0], &msg);

      assert(finlen <= rawbuf.size());

      // trim buffer to actual message size on wire
      rawbuf.resize(finlen);

      edgei->sendtoraw(rawbuf, na, srcna, src_edge_id);
    }
}

/// ************************************************
/// \brief RouteSystem::sendmavmsg
/// \param msg
/// \param to
/// ************************************************
void RouteSystem::sendmavmsg(mavlink_message_t &msg,
                             const list<SparseAddress> &to) const {
  __send(msg, SparseAddress(mcastId, getId(), 0), to);
}

/// ************************************************
/// \brief RouteSystem::sendmsg
/// \param data
/// \param bytes
/// \param mid
/// \param to
/// \return
/// ************************************************
bool RouteSystem::sendmsg(uint8_t *data, size_t bytes, uint8_t mid,
                          const list<SparseAddress> &to) {
  mavlink_message_t msg;
  msg.msgid = mid;

  copy(&data[0], &data[0] + bytes, &msg.payload64[0]);  // copy payload
  msg.len = uint8_t(bytes);

  __send(msg, SparseAddress(mcastId, getId(), 0), to);

  return true;
}

/// ************************************************
/// \brief RouteSystem::add_edge_transport
/// \param tr
/// \return
/// ************************************************
uint32_t RouteSystem::add_edge_transport(shared_ptr<AbstractEdgeInterface> tr) {
  //  transports.push_back(tr);

  // assign edge a name
  tr->edge_id = nextEdgeId++;

  {
    lock_guard<mutex> lock(edgeslock);
    edges[tr->edge_id] = tr;
  }

  tr->registerCallback(
      [this](uint32_t a, vector<uint8_t> &b, const native_addr_t &c) -> int {
        return receive(a, b, c);
      });

  return tr->edge_id;
}

/// ************************************************
/// \brief RouteSystem::receive
/// \param edge_id
/// \param msgdata
/// \param from
/// \return
/// ************************************************
int RouteSystem::receive(uint32_t edge_id, vector<uint8_t> &msgdata,
                         const native_addr_t &from) {
  shared_ptr<AbstractEdgeInterface> __aeiptr;
  {
    lock_guard<mutex> lock(edgeslock);
    auto edgei = edges.find(edge_id);

    if (edgei == edges.end()) {
      assert(false);
      return -1;
    }
    __aeiptr = edgei->second;
  }

  pointprec_t protoweight = 1.0;  // analogue of error

  // mandatory proto
  auto sparsaddr =
      __mandatory_proto_cb(edge_id, &msgdata[0], msgdata.size(), protoweight);

  // check update routing
  if (sparsaddr.valid()) {
    uint64_t newaddr =
        SPARSE_ADDR_TO_UINT64(sparsaddr.group_id, sparsaddr.instance_id);

    assert(__aeiptr);

    {
      lock_guard<mutex> lock(__aeiptr->feasaddrlock);
      __aeiptr->feasable_addrs.insert(newaddr);
    }

    uint32_t route_weight = (1.0 - protoweight) * 100.0;
    route_row_t routenote = make_tuple(route_weight, from, __aeiptr);

    fflow::post_function<void>([newaddr, routenote, this]() -> void {
      lock_guard<mutex> lock(routelock);
      /// always update
      route_table[newaddr] = routenote;
    });
  }

  // all other protocols
  for (const auto &proto : protoqueue) {
    __UNUSED__ auto ptr =
        proto(edge_id, &msgdata[0], msgdata.size(), protoweight);
  }

  return 0;

}  // namespace fflow

/// ************************************************
/// \brief RouteSystem::__mandatory_proto_cb
/// \param chan
/// \param ptr
/// \param len
/// \param outerr
/// \return
/// ************************************************
fflow::SparseAddress RouteSystem::__mandatory_proto_cb(
    uint32_t chan, const uint8_t *ptr, size_t len, pointprec_t & /*outerr*/) {
  mavlink_message_t msg;
  mavlink_status_t status;

  uint32_t mcast_id = -1;  // equivalient to mavlink SysId
  uint32_t inst_id = -1;   // equivalent to mavlink CompId, used as an
                           // instance_id in SparseAddress

  // Make target sparse address
  fflow::SparseAddress srcAddr = fflow::SparseAddress(mcast_id, inst_id, 0);

  for (size_t i = 0; i < len; i++) {
    if (mavlink_parse_char(chan, ptr[i], &msg, &status) ==
        MAVLINK_FRAMING_OK) {  // message received
      mavlink_message_t *rxmsg = reinterpret_cast<mavlink_message_t *>(&msg);

      /// @todo Check sequence number consistency int(rxmsg->seq)

      // Update source address
      srcAddr.group_id = uint32_t(rxmsg->sysid);
      srcAddr.instance_id = uint32_t(rxmsg->compid);

      uint32_t __tgtSysId = 0;
      uint32_t __tgtCompId = 0;

      // Try to get target id's
      const mavlink_msg_entry_t *msg_info = mavlink_get_msg_entry(rxmsg->msgid);
      if (msg_info != NULL && msg_info->target_system_ofs) {  // ok, get id's
        uint8_t *payload8 = reinterpret_cast<uint8_t *>(&rxmsg->payload64[0]);
        __tgtSysId = payload8[msg_info->target_system_ofs];
        __tgtCompId = payload8[msg_info->target_component_ofs];
      }

      // Make target sparse address
      fflow::SparseAddress tgtAddr(__tgtSysId, __tgtCompId, 0);

      // forward messages
      if (getForwarding() && __is_forward_addr(tgtAddr) &&
          (chan != loopBackEdgeId)  // check is not loopback
      ) {
        // allocate new message for async ops
        shared_ptr<mavlink_message_t> pmsg = make_shared<mavlink_message_t>();

        memcpy(static_cast<void *>(pmsg.get()), static_cast<void *>(&msg),
               sizeof(msg));

        uint32_t from_group_id =
            srcAddr.group_id;  // id of function flow instance
        uint32_t from_instance_id =
            srcAddr.instance_id;  // id of function flow instance
        uint32_t to_group_id =
            tgtAddr.group_id;  // id of function flow instance
        uint32_t to_instance_id =
            tgtAddr.instance_id;  // id of function flow instance

        // do actual forward with dummy QOS filtering for parameter protocol
        int priorityQOS = 0;
        if ((pmsg->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) ||
            (pmsg->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) ||
            (pmsg->msgid == MAVLINK_MSG_ID_PARAM_SET) ||
            (pmsg->msgid == MAVLINK_MSG_ID_PARAM_VALUE) ||
            (pmsg->msgid == MAVLINK_MSG_ID_PARAM_EXT_VALUE) ||
            (pmsg->msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST) ||
            (pmsg->msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ) ||
            (pmsg->msgid == MAVLINK_MSG_ID_PARAM_EXT_SET)) {
          priorityQOS = 1;
        }
        fflow::post_function<void>(
            [this, pmsg, from_group_id, from_instance_id, to_group_id,
             to_instance_id](void) -> void {
              fflow::SparseAddress __srcAddr(from_group_id, from_instance_id,
                                             0);
              fflow::SparseAddress __tgtAddr(to_group_id, to_instance_id, 0);

              __send(*pmsg, __srcAddr, {__tgtAddr}, true);
            },
            priorityQOS);

      } else {
        VLOG_IF(5, true) << "Forwarding is not enabled";
      }

      if (__is_local_addr(tgtAddr)) {
        VLOG_IF(5, true) << "Process locally" << int(rxmsg->seq);

        bool run_handlers = true;
        BaseComponentPtr tgtComp = nullptr;
        if (tgtAddr.instance_id != MAV_COMP_ID_ALL) {
          tgtComp = getCBus().get_component(tgtAddr.instance_id);
          if (!tgtComp) run_handlers = false;
        }

        if (run_handlers) {
          for (const auto &entry : all_msg_handlers[rxmsg->msgid])
            /// < @todo Use component interface instead of tgtAddr
            entry(reinterpret_cast<uint8_t *>(&rxmsg->payload64[0]), rxmsg->len,
                  srcAddr, tgtComp);
        }
      }
    }  // if
  }    // for

  return move(srcAddr);
}

void RouteSystem::setMcompId() {
  uint32_t mcompId = getId();

  /// @TODO Make this a function call
  // update routing info
  uint64_t newaddr = SPARSE_ADDR_TO_UINT64(mcastId, mcompId);

  shared_ptr<AbstractEdgeInterface> __aaiptr;
  {
    lock_guard<mutex> lock(edgeslock);

    auto edgei = edges.find(loopBackEdgeId);

    if (edgei == edges.end()) {
      assert(false);
      return;
    }

    __aaiptr = edgei->second;
  }

  __aaiptr->feasable_addrs.insert(newaddr);

  uint32_t route_weight = (1.0 - 0.0) * 100.0;
  native_addr_t naddr;
  naddr.typ = FLOWADDR_UNICAST;
  route_row_t routenote = make_tuple(route_weight, naddr, __aaiptr);

  // fflow::post_function<void>([newaddr, routenote, this]() -> void
  {
    lock_guard<mutex> lock(routelock);
    route_table[newaddr] = routenote;
  }
  //);
}

void RouteSystem::setMcastId(const uint32_t &value) { mcastId = value; }

size_t RouteSystem::muproto_message_pack(uint32_t msg_id,
                                         const vector<uint8_t> &pload,
                                         mavlink_message_t &msg) {
  //  trim extra
  size_t siz = min(int(pload.size()), MAVLINK_MAX_PAYLOAD_LEN);

  msg.msgid = msg_id;
  msg.len = uint8_t(siz);
  copy(pload.begin(), pload.begin() + siz,
       _MAV_PAYLOAD_NON_CONST(&msg));  // copy payload
  return siz;
}

int RouteSystem::addComponent(BaseComponentPtr component) {
  if (component->getRoster() == nullptr)
    component->setRoster(RouteSystemPtr(this));
  getCBus().add_component(component);
  return 0;
}

void RouteSystem::removeComponent(int id) { getCBus().remove_component(id); }

BaseComponentPtr RouteSystem::getComponent(int id) {
  return getCBus().get_component(id);
}

bool RouteSystem::startImpl() {
  LOG(INFO) << "Router starting => id=" << static_cast<int>(getId());
  return true;
}

void RouteSystem::stopImpl() {
  LOG(INFO) << "Router stopping => id=" << static_cast<int>(getId());
}

bool RouteSystem::updateParameterImpl(const string &param_id,
                                      const MavParams::ParamUnion &value) {
  bool result = false;
  if (param_id.compare(PARAMETER_FORWARDING) == 0) {
    setForwarding(value.param_uint8);
    result = true;
  }
  return result;
}

}  // namespace fflow
