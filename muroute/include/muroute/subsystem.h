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

#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

#include <muflow/sparseutils.h>

#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "componentbus.h"
#include "mavlink2/mavlink_types.h"
#include "proto.h"

#define MAX_MAVLINK_MESSAGE_SIZE 1024

#define MAVPAYLOAD_TO_MAVMSG(payload)     \
  (reinterpret_cast<mavlink_message_t *>( \
      payload - offsetof(mavlink_message_t, payload64)))

namespace fflow {

typedef struct __message_handler_note {
  uint32_t id;
  std::function<fflow::pointprec_t(uint8_t *, size_t, fflow::SparseAddress,
                                   BaseComponentPtr)>
      handler;
} message_handler_note_t;

// <channel, data, len, result>
// [[deprecated]]
typedef std::function<uint8_t *(uint32_t, const uint8_t *, size_t,
                                pointprec_t &)>
    f_protocol_handler;

// < weight (less is better), native addr, edge interface pointer>
typedef std::tuple<uint32_t, native_addr_t,
                   std::shared_ptr<AbstractEdgeInterface>>
    route_row_t;

/// ////////////////////////////////////
/// \brief The IMavlinkProtocol class
///
class IMavlinkProtocol {
 public:
  virtual ~IMavlinkProtocol() {}
  virtual void setRoster(fflow::RouteSystemPtr) = 0;
  virtual fflow::RouteSystemPtr getRoster() = 0;
  virtual const fflow::message_handler_note_t *get_table() const = 0;
  virtual size_t get_table_len() const = 0;
};

/// ////////////////////////////////////
/// \brief The RouteSystem class
///
class RouteSystem final : public BaseComponent {
  // these are using __send()
  friend class BaseComponent;
  friend class MavParamProto;

  class BridgeComponent : public BaseComponent {
   public:
    BridgeComponent() { setId(MAV_COMP_ID_PATHPLANNER); }
    virtual ~BridgeComponent() {}
  };

  RouteSystem();
  void setMcompId();

 public:
  static const char PARAMETER_FORWARDING[];

 public:
  static RouteSystemPtr createRouteSys() {
    return std::shared_ptr<RouteSystem>(new RouteSystem());
  }

  virtual ~RouteSystem();

  /// **************************************
  /// Component API
  int addComponent(BaseComponentPtr);
  void removeComponent(int);
  BaseComponentPtr getComponent(int);

  /// **************************************
  /// Protocol API
  [[deprecated]] int add_protocol(f_protocol_handler);

  int add_protocol2(const message_handler_note_t *, size_t);

  /// **************************************
  /// Communication
  // [[deprecated]]
  void sendmavmsg(mavlink_message_t &, const std::list<SparseAddress> &) const;

  bool sendmsg(uint8_t *, size_t, uint8_t, const std::list<SparseAddress> &);

  uint32_t add_edge_transport(std::shared_ptr<AbstractEdgeInterface>);

  int receive(uint32_t, std::vector<uint8_t> &, const native_addr_t &);

  /// **************************************
  ///  Getters/Setters
  uint32_t getMcastId() const { return mcastId; }

  void setMcastId(const uint32_t &);

  uint32_t getMcompId() const { return getId(); }

  static size_t muproto_message_pack(uint32_t, const std::vector<uint8_t> &,
                                     mavlink_message_t &);

  bool setForwarding(bool onoff) {
    forwarding_ = onoff;
    return forwarding_;
  }

  bool getForwarding() const { return forwarding_; }

  void printRouteTable() const;

  ComponentBus &getCBus() { return cbus; }

 private:
  bool startImpl() override;
  void stopImpl() override;
  bool updateParameterImpl(const std::string &, const ParamUnion &) override;

  void __send(mavlink_message_t &, const SparseAddress &,
              const std::list<SparseAddress> &, bool forwarded = false) const;
  inline bool __is_local_addr(const SparseAddress &) const;
  inline bool __is_forward_addr(const SparseAddress &) const;
  inline bool __is_bcast_addr(const SparseAddress &) const;

  SparseAddress __mandatory_proto_cb(uint32_t, const uint8_t *, size_t,
                                     pointprec_t &);

  // [[deprecated]]
  std::vector<f_protocol_handler> protoqueue;

  /// < guards
  mutable std::mutex recvlock;
  mutable std::mutex routelock;
  mutable std::mutex edgeslock;

  // instance id to routing info
  mutable std::unordered_map<uint64_t, route_row_t> route_table;

  std::unordered_map<uint32_t, std::shared_ptr<AbstractEdgeInterface>> edges;

  /// < counters
  uint32_t nextEdgeId;  // ID counter

  /// < params
  bool forwarding_;  // alien message forwarding enabled/disabled
  uint32_t mcastId;  // system id / which is multicast address

  /// < protocols
  std::unordered_map<
      uint32_t,
      std::vector<std::function<fflow::pointprec_t(
          uint8_t *, size_t, fflow::SparseAddress, BaseComponentPtr)>>>
      all_msg_handlers;

  /// < default protocols (microservices)
  std::unique_ptr<IMavlinkProtocol> param_proto;

  /// < local components list (loopback)
  //  std::unordered_set<uint32_t>
  //      componentIDTable;  // table of IDs of local components

  /// < component bus holding all components of the system, including router
  /// itself
  ComponentBus cbus;

  uint32_t loopBackEdgeId;  // ID of loopback edge transport
};

std::shared_ptr<fflow::AbstractEdgeInterface> createEdgeFunctionByName(
    const std::string &);

}  // namespace fflow

#endif  // SUBSYSTEM_H
