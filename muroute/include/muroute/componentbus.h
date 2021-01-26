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

#ifndef COMPONENTBUS_H
#define COMPONENTBUS_H

#include <map>

#include <muflow/sparseutils.h>
#include <muqueue/erqperiodic.h>

#include "mavlink2/common/mavlink.h"
#include "mavparams.h"

namespace fflow {

class RouteSystem;
typedef std::shared_ptr<RouteSystem> RouteSystemPtr;
typedef std::weak_ptr<RouteSystem> RouteSystemWPtr;

///
/// \brief The BaseComponent class incapsulates component id and provides a
/// heartbeat
///
/// Important:
/// 1. All inherited classes are free to override startImpl/stopImpl methods if
///    they have any logic to place there
/// 2. If BaseComponent (or the one inherited from it) is added to ComponentBus
///    with add_component method, then this component will be automatically
///    started and if it is removed with remove_component, then it will be
///    automatically stopped - no need for the BaseComponent instance to call
///    start/stop directly
///
class BaseComponent : public MavParams {
 public:
  BaseComponent() {}
  BaseComponent(uint8_t id) : id_(id) {}
  BaseComponent(fflow::RouteSystemPtr roster) : roster_(roster) {}
  BaseComponent(uint8_t id, fflow::RouteSystemPtr roster)
      : id_(id), roster_(roster) {}
  virtual ~BaseComponent() { stop(); }

 public:
  uint8_t getId() const { return id_; }
  void setId(uint8_t id) { id_ = id; }

  const fflow::RouteSystemPtr getRoster() const { return roster_; }
  void setRoster(fflow::RouteSystemPtr roster) { roster_ = roster; }

  void send(mavlink_message_t &, const std::list<fflow::SparseAddress> &,
            bool forwarded = false) const;

  bool updateParameter(const std::string &, const ParamUnion &);

 public:
  bool start() {
    if (id_ == MAV_COMP_ID_ALL || roster_ == nullptr) return false;
    if (!started_ && startImpl()) {
      startHearbeat();
      started_ = true;
    }
    return started_;
  }

  void stop() {
    if (started_) {
      stopHeartbeat();
      stopImpl();
      started_ = false;
    }
  }

 protected:
  virtual bool startImpl() { return true; }
  virtual void stopImpl() {}
  virtual bool updateParameterImpl(const std::string &, const ParamUnion &) {
    return false;
  }

 private:
  void startHearbeat() {
    fflow::add_periodic_handled<void>(
        ([&](void) -> void { heartbeatCallback(); }), 0.1, 1.0, heartbeat_erq_);
  }

  void stopHeartbeat() {
    if (heartbeat_erq_) heartbeat_erq_->removeFrom(nullptr);
  }

  void heartbeatCallback();

 private:
  uint8_t id_ = MAV_COMP_ID_ALL;
  fflow::AsyncERQPtr heartbeat_erq_ = nullptr;
  fflow::RouteSystemPtr roster_ = nullptr;
  bool started_ = false;
};

typedef BaseComponent *BaseComponentPtr;

/// *************************************************
/// \brief The ComponentBus class - components aggregator
///
/// *************************************************
class ComponentBus {
 public:
  ComponentBus() {}
  virtual ~ComponentBus() { clear(); }

 public:
  bool add_component(BaseComponentPtr);
  bool remove_component(int);
  BaseComponentPtr get_component(int);
  bool has_component(int) const;
  void clear();

  size_t size() const { return idToComp_.size(); }
  auto begin() { return idToComp_.begin(); }
  auto end() { return idToComp_.end(); }

 protected:
  std::map<int, BaseComponentPtr> idToComp_;
};

}  // namespace fflow

#endif  // COMPONENTBUS_H
