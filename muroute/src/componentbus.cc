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

#include "muroute/componentbus.h"

#include <glog/logging.h>

#include "muroute/subsystem.h"

using namespace std;
using namespace fflow;

void BaseComponent::heartbeatCallback() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(roster_->getMcastId(), id_, &msg, MAV_TYPE_GENERIC,
                             MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0,
                             MAV_STATE_ACTIVE);
  send(msg, {SparseAddress(0, MAV_COMP_ID_ALL, 0)});
}

void BaseComponent::send(mavlink_message_t &msg,
                         const std::list<fflow::SparseAddress> &dst,
                         bool forwarded) const {
  const auto src = SparseAddress(roster_->getMcastId(), id_, 0);
  roster_->__send(msg, src, {dst}, forwarded);
}

bool BaseComponent::updateParameter(const string &name,
                                    const ParamUnion &value) {
  string old_value = getParameterValue(name);
  if (old_value.empty()) return false;

  bool success = false;
  if (updateParameterImpl(name, value)) {
    success = updateParameterValue(name, value);
    if (!success) {
      // rollback
      ParamUnion u;
      MavParams::toParamUnion(old_value, value.type, u);
      __UNUSED__ bool res = updateParameterImpl(name, u);
      assert(res);
    }
  }

  return success;
}

void ComponentBus::clear() {
  // stops all registered componets and removes them from container
  for (auto it = idToComp_.begin(); it != idToComp_.end(); ++it) {
    remove_component(it->first);
  }
}

bool ComponentBus::add_component(BaseComponentPtr component) {
  bool result = false;
  int comp_id = component->getId();

  if (!has_component(comp_id)) {
    if (component->start()) {
      idToComp_.insert(make_pair(comp_id, component));
      result = true;
      LOG(INFO) << "Component with ID=" << static_cast<int>(component->getId())
                << " added to ComponentBus=" << this;
    } else {
      LOG(ERROR) << "Failed to start component with ID=" << comp_id;
    }
  } else {
    LOG(INFO) << "Component is already present";
  }

  return result;
}

bool ComponentBus::remove_component(int comp_id) {
  bool result = false;
  const auto &it = idToComp_.find(comp_id);
  if (it != idToComp_.end()) {
    it->second->stop();
    idToComp_.erase(it);
    result = true;
    LOG(INFO) << "Component with ID=" << static_cast<int>(it->second->getId())
              << " removed from ComponentBus=" << this;
  }
  return result;
}

bool ComponentBus::has_component(int comp_id) const {
  const auto &it = idToComp_.find(comp_id);
  return (it != idToComp_.end());
}

BaseComponentPtr ComponentBus::get_component(int comp_id) {
  std::map<int, BaseComponentPtr>::iterator it = idToComp_.find(comp_id);
  if (it != idToComp_.end())
    return it->second;
  else
    return NULL;
}
