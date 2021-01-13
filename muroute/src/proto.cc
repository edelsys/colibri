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

namespace fflow {
AbstractEdgeInterface::AbstractEdgeInterface() : recv_cb(nullptr) {}

/*virtual*/ AbstractEdgeInterface::~AbstractEdgeInterface() {}
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

  native_addr_t naddr;
  naddr.typ = FLOWADDR_UNICAST;
  if (recv_cb) {
    recv_cb(edge_id, msg, naddr);
  }
}

/*virtual*/ bool LoopBackInterface::open(const std::string & /*iface*/,
                                         uint32_t /*port*/) {
  return true;
}

}  // namespace fflow
