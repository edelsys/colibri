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

#include "muroute/funserial.h"

#include <glog/logging.h>

namespace fflow {

using namespace proto;

EdgeSerial::EdgeSerial() : ser_port(*this) {
  AbstractEdgeInterface::name = "EdgeSerial";
}

EdgeSerial::~EdgeSerial() {}

bool EdgeSerial::open(const std::string &iface, uint32_t port) {
  try {
    ser_port.open(iface);
    ser_port.set_option(boost::asio::serial_port_base::baud_rate(port));

    recv_top();

    // start io service
    dontuseme = std::thread([&] { run(); });
    dontuseme.detach();

  } catch (const std::exception &ex) {
    LOG(ERROR) << ex.what();
    return false;
  }

  return true;
}

/*virtual*/ void EdgeSerial::sendtoraw(std::vector<uint8_t> &msg,
                                       const native_addr_t & /*to*/,
                                       const native_addr_t & /*from*/,
                                       uint32_t from_edge_id) {
  std::array<ioas::const_buffer, 1> buf;
  buf[0] = ioas::buffer(&msg[0], msg.size());

  if (from_edge_id == edge_id) {
    VLOG(6) << "Message Loop DETECTED";
    return;
  }

  ser_port.async_write_some(
      buf, [&](const ioas_err_t &error, std::size_t bytes_transferred) -> void {
        if (error) {
          LOG(WARNING) << __FILE__ << ": " << __FUNCTION__ << __LINE__
                       << " error: " << error.message() << " : = > "
                       << " wrote " << bytes_transferred;
        }
      });
}

void EdgeSerial::recv_top() {
  packet_buffer.alloc(TR_MAX_PAYLOAD_LEN);
  std::array<ioas::mutable_buffer, 1> buf;
  buf[0] =
      ioas::buffer(&packet_buffer.msgdata[0], packet_buffer.msgdata.size());

  ser_port.async_read_some(
      buf, [this](const ioas_err_t &error, std::size_t bytes) -> void {
        std::vector<uint8_t> msgdata(packet_buffer.msgdata.begin(),
                                     packet_buffer.msgdata.begin() + bytes);
        recv_bottom(std::move(msgdata), error);
      });
}

void EdgeSerial::recv_bottom(std::vector<uint8_t> msgdata,
                             const ioas_err_t &error) {
  if (!error) {
    native_addr_t naddr;
    naddr.typ = FLOWADDR_UNICAST;
    if (recv_cb) recv_cb(edge_id, msgdata, naddr);
  }

  recv_top();
}

}  // namespace fflow
