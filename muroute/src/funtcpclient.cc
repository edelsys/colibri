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

#include "muroute/funtcpclient.h"

#include <glog/logging.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>

#include "muqueue/erqperiodic.h"
#include "muroute/muroute.h"

using namespace fflow;
using namespace fflow::proto;
using namespace boost::asio;
using namespace boost::asio::ip;

EdgeTcpClient::EdgeTcpClient() : tcpSocket(*this) {
  AbstractEdgeInterface::name = "EdgeTcpClient";
}

EdgeTcpClient::~EdgeTcpClient() {
  stop();
  if (dontuseme.joinable()) dontuseme.join();
}

bool EdgeTcpClient::open(uint32_t _port) { return open("127.0.0.1", _port); }

// iface is an ip address
bool EdgeTcpClient::open(const std::string &iface, uint32_t _port) {
  bool ret = true;

  try {
    remoteEndpoint = boost::asio::ip::tcp::endpoint(
        boost::asio::ip::address::from_string(iface), _port);

    VLOG(1) << " connecting to " << remoteEndpoint.address().to_string();
    VLOG(2) << remoteEndpoint.address().to_string() << ":" << _port;
    VLOG(2) << selfEndpoint.address().to_string();
    VLOG(2) << bcast.to_string();
    VLOG(2) << "---\n";

    tcpSocket.connect(remoteEndpoint);

    recv_top();

    // announce route info periodically
    //    add_periodic<void>(([&](void) -> void { notifyrouteinfo(); }),
    //    .27, 1.0);

    // start io service
    dontuseme = std::thread([&] { run(); });
    dontuseme.detach();

  } catch (const std::exception &ex) {
    LOG(ERROR) << ex.what();
    ret = false;
  }

  return ret;
}

void EdgeTcpClient::sendtoraw(std::vector<uint8_t> &msg,
                              const native_addr_t & /*to*/,
                              const native_addr_t & /*from*/,
                              uint32_t from_edge_id) {
  std::array<ioas::const_buffer, 1> buf;

  if (from_edge_id == edge_id) {
    VLOG(6) << "Message Loop DETECTED";
    return;
  }

  buf[0] = ioas::buffer(&msg[0], msg.size());

  tcpSocket.async_write_some(buf,
                             [&](const ioas_err_t &, std::size_t) -> void {});
}

void EdgeTcpClient::notifyrouteinfo() {
  std::shared_ptr<__muflow_packet__> msg =
      std::make_shared<__muflow_packet__>(FLOWPROTO_ID_ROUTEINFO);

  msg->alloc(sizeof(uint32_t) * ports_in_use.size());

  int i = 0;
  for (const auto port : ports_in_use) {
    *reinterpret_cast<uint32_t *>(&msg->payload[i * sizeof(uint32_t)]) =
        htonl(port);
  }

  // prepare buffers
  std::array<ioas::const_buffer, 1> buf;
  buf[0] = ioas::buffer(&msg->msgdata[0], msg->msgdata.size());

  // send ports announcements to ports
  for (__UNUSED__ const auto &port : ports_in_use) {
    tcpSocket.async_write_some(
        buf, [&](const ioas_err_t &error, std::size_t) -> void {
          if (error) {
            LOG(WARNING) << error.message() << std::endl;
          }
        });
  }
}

// FIMXE: bottleneck
void EdgeTcpClient::recv_top() {
  std::array<ioas::mutable_buffer, 1> buf;

  msgdata.resize(TR_MAX_PAYLOAD_LEN);

  buf[0] = ioas::buffer(&msgdata[0], msgdata.size());

  ioas_err_t error;
  tcpSocket.async_read_some(
      buf, [this](const ioas_err_t &error, std::size_t bytes) -> void {
        std::vector<uint8_t> _msgdata(msgdata.begin(), msgdata.begin() + bytes);

        recv_bottom(_msgdata, error);
      });
}

/// \todo need length of packet to be passed here
void EdgeTcpClient::recv_bottom(std::vector<uint8_t> msgdata,
                                const ioas_err_t &error) {
  if (!error) {
    if (remoteEndpoint != selfEndpoint) {
      struct sockaddr_in sa;
      sa.sin_addr.s_addr = htonl(remoteEndpoint.address().to_v4().to_ulong());
      sa.sin_port = htons(remoteEndpoint.port());

      native_addr_t naddr;
      naddr.typ = FLOWADDR_UNICAST;
      naddr.addr.in = sa;
      naddr.len = sizeof(sa);

      if (recv_cb) recv_cb(edge_id, msgdata, naddr);
    }
  }

  recv_top();
}
