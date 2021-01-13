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

#include "muroute/funudp.h"

#include <arpa/inet.h>
#include <errno.h>
#include <glog/logging.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>

using namespace fflow;
using namespace fflow::proto;
using namespace boost::asio;
using namespace boost::asio::ip;

static address getIpV4AddrDefault() {
  io_service asio_service;
  udp::resolver resolver(asio_service);
  udp::resolver::query query(udp::v4(), "www.google.com", "");
  udp::resolver::iterator endpoints = resolver.resolve(query);
  udp::endpoint ep = *endpoints;
  udp::socket socket(asio_service);
  socket.connect(ep);
  address addr = socket.local_endpoint().address();
  socket.close();
  return addr;
}

static std::string getInterfaceNameByIpV4(const address &addr) {
  const std::string addr_name = addr.to_string();
  struct ifaddrs *addrs, *iap;
  std::string iface = "";
  bool found = false;
  char buf[32];

  getifaddrs(&addrs);
  for (iap = addrs; iap != NULL; iap = iap->ifa_next) {
    if (iap->ifa_addr && (iap->ifa_flags & IFF_UP) &&
        iap->ifa_addr->sa_family == AF_INET) {
      struct sockaddr_in *sa =
          reinterpret_cast<struct sockaddr_in *>(iap->ifa_addr);
      inet_ntop(iap->ifa_addr->sa_family, static_cast<void *>(&(sa->sin_addr)),
                buf, sizeof(buf));
      if (!addr_name.compare(buf)) {
        found = true;
        break;
      }
    }
  }

  if (found) iface = iap->ifa_name;
  freeifaddrs(addrs);
  return iface;
}

EdgeUdp::routeDb_t EdgeUdp::getEndpoints() const {
  std::lock_guard<std::mutex> lock(endpointslock);
  routeDb_t __endpoints = endpoints;
  return __endpoints;
}

EdgeUdp::EdgeUdp() : asioSocket(*this) {
  AbstractEdgeInterface::name = "EdgeUdp";
}

EdgeUdp::~EdgeUdp() {
  stop();
  if (dontuseme.joinable()) dontuseme.join();
}

bool EdgeUdp::open(uint32_t _port) {
  const std::string iface = getInterfaceNameByIpV4(getIpV4AddrDefault());
  return iface != "" ? open(iface, _port) : false;
}

bool EdgeUdp::open(const std::string &iface, uint32_t _port) {
  bool ret = true;

  try {
    asioSocket.open(ioasip::udp::v4());
    asioSocket.set_option(ioasip::udp::socket::broadcast(true));

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, iface.c_str(), iface.size());

    struct sockaddr_in *sa = NULL;

    // setup local
    if (ioctl(asioSocket.native_handle(), SIOCGIFADDR, &ifr) == -1) {
      throw std::system_error(errno, std::system_category());
    }

    sa = reinterpret_cast<struct sockaddr_in *>(&ifr.ifr_addr);
    selfEndpoint.address(ioasip::address_v4(ntohl(sa->sin_addr.s_addr)));

    if (ioctl(asioSocket.native_handle(), SIOCGIFBRDADDR, &ifr) == -1) {
      throw std::system_error(errno, std::system_category());
    }

    sa = reinterpret_cast<struct sockaddr_in *>(&ifr.ifr_broadaddr);
    bcast = ioasip::address_v4(ntohl(sa->sin_addr.s_addr));

    VLOG(2) << selfEndpoint.address().to_string();
    VLOG(2) << bcast.to_string();

    uint32_t port = _port;
    ioas_err_t error;

    // try to bind any port starting from '_port'
    do {
      asioSocket.bind(ioasip::udp::endpoint(ioasip::udp::v4(), port++), error);
    } while (error);

    selfEndpoint.port(--port);
    //        ports_in_use.insert(_port);
    ports_in_use.insert(selfEndpoint.port());

    recv_top();

    // start io service
    dontuseme = std::thread([&] { run(); });
    dontuseme.detach();

  } catch (const std::exception &ex) {
    LOG(ERROR) << ex.what();
    ret = false;
  }

  return ret;
}

void EdgeUdp::sendtoraw(std::vector<uint8_t> &msg, const native_addr_t &to,
                        const native_addr_t &from, uint32_t /*from_edge_id*/) {
  std::array<ioas::const_buffer, 1> buf;
  buf[0] = ioas::buffer(&msg[0], msg.size());

  ioasip::udp::endpoint epfrom(
      boost::asio::ip::address_v4(ntohl(from.addr.in.sin_addr.s_addr)),
      ntohs(from.addr.in.sin_port));

  const auto &addr = to;
  {
    if (addr.typ == FLOWADDR_BROADCAST) {
#ifdef ENABLE_TRUE_BROADCAST
      for (const auto &port : ports_in_use) {
        asioSocket.async_send_to(buf, ioasip::udp::endpoint(bcast, port),
                                 [&](const ioas_err_t &error,
                                     std::size_t bytes_transferred) -> void {});
      }
#else
      routeDb_t enpts = getEndpoints();
      for (const auto &ep : enpts) {
        if (epfrom != ep)
          asioSocket.async_send_to(
              buf, ep,
              [&](const ioas_err_t & /*error*/,
                  std::size_t /*bytes_transferred*/) -> void {});
      }
#endif
    } else {
      const struct sockaddr_in &sa = addr.addr.in;
      ioasip::udp::endpoint dest(ioasip::address_v4(ntohl(sa.sin_addr.s_addr)),
                                 ntohs(sa.sin_port));

      asioSocket.async_send_to(buf, dest, [](ioas_err_t, std::size_t) {});
    }
  }
}

void EdgeUdp::notifyrouteinfo() {
#ifdef ENABLE_UDP_NOTIFY_ROUTE
  std::shared_ptr<__muflow_packet__> msg =
      std::make_shared<__muflow_packet__>(FLOWPROTO_ID_ROUTEINFO);

  msg->alloc(sizeof(uint32_t) * ports_in_use.size());

  int i = 0;
  for (const auto port : ports_in_use) {
    *(uint32_t *)&msg->payload[i * sizeof(uint32_t)] = htonl(port);
  }

  // prepare buffers
  std::array<ioas::const_buffer, 1> buf;
  buf[0] = ioas::buffer(&msg->msgdata[0], msg->msgdata.size());

  // send ports announcements to ports
  for (const auto &port : ports_in_use) {
    asioSocket.send_to(buf, ioasip::udp::endpoint(bcast, port));
  }
#endif
}

void EdgeUdp::recv_top() {
  std::array<ioas::mutable_buffer, 1> buf;

  packet_buffer.alloc(TR_MAX_PAYLOAD_LEN);

  buf[0] =
      ioas::buffer(&packet_buffer.msgdata[0], packet_buffer.msgdata.size());

  remoteEndpoint = ioasip::udp::endpoint(ioasip::udp::v4(), 0);

  asioSocket.async_receive_from(
      buf, remoteEndpoint,
      [this](const ioas_err_t &error, std::size_t bytes) -> void {
        std::vector<uint8_t> msgdata(packet_buffer.msgdata.begin(),
                                     packet_buffer.msgdata.begin() + bytes);
        recv_bottom(std::move(msgdata), error);
      });
}

/// \todo need length of packet to be passed here
void EdgeUdp::recv_bottom(std::vector<uint8_t> msgdata,
                          const ioas_err_t &error) {
  if (!error) {
    if (remoteEndpoint != selfEndpoint) {
      struct sockaddr_in sa;
      sa.sin_addr.s_addr = htonl(remoteEndpoint.address().to_v4().to_ulong());
      sa.sin_port = htons(remoteEndpoint.port());

      VLOG(4) << "RECV FROM " << remoteEndpoint;

      native_addr_t naddr;
      naddr.typ = FLOWADDR_UNICAST;
      naddr.addr.in = sa;
      naddr.len = sizeof(sa);

      if (recv_cb) {
        int ret = recv_cb(edge_id, msgdata, naddr);
        // update ports in use (for routing purposes)
        if (ret == 0) {  // ok
          ports_in_use.insert(remoteEndpoint.port());
          {
            std::lock_guard<std::mutex> lock(endpointslock);
            auto result = endpoints.insert(remoteEndpoint);
            VLOG_IF(4, !result.second)
                << "Endpoint already exists: " << remoteEndpoint;
          }
        }
      }
    }
  }

  recv_top();
}
