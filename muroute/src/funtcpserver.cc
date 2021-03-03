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

#include "muroute/funtcpserver.h"

#include <glog/logging.h>

#include <array>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>

using namespace fflow;
using namespace fflow::proto;

#if BOOST_VERSION >= 107000
#define GET_IO_SERVICE(s) \
  ((boost::asio::io_context &)(s).get_executor().context())
#else
#define GET_IO_SERVICE(s) ((s).get_io_service())
#endif

EdgeTcpServer::EdgeTcpServer() : acceptor_(*this) {
  AbstractEdgeInterface::name = "EdgeTcpServer";
}

EdgeTcpServer::~EdgeTcpServer() {
  stop();
  if (dontuseme.joinable()) dontuseme.join();
}

bool EdgeTcpServer::open(uint32_t _port) { return open("127.0.0.1", _port); }

// iface is an ip address
bool EdgeTcpServer::open(const std::string & /*iface*/, uint32_t _port) {
  bool ret = true;
  try {
    VLOG(1) << "creating  ";
    VLOG(2) << serv_endpoint.address().to_string();
    VLOG(2) << bcast.to_string();
    VLOG(2) << "---\n";

    serv_endpoint = {ioasip::tcp::v4(), static_cast<unsigned short>(_port)};
    acceptor_.open(serv_endpoint.protocol());
    acceptor_.bind(serv_endpoint);
    acceptor_.listen();

    accept_top();

    // start io service
    dontuseme = std::thread([&] { run(); });
    dontuseme.detach();

  } catch (const std::exception &ex) {
    LOG(ERROR) << ex.what();
    ret = false;
  }

  return ret;
}

void EdgeTcpServer::sendtoraw(std::vector<uint8_t> &msg,
                              const native_addr_t &to,
                              const native_addr_t &from,
                              uint32_t /*from_edge_id*/) {
  std::array<ioas::const_buffer, 1> buf;

  buf[0] = ioas::buffer(&msg[0], msg.size());

  boost::asio::ip::tcp::endpoint epfrom(
      boost::asio::ip::address_v4(ntohl(from.addr.in.sin_addr.s_addr)),
      ntohs(from.addr.in.sin_port));

  if (to.typ == FLOWADDR_BROADCAST) {
    for (auto &connstr : endpointMap) {
      if (epfrom != connstr.first)
        connstr.second->socket().async_write_some(
            buf,
            [&](const ioas_err_t & /*error*/,
                std::size_t /*bytes_transferred*/) -> void {});
    }
  } else {
    boost::asio::ip::tcp::endpoint ep(
        boost::asio::ip::address_v4(ntohl(to.addr.in.sin_addr.s_addr)),
        ntohs(to.addr.in.sin_port));

    if (endpointMap.find(ep) != endpointMap.end()) {
      endpointMap[ep]->socket().async_write_some(
          buf,
          [&](const ioas_err_t & /*error*/,
              std::size_t /*bytes_transferred*/) -> void {});
    } else {
    }
  }
}

void EdgeTcpServer::accept_top() {
  ioas_err_t error;

  tcp_connection::pointer new_connection =
      tcp_connection::create(GET_IO_SERVICE(acceptor_));

  acceptor_.async_accept(
      new_connection->socket(),
      [this, new_connection](const ioas_err_t &error) -> void {
        accept_bottom(new_connection, error);
      });
}

/// \todo need length of packet to be passed here
void EdgeTcpServer::accept_bottom(tcp_connection::pointer conn,
                                  const ioas_err_t &error) {
  if (!error) {
    VLOG(1) << "New connection from "
            << conn->socket().remote_endpoint().address().to_string() << ":"
            << conn->socket().remote_endpoint().port();

    // save remote endpoint for future use
    conn->remote = conn->socket().remote_endpoint();

    endpointMap[conn->remote] = conn;
    recv_top(conn);
  } else {
    LOG(ERROR) << __PRETTY_FUNCTION__ << ": " << __LINE__ << "  "
               << error.message();
  }

  accept_top();
}

void EdgeTcpServer::recv_top(tcp_connection::pointer conn) {
  std::array<ioas::mutable_buffer, 1> buf;

  msgdata.resize(TR_MAX_PAYLOAD_LEN);
  buf[0] = ioas::buffer(&msgdata[0], msgdata.size());

  ioas_err_t error;
  conn->socket().async_read_some(
      buf, [this, conn](const ioas_err_t &error, std::size_t bytes) -> void {
        if (error == ioas::error::eof ||
            error == ioas::error::connection_reset) {
          if (error == ioas::error::eof) {
            conn->socket().shutdown(ioasip::tcp::socket::shutdown_both);
          }
          conn->socket().close();
          endpointMap.erase(conn->remote);

        } else {
          std::vector<uint8_t> _msgdata(msgdata.begin(),
                                        msgdata.begin() + bytes);
          recv_bottom(conn, _msgdata, error);
        }
      });
}

void EdgeTcpServer::recv_bottom(tcp_connection::pointer conn,
                                std::vector<uint8_t> msg,
                                const ioas_err_t &error) {
  if (!error) {
    if (recv_cb) {
      auto rep = conn->socket().remote_endpoint();

      struct sockaddr_in sa;

      sa.sin_addr.s_addr = htonl(rep.address().to_v4().to_ulong());
      sa.sin_port = htons(rep.port());

      native_addr_t naddr;
      naddr.typ = FLOWADDR_UNICAST;
      naddr.addr.in = sa;
      naddr.len = sizeof(sa);

      recv_cb(edge_id, msg, naddr);
    }
  }

  recv_top(conn);
}
