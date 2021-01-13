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

#ifndef FUNTCPCLIENT_H
#define FUNTCPCLIENT_H

#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

// asio switch
#if 1
#include <boost/asio.hpp>
namespace ioas = boost::asio;
namespace ioasip = boost::asio::ip;
typedef boost::system::error_code ioas_err_t;
#else
#include <asio.hpp>
#include <system_error>
namespace ioas = ::asio;
namespace ioasip = ::asio::ip;
typedef std::error_code ioas_err_t;
#endif

#include "proto.h"

namespace fflow {

class EdgeTcpClient : public ioas::io_service, public AbstractEdgeInterface {
  typedef std::set<ioasip::tcp::endpoint> routeDb_t;
  typedef std::unordered_set<uint32_t> portDb_t;

  portDb_t ports_in_use;
  routeDb_t routing_table;

  std::vector<uint8_t> msgdata;

  std::mutex socklock;
  ioasip::tcp::socket tcpSocket;

  ioasip::tcp::endpoint selfEndpoint;
  ioasip::tcp::endpoint remoteEndpoint;

  ioasip::address bcast;
  ioasip::address mcast;

  /// @todo dontuseme
  std::thread dontuseme;

 public:
  EdgeTcpClient();
  virtual ~EdgeTcpClient();

  bool open(uint32_t);

  boost::asio::ip::address getSelfAddress() const {
    return selfEndpoint.address();
  }

  unsigned short getSelfPort() const { return selfEndpoint.port(); }

  boost::asio::ip::address getRemoteAddress() const {
    return selfEndpoint.address();
  }

  unsigned short getRemotePort() const { return selfEndpoint.port(); }

  // EDGE interface
  virtual bool open(const std::string &, uint32_t);
  virtual void sendtoraw(std::vector<uint8_t> &, const native_addr_t &,
                         const native_addr_t &, uint32_t);
  virtual void notifyrouteinfo();

 private:
  ///
  /// \brief recv_top top half of receiver handler
  ///
  void recv_top();

  ///
  /// \brief recv_bottom bottom half of receiver handler
  ///
  void recv_bottom(std::vector<uint8_t>, const ioas_err_t &);
};

}  // namespace fflow

#endif  // FUNTCPCLIENT_H
