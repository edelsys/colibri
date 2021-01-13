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

#ifndef FUN_TCP_SERVER_H
#define FUN_TCP_SERVER_H

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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

class tcp_connection : public std::enable_shared_from_this<tcp_connection> {
 public:
  typedef std::shared_ptr<tcp_connection> pointer;

  static pointer create(ioas::io_service& io_service) {
    return pointer(new tcp_connection(io_service));
  }

  ioasip::tcp::socket& socket() { return socket_; }
  std::vector<uint8_t> msgdata;
  ioasip::tcp::endpoint remote;

 private:
  tcp_connection(boost::asio::io_service& io_service) : socket_(io_service) {}

  void handle_write(const boost::system::error_code& /*error*/,
                    size_t /*bytes_transferred*/) {}

  ioasip::tcp::socket socket_;
  std::string message_;
};

class EdgeTcpServer : public ioas::io_service, public AbstractEdgeInterface {
  typedef std::map<ioasip::tcp::endpoint, tcp_connection::pointer> mappingDb_t;

  mappingDb_t endpointMap;

  ioasip::tcp::acceptor acceptor_;
  ioasip::tcp::endpoint serv_endpoint;

  ioasip::address bcast;
  ioasip::address mcast;

  std::vector<uint8_t> msgdata;

  /// @todo dontuseme
  std::thread dontuseme;

 public:
  EdgeTcpServer();
  virtual ~EdgeTcpServer();

  bool open(uint32_t);

  // EDGE interface
  virtual bool open(const std::string&, uint32_t);
  virtual void sendtoraw(std::vector<uint8_t>&, const native_addr_t&,
                         const native_addr_t&, uint32_t);

 private:
  ///
  /// \brief recv_top top half of receiver handler
  ///
  void accept_top();

  ///
  /// \brief recv_bottom bottom half of receiver handler
  ///
  void accept_bottom(tcp_connection::pointer, const ioas_err_t&);

  void recv_top(tcp_connection::pointer);

  void recv_bottom(tcp_connection::pointer, std::vector<uint8_t>,
                   const ioas_err_t&);
};

}  // namespace fflow
#endif  // FUN_TCP_SERVER_H
