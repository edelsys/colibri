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

#ifndef FUNCAN_H
#define FUNCAN_H

#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_set>

// asio switch
#include <boost/asio.hpp>
namespace ioas = boost::asio;
namespace ioasip = boost::asio::ip;
typedef boost::system::error_code ioas_err_t;

#include "proto.h"

using namespace boost;

namespace fflow {

/**
 * @brief The FunCan class
 */
class EdgeCan : public ioas::io_service, public AbstractEdgeInterface {
 public:
  EdgeCan();
  virtual ~EdgeCan();

  /**
   * @brief open
   * @param iface
   * @return
   */
  bool open(const std::string &) {
    throw std::logic_error(std::string("Not Implememented ") +
                           __PRETTY_FUNCTION__);
    return false;
  }

  // EDGE interface
  virtual bool open(const std::string &, uint32_t) {
    throw std::logic_error(std::string("Not Implememented ") +
                           __PRETTY_FUNCTION__);

    return false;
  }

  virtual void sendto(std::vector<uint8_t> &, const native_addr_t &) {
    throw std::logic_error(std::string("Not Implememented ") +
                           __PRETTY_FUNCTION__);
  }

  virtual void sendtoraw(std::vector<uint8_t> &, const native_addr_t &) {
    throw std::logic_error(std::string("Not Implememented ") +
                           __PRETTY_FUNCTION__);
  }

 private:
  void recv_top();
  void recv_bottom(std::vector<uint8_t> msgdata, const ioas_err_t &error);

  std::mutex canlock;
  ioasip::udp::socket cansocket;
  fflow::proto::__muflow_packet__ databuf;

  /// @todo dontuseme
  std::thread dontuseme;
};

}  // namespace fflow

#endif  // FUNCAN_H
