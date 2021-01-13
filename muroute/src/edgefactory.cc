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

#include <utility>

#include <glog/logging.h>

#include "muflow/muflow.h"
#include "muroute/funcan.h"
#include "muroute/funserial.h"
#include "muroute/funtcpclient.h"
#include "muroute/funtcpserver.h"
#include "muroute/funudp.h"

namespace fflow {
/*static*/ std::shared_ptr<fflow::AbstractEdgeInterface>
createEdgeFunctionByName(const std::string &name) {
  std::shared_ptr<fflow::AbstractEdgeInterface> res;

  LOG(WARNING) << "Creating edge function " << name;

  //!!! USE REGISTRATION TABLE FOR FUTURE (NOT 'if' STATEMENT)
  if (name == "EdgeUdp") {
    return std::move(std::make_shared<EdgeUdp>());
  } else if (name == "EdgeTcpClient") {
    return std::move(std::make_shared<EdgeTcpClient>());
  } else if (name == "EdgeTcpServer") {
    return std::move(std::make_shared<EdgeTcpServer>());
  } else if (name == "EdgeSerial") {
    return std::move(std::make_shared<EdgeSerial>());
  } else if (name == "EdgeCan") {
    //    return std::make_shared<EdgeCan>();
  }
  return res;
}

}  // namespace fflow
