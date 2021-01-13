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

#include "muflow/functiontable.h"

#include <iostream>
#include <map>

#include <muqueue/muqueue.h>
#include <muqueue/scheduler.h>

#include "muflow/muflow.h"

using namespace fflow;

// ----------- EXPERIMENTAL ---
FunctionTLB::FunctionTLB() {}

FunctionTLB::~FunctionTLB() {}

uint64_t FunctionTLB::add_argtype_detector(const tArgType &atype,
                                           const fNeutrino &f2) {
  _TLBuffer_.push_back(std::pair<tArgType, fNeutrino>(atype, f2));
  VLOG(1) << " FunctionTLB::add_argtype_detector() " << _TLBuffer_.size()
          << std::endl;
  return 0;
}

FunctionTLB::ftable_list_t FunctionTLB::look_for(const tArgType &atype) {
  ftable_list_t all;

  VLOG(1) << "FunctionTLB::ftable_list_t FunctionTLB::look_for() "
          << atype.vec.size() << " " << _TLBuffer_.size() << std::endl;

  for (const auto &pr : _TLBuffer_) {
    if (pr.first == atype) {
      all.push_front(pr.second);
    }
  }
  return all;
}
