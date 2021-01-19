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

#ifndef FUNCTIONTABLE_H
#define FUNCTIONTABLE_H

#include <cstdint>
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <unordered_map>

#include "sparseutils.h"

namespace fflow {

class FunctionTLB {
 public:
  virtual ~FunctionTLB();

  typedef std::function<pointprec_t(time_dbl_t, const tArg &)> fNeutrino;

  typedef std::forward_list<fNeutrino> ftable_list_t;

  uint64_t add_argtype_detector(const tArgType &, const fNeutrino &);

  ftable_list_t look_for(const tArgType &);

  static FunctionTLB &getTLB() {
    static FunctionTLB functionTlb;  // function tlb
    return functionTlb;
  }

 private:
  FunctionTLB();
  std::vector<std::pair<tArgType, fNeutrino> > _TLBuffer_;
};

}  // namespace fflow

#endif  // FUNCTIONTABLE_H