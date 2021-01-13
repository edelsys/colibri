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

#include "muqueue/detachedfunction.h"

using namespace fflow;

std::mutex *DetachedFunction::accesslock = new std::mutex();

DetachedFunctionBase::~DetachedFunctionBase() {}

DetachedFunction::DetachedFunction(const std::function<void()> &f)
    : func(std::move(f)) {}

DetachedFunction::~DetachedFunction() {}

boost::object_pool<DetachedFunction> &DetachedFunction::getpool() {
  static boost::object_pool<DetachedFunction> *mpool =
      new boost::object_pool<DetachedFunction>(sizeof(DetachedFunction));
  return *mpool;
}

void *DetachedFunction::operator new(size_t /*siz*/) {
  std::lock_guard<std::mutex> lock(*accesslock);
  void *p = getpool().malloc();
  return p;
}

void DetachedFunction::operator delete(void *o) {
  std::lock_guard<std::mutex> lock(*accesslock);
  getpool().free(reinterpret_cast<DetachedFunction *>(o));
}

void DetachedFunction::operator delete(void *o, size_t /*siz*/) {
  std::lock_guard<std::mutex> lock(*accesslock);
  getpool().free(reinterpret_cast<DetachedFunction *>(o));
}
