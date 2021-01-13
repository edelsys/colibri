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

#include <muqueue/asyncsubsys.h>
#include <muqueue/erqfstat.h>
#include <muqueue/erqio.h>
#include <muqueue/erqperiodic.h>
#include <muqueue/erqsignal.h>
#include <muqueue/erqtimer.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <boost/test/unit_test.hpp>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>

using namespace boost;
using namespace fflow;

int main(int, char **) {
  // define atomic counters
  std::atomic<std::uint64_t> p1(0);
  std::atomic<std::uint64_t> p2(0);
  std::atomic<std::uint64_t> p3(0);

  // start periodic functions
  add_periodic<void>(([&](void) -> void { p1++; }), 0.000001, 0.0001);
  add_periodic<void>(([&](void) -> void { p2++; }), 0.000001, 0.00001);
  add_periodic<void>(([&](void) -> void { p3++; }), 0.000001, 0.000001);

  std::uint64_t _p1 = p1, _p2 = p2, _p3 = p3;

  auto tp1 = fflow::now();

  std::this_thread::sleep_for(std::chrono::seconds(30));

  auto tp2 = fflow::now();

  std::uint64_t __p1 = p1, __p2 = p2, __p3 = p3;

  std::cout << std::setprecision(14) << __p1 << std::endl;
  std::cout << std::setprecision(14) << __p2 << std::endl;
  std::cout << std::setprecision(14) << __p3 << std::endl;

  std::cout << (tp2 - tp1) / (__p1 - _p1) << std::endl;
  std::cout << (tp2 - tp1) / (__p2 - _p2) << std::endl;
  std::cout << (tp2 - tp1) / (__p3 - _p3) << std::endl;

  AsyncSubsystem::instance().disableAllERQ();

  std::cout << " " << TaskScheduler::instance()->queues.size() << std::endl;
  std::cout << " " << TaskScheduler::instance()->threadsTable.size()
            << std::endl;

  return 0;
}
