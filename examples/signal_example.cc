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
  std::promise<bool> a;

  std::cout << std::setprecision(14) << "before timer: " << fflow::now()
            << std::endl;

  AsyncERQPtr tmpptr;

  fflow::add_sighandler<void>(
      ([&](void) -> void { std::cout << "SIGHUP" << std::endl; }), SIGHUP,
      tmpptr);
  fflow::add_sighandler<void>(
      ([&](void) -> void { std::cout << "SIGTERM" << std::endl; }), SIGTERM,
      tmpptr);
  fflow::add_sighandler<void>(
      ([&](void) -> void { std::cout << "SIGINT" << std::endl; }), SIGINT,
      tmpptr);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  //  ev::sig::feed_event(SIGEV_SIGNAL);
  kill(getpid(), SIGHUP);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  kill(getpid(), SIGTERM);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  kill(getpid(), SIGINT);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return 0;
}