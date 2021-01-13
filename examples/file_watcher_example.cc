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

int main(int, char**) {
  std::promise<bool> a;
  std::ofstream o("/tmp/file0");
  AsyncERQPtr tmpptr;

  struct stat __stat;
  stat("/tmp/file0", &__stat);
  printf("old size  %ld\n", static_cast<long>(__stat.st_size));
  printf("old atime %ld\n", (long)__stat.st_atime);
  printf("old mtime %ld\n", (long)__stat.st_mtime);

  fflow::add_filehandler<void>(([&](fflow::fstat_data attr) -> void {
                                 printf("size  %ld\n",
                                        static_cast<long>(attr.st_size));
                                 printf("atime %ld\n", (long)attr.st_atime);
                                 printf("mtime %ld\n", (long)attr.st_mtime);
                                 a.set_value(true);
                               }),
                               "/tmp/file0", 0.2, tmpptr);

  // wait a little bit
  sleep(1);

  // modify
  o << "Hi";
  o.flush();

  auto a1 = a.get_future();
  a1.wait();
}
