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

/**
 * @file io_example.cc
 *
 * This file contains example of using i/o wrappers of colibri.
 * As a source of i/o events the socket descriptor is used.
 *
 * Open a socket and pass its descriptor to colibri with 'add_io'
 * method along with callback function which is a lambda-function
 * in this case.
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
#if 1
  static int sock_fd;

  int master_fd;
  int sock_opt = 1;
  int conn_port = 7000;
  struct sockaddr_in addr;
  socklen_t addrlen;

  // ---------------------------------------
  // create and setup a listening tcp socket
  master_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (master_fd == -1) {
    perror("socket");
  }

  // reusable to avoid badly closed ones on the same port
  if (setsockopt(master_fd, SOL_SOCKET, SO_REUSEADDR,
                 static_cast<void *>(&sock_opt), sizeof(sock_opt)) == -1) {
    perror("setsockopt");
  }

  // use any address as its listening
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(conn_port);
  addrlen = sizeof(addr);

  if (::bind(master_fd, reinterpret_cast<struct sockaddr *>(&addr), addrlen) !=
      0) {
    perror("bind");
  }

  if (listen(master_fd, 3) != 0) {
    perror("listen");
  }

  fprintf(stderr, "awaiting a connection on port %d\n", conn_port);
  sock_fd =
      accept(master_fd, reinterpret_cast<struct sockaddr *>(&addr), &addrlen);
  if (sock_fd == -1) {
    perror("accept");
  }
  fputs("in: connection established\n", stderr);

  // start serving on the socket
  AsyncERQPtr erqhdlr;
  add_io_handled<void>(([&](void) -> void {
                         int r, t;
                         char buf[1024];

                         std::cout << "e";

                         for (t = 0;
                              (r = read(sock_fd, buf, sizeof(buf))) > 0;) {
                           t += r;
                           if (write(STDOUT_FILENO, buf, r)) {
                           }  // copy input to stdout
                              //  write back
                           write(sock_fd, buf, r);
                           if (buf[r - 1] == '\n') break;  // one line at a time
                         }
                         fprintf(stderr, "in: count = %d\n", t);
                         if (r == 0) {
                           fputs("in: connection closed\n", stderr);
                           if (erqhdlr) erqhdlr->removeFrom(nullptr);
                         } else if (r < 0) {
                           perror("read");
                         }
                       }),
                       sock_fd, EV_READ, erqhdlr);

#endif  // 0

  pause();

  // std::cout << " type any command 1\n";

  // char c;
  // std::cin >> c;

  // Notify all eventloops to stop firing events
  AsyncSubsystem::instance().disableAllERQ();

  return 0;
}
