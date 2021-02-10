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

#ifndef ASYNCSUBSYS_H
#define ASYNCSUBSYS_H

#include <map>
#include <set>

#include "asynccontroller.h"

namespace fflow {

struct StatNote {};

class AsyncSubsystem {
  const static uint32_t NUMWORKERS = 6;

  /**
   * @brief The StatTable struct
   *
   * Holds all controllers runtime statistics
   * which may be used for load balancing
   */
  struct StatTable
      : public std::vector<std::pair<AsyncControllerPtr, StatNote> > {};

  StatTable subsystemControlTable;  ///< performance statistics table
  std::uint64_t nextid;             ///<
  std::uint64_t maxtblsiz;          ///<
  std::map<AsyncERQPtr, AsyncControllerPtr>
      ERQtoControllerMap;  ///< this links ERQs with their controller

  std::mutex tblLock;

 public:
  AsyncSubsystem();

  virtual ~AsyncSubsystem();

  bool registerERQ(AsyncERQPtr);

  bool removeERQ(AsyncERQPtr);

  static AsyncSubsystem &instance();

  void disableAllERQ() {
    for (auto &evl : subsystemControlTable) {
      evl.first->disableERQ();
    }
  }

  void enableAllERQ() {
    for (auto &evl : subsystemControlTable) {
      evl.first->enableERQ();
    }
  }

  void suspendAllERQ() {
    for (auto &evl : subsystemControlTable) {
      evl.first->suspendERQ();
    }
  }

 private:
  bool contains(AsyncControllerPtr);

  bool addERQController(AsyncControllerPtr);

  bool removeERQController(AsyncControllerPtr);

  AsyncControllerPtr getERQControllerForERQ(AsyncERQPtr);
};

}  // namespace fflow

#endif  // ASYNCSUBSYS_H
