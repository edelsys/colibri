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

#include "muqueue/asyncsubsys.h"
#include "muqueue/scheduler.h"

using namespace fflow;

AsyncSubsystem::AsyncSubsystem() : nextid(0), maxtblsiz(NUMWORKERS) {
  subsystemControlTable.clear();
}

/*virtual*/ AsyncSubsystem::~AsyncSubsystem() {}

bool AsyncSubsystem::registerERQ(AsyncERQPtr bfp) {
  std::lock_guard<std::mutex> lock(tblLock);

  AsyncControllerPtr el = getERQControllerForERQ(bfp);
  el->addERQHandler(bfp);
  // make note
  ERQtoControllerMap[bfp] = el;
  return false;
}

bool AsyncSubsystem::removeERQ(AsyncERQPtr bfp) {
  std::lock_guard<std::mutex> lock(tblLock);
  if (ERQtoControllerMap.find(bfp) != ERQtoControllerMap.end()) {
    return ERQtoControllerMap.at(bfp)->removeERQHandler(bfp);
  }
  return true;
}

/*static*/ AsyncSubsystem &AsyncSubsystem::instance() {
  static AsyncSubsystem inst;
  return inst;
}

bool AsyncSubsystem::contains(AsyncControllerPtr ctlr) {
  bool ret = false;
  for (auto &evl : subsystemControlTable) {
    if (ctlr == evl.first) {
      ret = true;
      break;
    }
  }
  return ret;
}

bool AsyncSubsystem::addERQController(AsyncControllerPtr lop) {
  if (!contains(lop)) {
    subsystemControlTable.push_back(
        std::pair<AsyncControllerPtr, StatNote>(lop, StatNote()));

    lop->enableERQ();

#ifdef BREAK_EV_LOOP_EXPERIMENTAL
    TaskScheduler::acquire_controller_thread(lop);
#else
    // post function to thread pool
    TaskScheduler::create_thread(
        std::bind(&AsyncController::worker, lop.get()));
#endif

    return true;
  }
  return false;
}

bool AsyncSubsystem::removeERQController(AsyncControllerPtr lop) {
  if (contains(lop)) {
    lop->disableERQ();
    for (auto &evl : subsystemControlTable) {
      if (lop == evl.first) {
        //        statTable.erase(evl);
        break;
      }
    }
  }
  return true;
}

// round robin
AsyncControllerPtr AsyncSubsystem::getERQControllerForERQ(AsyncERQPtr /*bfp*/) {
  size_t q = nextid % maxtblsiz;
  if (subsystemControlTable.size() < (q + 1)) {
    addERQController(std::make_shared<AsyncController>());
  }
  nextid++;
  return subsystemControlTable[q].first;
}
