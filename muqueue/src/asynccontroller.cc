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

#include "muqueue/asynccontroller.h"

#include <assert.h>
#include <unistd.h>

#include <iostream>

using namespace fflow;

// #define WITH_EV_NON_BLOCKING
#define WITH_ADD_ERQ_INSIDE_LOOP

AsyncController::AsyncController()
/*: ev::dynamic_loop()*/ {
  candidates_for_addition.clear();
  candidates_for_removal.clear();

  check_w.set<AsyncController, &AsyncController::checkcontroller>(this);
  check_w.set(getNativeHandler());
  check_w.start();

  before_w.set<AsyncController, &AsyncController::before>(this);
  before_w.set(getNativeHandler());
  before_w.start();

  // #ifdef WITH_EV_NON_BLOCKING
  // idle_w.set<AsyncController, &AsyncController::noop>(this);
  // idle_w.set(getNativeHandler());
  // idle_w.start();
  // #endif
}

/*virtual*/ AsyncController::~AsyncController() {
  runloop = false;
  for (const auto &ei __UNUSED__ : candidates_in_use) {
    removeERQHandler(ei);
  }

#ifdef BREAK_EV_LOOP_EXPERIMENTAL
  // stop();
#else
  break_loop(ev::ALL);
#endif
}

bool AsyncController::addERQHandler(AsyncERQPtr bfp) {
#ifdef WITH_ADD_ERQ_INSIDE_LOOP
  std::lock_guard<std::mutex> lock(accesslock);
  candidates_for_addition.push_back(bfp);
  cnotify.notify_one();
#else
  bfp->scheduleIn(this);
#endif
  return false;
}

bool AsyncController::removeERQHandler(AsyncERQPtr bfp) {
#ifdef WITH_ADD_ERQ_INSIDE_LOOP
  std::lock_guard<std::mutex> lock(accesslock);
  candidates_for_removal.push_back(bfp);
  cnotify.notify_one();
#else
  bfp->removeFrom(this);
#endif
  return false;
}

bool AsyncController::enableERQ() {
  runloop = true;
  return true;
}

bool AsyncController::disableERQ() {
  runloop = false;
  for (const auto &ei : candidates_in_use) {
    removeERQHandler(ei);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // we also should join the thread
  break_loop(ev::ALL);

  return true;
}

bool AsyncController::suspendERQ() {
  assert(false);
  return false;
}

bool AsyncController::wakeupERQ() {
  assert(false);
  return false;
}

void AsyncController::worker() {
#ifdef WITH_EV_NON_BLOCKING
  for (;;)
    run(EVRUN_NOWAIT);  // dont block
                        // for (;;) run(EVRUN_ONCE);  // block
#else
  run();  // block
#endif
}

void AsyncController::checkcontroller(ev::check &, int) {}

void AsyncController::before(ev::prepare &, int) {
  std::lock_guard<std::mutex> lock(accesslock);

  // remove
  for (const auto &eout : candidates_for_removal) {
    eout->removeFrom(this);
    candidates_in_use.erase(candidates_in_use.find(eout));
  }

  // add
  for (const auto &ein : candidates_for_addition) {
    ein->scheduleIn(this);
    candidates_in_use.insert(ein);
  }

  // clean tails
  candidates_for_addition.clear();

  //
  candidates_for_removal.clear();

  // TODO: update stat
}

template <typename T>
void AsyncController::noop(T & /* w */, int /* revents */) {
  std::unique_lock<std::mutex> lock(accesslock);

  if (candidates_in_use.empty()) {
    cnotify.wait_for(lock, std::chrono::duration<int, std::milli>(50));
  } else if (ev_pending_count(getNativeHandler())) {
    ev_invoke_pending(getNativeHandler());
  } else {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}
