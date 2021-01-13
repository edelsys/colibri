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

#ifndef ASYNCCONTROLLER_H
#define ASYNCCONTROLLER_H

#include <ev++.h>

#include <atomic>
#include <condition_variable>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

#include "muqueue.h"

namespace fflow {

typedef ev_tstamp stamp_t;
typedef int iofd_t;

class AsyncController;

class AsyncERQ {
  friend class AsyncController;

 public:
  AsyncERQ() = default;
  virtual ~AsyncERQ() {}

  virtual bool scheduleIn(AsyncController *) = 0;
  virtual bool removeFrom(AsyncController *) = 0;
};

typedef std::shared_ptr<AsyncERQ> AsyncERQPtr;

/**
 * @brief The AsyncController class conceptually
 * close to interrupt controller
 */
class AsyncController : public ev::dynamic_loop {
  std::atomic<bool> runloop;

  std::list<AsyncERQPtr> candidates_for_removal;

  std::list<AsyncERQPtr> candidates_for_addition;

  std::unordered_set<AsyncERQPtr> candidates_in_use;

  std::mutex accesslock;
  std::condition_variable cnotify;  ///< condition variable

  // noncopyable
  AsyncController(const AsyncController &) = delete;
  AsyncController &operator=(const AsyncController &) = delete;

 public:
  AsyncController();
  virtual ~AsyncController();

  bool addERQHandler(AsyncERQPtr);

  bool removeERQHandler(AsyncERQPtr);

  bool enableERQ();

  bool disableERQ();

  bool suspendERQ();

  bool wakeupERQ();

  void worker();  // main worker task

#ifdef BREAK_EV_LOOP_EXPERIMENTAL
  // alternative interface to worker() with possibility to break
  // event loop which is incapsulated by controller
  void start() {
    if (nullptr == worker_)
      worker_ =
          std::make_shared<std::thread>(&AsyncController::worker_func, this);
  }

  void stop() {
    if (worker_) {
      work_stopper_.send();
      worker_->join();
    }
  }

  std::shared_ptr<std::thread> getWorker() const { return worker_; }
#endif

  static AsyncController &getDrainController() {
    static AsyncController drain;
    return drain;
  }

  struct ev_loop *getNativeHandler() {
    return raw_loop;
  }

 private:
  ev::idle idle_w;       // used to prevent busyloop
  ev::check check_w;     // used to prevent busyloop
  ev::prepare before_w;  // loop entrance

#ifdef BREAK_EV_LOOP_EXPERIMENTAL
  std::shared_ptr<std::thread> worker_ = nullptr;
  ev::async work_stopper_;

  void worker_func() {
    work_stopper_.set(getNativeHandler());
    work_stopper_.set<AsyncController, &AsyncController::on_stop>(this);
    work_stopper_.start();
    worker();  // launch event loop
  }

  void on_stop() {
    work_stopper_.stop();
    break_loop(ev::ALL);
  }
#endif

  void before(ev::prepare &, int);

  void checkcontroller(ev::check &, int);

  template <typename T>
  void noop(T &, int);
};

typedef std::shared_ptr<AsyncController> AsyncControllerPtr;

}  // namespace fflow

#endif  // ASYNCCONTROLLER_H
