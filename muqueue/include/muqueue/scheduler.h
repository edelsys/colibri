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

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <atomic>
#include <boost/circular_buffer.hpp>
#include <functional>
#include <future>
#include <mutex>
#include <set>
#include <vector>

#ifdef BREAK_EV_LOOP_EXPERIMENTAL
#include <unordered_map>

#include "asynccontroller.h"
#endif

#include "detachedfunction.h"

namespace fflow {
struct TaskScheduler;

class WQueue {
  friend TaskScheduler;

  std::atomic<bool> running;              ///< flag indicating is queue running
  std::atomic<std::uint32_t> heartbeat0;  ///<  heartbeat counter

  std::mutex accesslock;            ///< mutex
  std::condition_variable cnotify;  ///< condition variable
  std::shared_ptr<std::thread> th;  ///< worker thread

  boost::circular_buffer<DetachedFunctionBasePtr>
      wqueue;  ///< workqueu itself (curcular buffer)

  // noncopyable
  WQueue(const WQueue &) = delete;
  WQueue &operator=(const WQueue &) = delete;

 protected:
  bool setsize(size_t size) {
    wqueue.set_capacity(size);
    return true;  /// @bug
  }

 public:
  WQueue();
  virtual ~WQueue();

  void enqueue(DetachedFunctionBasePtr);

  void worker();

  size_t getsize() { return wqueue.capacity(); }

  const boost::circular_buffer<DetachedFunctionBasePtr> &getbuf() const;
};

struct TaskScheduler {
  const static uint32_t TASKWOKERS = 4;
  const static uint32_t TASKQUEUESIZE = 1000;

#ifdef BREAK_EV_LOOP_EXPERIMENTAL
  class Hasher {
   public:
    size_t operator()(const std::shared_ptr<std::thread> t1) const {
      return std::hash<int>()(t1->native_handle());
    }
  };

  struct Equal {
    bool operator()(const std::shared_ptr<std::thread> &lhs,
                    const std::shared_ptr<std::thread> &rhs) const {
      return lhs->native_handle() == rhs->native_handle();
    }
  };
#endif

  std::uint64_t id;                                     ///< identifier
  std::mutex accesslock;                                ///< mutex
  std::vector<WQueue *> queues;                         ///< task queues
  std::set<std::shared_ptr<std::thread>> threadsTable;  ///<
#ifdef BREAK_EV_LOOP_EXPERIMENTAL
  std::unordered_map<std::shared_ptr<std::thread>, AsyncControllerPtr, Hasher,
                     Equal>
      controllersTable;
#endif

  TaskScheduler(size_t, size_t queuesize = TASKQUEUESIZE);
  virtual ~TaskScheduler();

  TaskScheduler(const TaskScheduler &) = delete;
  TaskScheduler &operator=(const TaskScheduler &) = delete;

  static std::unique_ptr<TaskScheduler> &instance();

  void enqueue_impl(DetachedFunctionBasePtr f, int prio) {
    // round robin
    size_t qsiz = queues.size();
    assert(qsiz);

    if (prio > 0 || (qsiz == 1))  // dummy QOS
    {
      queues[0]->enqueue(f);  // pass to workqueue
    } else if (qsiz > 1) {
      std::uint64_t next = (id++ % (qsiz - 1)) + 1;
      queues[next]->enqueue(f);  // pass to workqueue
    } else {
      assert(false);
    }
  }

  static void enqueue(DetachedFunctionBasePtr t, int prio = 0) {
    return instance()->enqueue_impl(t, prio);
  }

  static void getInfo() {
    instance()->queues.size();
    for (const auto &queue : instance()->queues) {
      __UNUSED__ auto cbuf = queue->getbuf();
      __UNUSED__ size_t cap = cbuf.capacity();
      __UNUSED__ size_t siz = cbuf.size();
      __UNUSED__ size_t max = cbuf.max_size();
      __UNUSED__ bool full = cbuf.full();
    }
  }

  template <typename F>
  std::shared_ptr<std::thread> __create_thread(F threadfunc) {
    std::lock_guard<std::mutex> g(accesslock);
    auto th = std::make_shared<std::thread>(threadfunc);
    threadsTable.insert(th);

    sched_param schprio;
    schprio.sched_priority = 0;
    if (pthread_setschedparam(th->native_handle(), SCHED_RR, &schprio)) {
    }

    return th;
  }

  template <typename F>
  static std::shared_ptr<std::thread> create_thread(F threadfunc) {
    return instance()->__create_thread(threadfunc);
  }

#ifdef BREAK_EV_LOOP_EXPERIMENTAL
  void __acquire_controller_thread(AsyncControllerPtr acp) {
    std::lock_guard<std::mutex> g(accesslock);
    std::shared_ptr<std::thread> thp = acp->getWorker();
    threadsTable.insert(thp);
    controllersTable[thp] = acp;
  }

  static void acquire_controller_thread(AsyncControllerPtr acp) {
    acp->start();
    instance()->__acquire_controller_thread(acp);
  }
#endif
};

template <typename T>
void post_function(const std::function<T()> &f, int prio = 0) {
#ifdef DETACHED_SHARED_PTR
  DetachedFunctionBasePtr t = std::make_shared<DetachedFunction>(f);
#else
  DetachedFunctionBasePtr t = new DetachedFunction(f);
#endif
  TaskScheduler::enqueue(t, prio);
  return;
}

template <typename Ret, typename... Args>
void post_function2(const std::function<Ret(Args...)> &f, Args... args) {
#ifdef DETACHED_SHARED_PTR
  DetachedFunctionBasePtr t =
      std::make_shared<DetachedFunction2<Ret, Args...>>(f, args...);
#else
  DetachedFunctionBasePtr t = new DetachedFunction2<Ret, Args...>(f, args...);
#endif
  TaskScheduler::enqueue(t);
  return;
}

}  // namespace fflow

#endif  // SCHEDULER_H
