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

#include "muqueue/scheduler.h"

#include <muconfig/muconfig.h>

using namespace fflow;

namespace fflow {

tTime now() {
  using cclock = std::chrono::high_resolution_clock;
  using dsec = std::chrono::duration<double>;
  using tps = std::chrono::time_point<cclock, dsec>;

  tps _now = cclock::now();
  return _now.time_since_epoch().count();
}

}  // namespace fflow

TaskScheduler::TaskScheduler(size_t thnum, size_t queuesize) : id(0) {
  // create wq pool of size 'thnum'
  for (size_t i = 0; i < thnum; i++) {
    WQueue *q = new WQueue();
    q->setsize(queuesize);
    q->running = true;
    q->th = __create_thread(std::bind(&WQueue::worker, q));
    queues.push_back(q);

    if (i == 0) {
      sched_param schprio;
      schprio.sched_priority = -10;
      if (pthread_setschedparam(q->th->native_handle(), SCHED_FIFO, &schprio)) {
      }
    }
  }
}

TaskScheduler::~TaskScheduler() {
  for (auto q : queues) {
    q->running = false;
  }

  // join all
  for (auto q : threadsTable) {
#ifdef BREAK_EV_LOOP_EXPERIMENTAL
    auto acp = controllersTable.find(q);
    if (acp != controllersTable.end()) acp->second->stop();
#endif
    if (q->joinable()) q->join();
  }
}

/*static*/ std::unique_ptr<TaskScheduler> &TaskScheduler::instance() {
  static std::unique_ptr<TaskScheduler> instptr;

  if (nullptr == instptr) {
    int num_workers = TaskScheduler::TASKWOKERS;
    int queue_capacity = TaskScheduler::TASKQUEUESIZE;

    std::unique_ptr<muconfig::MuConfig> cfg =
        muconfig::MuConfig::createConfig();
    if (cfg != nullptr && cfg->isOk()) {
      const std::string s_num_workers = "muqueue.scheduler.num_workers";
      auto o_num_workers = cfg->getObject(s_num_workers, muconfig::TYPE::INT);
      if (o_num_workers) {
        auto num_workersp = o_num_workers->getValueSimple<int>();
        if (num_workersp) num_workers = *num_workersp;
      }

      const std::string s_queue_capacity = "muqueue.scheduler.queue_capacity";
      auto o_queue_capacity =
          cfg->getObject(s_queue_capacity, muconfig::TYPE::INT);
      if (o_queue_capacity) {
        auto queue_capacityp = o_queue_capacity->getValueSimple<int>();
        if (queue_capacityp) queue_capacity = *queue_capacityp;
      }
    }

    instptr = std::make_unique<TaskScheduler>(num_workers, queue_capacity);
  }

  return instptr;
}

WQueue::WQueue() { wqueue.set_capacity(1000); }

/*virtual*/ WQueue::~WQueue() {
  running = false;
  th->join();
}

const boost::circular_buffer<DetachedFunctionBasePtr> &WQueue::getbuf() const {
  return wqueue;
}

void WQueue::worker() {
  running = true;

  for (; running.load();) {
    heartbeat0++;  // update heartbeat

    DetachedFunctionBasePtr bs;
    {
      std::unique_lock<std::mutex> lock(accesslock);
      if (wqueue.empty()) {
        cnotify.wait_for(lock, std::chrono::duration<int, std::milli>(5));
        continue;
      }

      bs = std::move(wqueue.front());
      wqueue.pop_front();  // remove from queue
    }

    bs->eval();  // execute function

#ifndef DETACHED_SHARED_PTR
    delete bs;  // clean
#endif
  }

  running = false;  // end
}

bool WQueue::enqueue(DetachedFunctionBasePtr t) {
  std::lock_guard<std::mutex> lock(accesslock);
  bool ret = wqueue.full();
  wqueue.push_back(t);
  cnotify.notify_one();
  return ret;
}
