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

#ifndef ERQTIMER_H
#define ERQTIMER_H

#include <functional>
#include <future>
#include <iostream>
#include <memory>

#include "asyncsubsys.h"
#include "scheduler.h"

namespace fflow {

template <typename Tret>
class TimerERQ : public AsyncERQ, public ev::timer {
  std::function<Tret()> func;  // user defined callback

  double delay;
  double repeat;

  // timer event callback
  void cb(timer &, int);

 public:
  TimerERQ(const std::function<Tret()>, double, double);
  virtual ~TimerERQ();

  virtual bool scheduleIn(AsyncController *);
  virtual bool removeFrom(AsyncController *);
};

template <typename T>
TimerERQ<T>::TimerERQ(const std::function<T()> f, double _delay, double _repeat)
    : func(f), delay(_delay), repeat(_repeat) {
  // set up callback
  set<TimerERQ<T>, &TimerERQ<T>::cb>(this);
  set(delay, repeat);
}

template <typename T>
/*virtual*/ TimerERQ<T>::~TimerERQ() {}

template <typename T>
/*virtual*/ bool TimerERQ<T>::scheduleIn(AsyncController *lp) {
  set(lp->getNativeHandler());  // set event loop to run in
  start();
  return true;
}

template <typename T>
/*virtual*/ bool TimerERQ<T>::removeFrom(AsyncController * /* lp */) {
  stop();  // stop this watcher
  set(AsyncController::getDrainController()
          .getNativeHandler());  // move to trash
  return true;
}

template <typename T>
void TimerERQ<T>::cb(timer & /* w */, int /* revents */) {
  post_function<T>(func);  // post detached function
  again();                 // reschedule
}

template <typename Tret>
std::shared_future<Tret> add_timer(const std::function<Tret()> f, stamp_t delay,
                                   stamp_t repeat) {
  AsyncERQPtr bf = std::static_pointer_cast<AsyncERQ>(
      std::make_shared<TimerERQ<Tret>>(f, delay, repeat));

  AsyncSubsystem::instance().registerERQ((bf));

  return std::shared_future<Tret>();
}

}  // namespace fflow

#endif  // ERQTIMER_H
