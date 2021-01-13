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

#ifndef ERQPERIODIC_H
#define ERQPERIODIC_H

#include <functional>
#include <future>
#include <iostream>
#include <memory>

#include "asyncsubsys.h"
#include "scheduler.h"

namespace fflow {

template <typename Tret>
class PeriodicERQ : public AsyncERQ, public ev::periodic {
  std::function<Tret()> func;  // user defined callback

  double shift;
  double period;

  // periodic event callback
  void cb(ev::periodic &, int);

 public:
  PeriodicERQ(const std::function<Tret()> _f, double timeshift, double _period);
  virtual ~PeriodicERQ();
  virtual bool scheduleIn(AsyncController *lp);
  virtual bool removeFrom(AsyncController *);
};

template <typename T>
PeriodicERQ<T>::PeriodicERQ(const std::function<T()> f, double timeshift,
                            double _period)
    : func(f), shift(timeshift), period(_period) {
  // set up callback
  set<PeriodicERQ<T>, &PeriodicERQ<T>::cb>(this);
  set(shift, period);
}

template <typename T>
/*virtual*/ PeriodicERQ<T>::~PeriodicERQ() {}

template <typename T>
/*virtual*/ bool PeriodicERQ<T>::scheduleIn(AsyncController *lp) {
  set(lp->getNativeHandler());  // set event loop to run in
  start();
  return true;
}

template <typename T>
/*virtual*/ bool PeriodicERQ<T>::removeFrom(AsyncController * /* lp */) {
  stop();  // stop this watcher
  set(AsyncController::getDrainController()
          .getNativeHandler());  // move to trash
  return true;
}

template <typename T>
void PeriodicERQ<T>::cb(periodic & /* w */, int /* revents */) {
  post_function<T>(func);  // post detached function
  again();                 // reschedule
}

template <typename Tret>
std::shared_future<Tret> add_periodic(const std::function<Tret()> f,
                                      stamp_t off, stamp_t period) {
  AsyncERQPtr bf = std::static_pointer_cast<AsyncERQ>(
      std::make_shared<PeriodicERQ<Tret>>(f, off, period));

  AsyncSubsystem::instance().registerERQ((bf));

  return std::shared_future<Tret>();
}

template <typename Tret>
std::shared_future<Tret> add_periodic_handled(const std::function<Tret()> f,
                                              stamp_t off, stamp_t period,
                                              AsyncERQPtr &bf_out) {
  AsyncERQPtr bf = std::static_pointer_cast<AsyncERQ>(
      std::make_shared<PeriodicERQ<Tret>>(f, off, period));

  AsyncSubsystem::instance().registerERQ((bf));

  bf_out = bf;

  return std::shared_future<Tret>();
}

}  // namespace fflow

#endif  // ERQPERIODIC_H
