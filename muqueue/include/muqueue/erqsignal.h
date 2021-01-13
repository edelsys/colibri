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

#ifndef ERQSIGNAL_H
#define ERQSIGNAL_H

#include <functional>
#include <future>

#include "asyncsubsys.h"
#include "scheduler.h"

namespace fflow {

template <typename T>
class SignalERQ : public AsyncERQ, public ev::sig {
  std::function<T()> func;  // callback
  int signo;
  int sigevents;
  void cb(ev::sig &, int);

 public:
  SignalERQ(const std::function<T()> &, int);
  virtual ~SignalERQ();

  virtual bool scheduleIn(AsyncController *);
  virtual bool removeFrom(AsyncController *);
};

template <typename T>
SignalERQ<T>::SignalERQ(const std::function<T()> &_f, int _sig)
    : func(_f), signo(_sig) {
  set<SignalERQ<T>, &SignalERQ<T>::cb>(this);
  set(signo);
}

template <typename T>
/*virtual*/ SignalERQ<T>::~SignalERQ() {}

template <typename T>
/*virtual*/ bool SignalERQ<T>::scheduleIn(AsyncController *lp) {
  set(lp->getNativeHandler());
  start();
  return true;
}

template <typename T>
/*virtual*/ bool SignalERQ<T>::removeFrom(AsyncController *) {
  stop();
  // drain
  set(AsyncController::getDrainController().getNativeHandler());
  return true;
}

template <typename T>
void SignalERQ<T>::cb(sig &, int) {
  post_function<T>(func);
}

template <typename Tret>
std::shared_future<Tret> add_sighandler(const std::function<Tret()> f,
                                        int __sig, AsyncERQPtr &bf_out) {
  AsyncERQPtr bf = std::static_pointer_cast<AsyncERQ>(
      std::make_shared<SignalERQ<Tret>>(f, __sig));

  AsyncSubsystem::instance().registerERQ((bf));

  bf_out = bf;

  return std::shared_future<Tret>();
}

}  // namespace fflow

#endif  // ERQSIGNAL_H
