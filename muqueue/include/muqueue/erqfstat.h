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

#ifndef ERQFSTAT_H
#define ERQFSTAT_H

#include <functional>
#include <future>

#include "asyncsubsys.h"
#include "scheduler.h"

namespace fflow {

typedef ev_statdata fstat_data;

template <typename T>
class FstatERQ : public AsyncERQ, public ev::stat {
  std::function<T(fstat_data)> func;  // callback
  std::string path;
  tTime intrvl;

  void cb(ev::stat &, int);

 public:
  FstatERQ(const std::function<T(fstat_data)> &, std::string &,
           fflow::tTime interval = 0);

  virtual ~FstatERQ();

  virtual bool scheduleIn(AsyncController *);
  virtual bool removeFrom(AsyncController *);
};

template <typename T>
FstatERQ<T>::FstatERQ(const std::function<T(fstat_data)> &_f,
                      std::string &_path, fflow::tTime interval)
    : func(_f), path(_path), intrvl(interval) {
  set<FstatERQ<T>, &FstatERQ<T>::cb>(this);
  set(path.c_str(), intrvl);
}

template <typename T>
/*virtual*/ FstatERQ<T>::~FstatERQ() {}

template <typename T>
/*virtual*/ bool FstatERQ<T>::scheduleIn(AsyncController *lp) {
  set(lp->getNativeHandler());
  start();
  return true;
}

template <typename T>
/*virtual*/ bool FstatERQ<T>::removeFrom(AsyncController *) {
  stop();
  // drain
  set(AsyncController::getDrainController().getNativeHandler());
  return true;
}

template <typename T>
void FstatERQ<T>::cb(ev::stat &, int) {
  fstat_data _attr = this->attr;
  post_function2<T, fstat_data>(func, _attr);
  // func(_attr);
}

template <typename Tret>
std::shared_future<Tret> add_filehandler(
    const std::function<Tret(fstat_data)> f, std::string _path,
    fflow::tTime interval, AsyncERQPtr &bf_out) {
  AsyncERQPtr bf = std::static_pointer_cast<AsyncERQ>(
      std::make_shared<FstatERQ<Tret>>(f, _path, interval));

  AsyncSubsystem::instance().registerERQ((bf));

  bf_out = bf;

  return std::shared_future<Tret>();
}

}  // namespace fflow

#endif  // ERQFSTAT_H
