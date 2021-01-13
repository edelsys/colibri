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

#ifndef ERQIO_H
#define ERQIO_H

#include <functional>
#include <future>

#include "asyncsubsys.h"
#include "scheduler.h"

namespace fflow {

template <typename T>
class IoERQ : public AsyncERQ, public ev::io {
  std::function<T()> func;  // callback
  iofd_t fd;
  iofd_t events;

  void cb(ev::io &, int);

 public:
  IoERQ(const std::function<T()> &, iofd_t, int);
  virtual ~IoERQ();

  virtual bool scheduleIn(AsyncController *);
  virtual bool removeFrom(AsyncController *);
};

template <typename T>
IoERQ<T>::IoERQ(const std::function<T()> &_f, iofd_t _fd, int events)
    : func(_f), fd(_fd) {
  set<IoERQ<T>, &IoERQ<T>::cb>(this);
  set(fd, events);
}

template <typename T>
/*virtual*/ IoERQ<T>::~IoERQ() {}

template <typename T>
/*virtual*/ bool IoERQ<T>::scheduleIn(AsyncController *lp) {
  set(lp->getNativeHandler());
  start();
  return true;
}

template <typename T>
/*virtual*/ bool IoERQ<T>::removeFrom(AsyncController *) {
  stop();
  // drain
  set(AsyncController::getDrainController().getNativeHandler());
  return true;
}

template <typename T>
void IoERQ<T>::cb(ev::io &, int) {
  post_function<T>(func);
}

template <typename Tret>
std::shared_future<Tret> add_io(const std::function<Tret()> &f, iofd_t _fd,
                                int events) {
  // create new IO event request
  AsyncERQPtr bf = std::static_pointer_cast<AsyncERQ>(
      std::make_shared<IoERQ<Tret>>(f, _fd, events));

  // register event request
  AsyncSubsystem::instance().registerERQ((bf));

  return std::shared_future<Tret>();
}

/**
 * @brief Adds handler for io events on the given descriptor
 *
 * The difference from add_io() call is that it gets back a smart pointer
 * to an object used internally in colibri for better control of its
 * lifetime later.
 *
 * Basically it is used in scenarios where it required to remove handler from
 * colibri pipeline at some point.
 *
 * For example if you want to close io descriptor and remove handler from event
 * loop you can use this way:

    AsyncController *async_controller = nullptr;
    erq_handle->removeFrom(async_controller);

 * where 'erq_handle' is AsyncERQPtr pointer filled in a function
 *
 * @tparam Tret return type
 * @param f is io callback
 * @param _fd is i/o descriptor
 * @param events defines what type of events to handle (derived from libev)
 * @param bf_out smart pointer to AsynERQ object which wraps the handler
 * @return std::shared_future<Tret>
 */
template <typename Tret>
std::shared_future<Tret> add_io_handled(const std::function<Tret()> f,
                                        iofd_t _fd, int events,
                                        AsyncERQPtr &bf_out) {
  AsyncERQPtr bf = std::static_pointer_cast<AsyncERQ>(
      std::make_shared<IoERQ<Tret>>(f, _fd, events));

  AsyncSubsystem::instance().registerERQ((bf));

  bf_out = bf;

  return std::shared_future<Tret>();
}

}  // namespace fflow

#endif  // ERQIO_H
