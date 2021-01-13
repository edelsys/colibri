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

#ifndef DETACHEDFUNCTION_H
#define DETACHEDFUNCTION_H

#include <functional>
#include <memory>
#include <mutex>

#include <boost/pool/object_pool.hpp>
#include <boost/pool/pool.hpp>

#define DETACHED_SHARED_PTR

namespace fflow {

class DetachedFunctionBase {
 public:
  virtual ~DetachedFunctionBase();
  virtual void eval() = 0;
};

#ifdef DETACHED_SHARED_PTR
typedef std::shared_ptr<DetachedFunctionBase> DetachedFunctionBasePtr;
#else
typedef DetachedFunctionBase *DetachedFunctionBasePtr;
#endif

class DetachedFunction : public DetachedFunctionBase {
  std::function<void()> func;

  static std::mutex *accesslock;

  DetachedFunction() = delete;

 public:
  DetachedFunction(const std::function<void()> &f);

  void *operator new(size_t);

  void operator delete(void *o);

  void operator delete(void *o, size_t);

  static boost::object_pool<DetachedFunction> &getpool();

  virtual ~DetachedFunction();

  virtual void eval() { func(); }
};

template <typename Ret, typename... Args>
class DetachedFunction2 : public DetachedFunctionBase {
  std::function<Ret(Args...)> func;
  std::tuple<Args...> _args;
  //  Ret _ret;

  DetachedFunction2() = delete;

 public:
  DetachedFunction2(const std::function<Ret(Args...)> f, Args... args)
      : func(std::move(f)), _args(std::forward<Args>(args)...) {}

  virtual ~DetachedFunction2() {}

  virtual void eval() { func(std::get<Args>(_args)...); }
};

}  // namespace fflow

#endif  // DETACHEDFUNCTION_H
