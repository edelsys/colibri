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

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE

#include <atomic>
#include <iostream>
#include <string>

#include <boost/test/unit_test.hpp>

#include "muflow/dynamicloader.h"

BOOST_AUTO_TEST_SUITE(plugins)

using namespace fflow;

BOOST_AUTO_TEST_CASE(pluginLoader2) {
  std::string fname = "libtestplugin2.so";
  void *handle = nullptr;

  bool isok = DynamicLoader::loadFunctionFromFile(fname, &handle);

  char c;
  std::cin >> c;

  if (handle) DynamicLoader::releaseFunction(handle);

  BOOST_CHECK_MESSAGE(isok, "Plugin name = " << fname);
}

BOOST_AUTO_TEST_CASE(pluginLoader1) {
  std::string fname = "libtestplugin.so";
  void *handle = nullptr;

  bool isok = DynamicLoader::loadFunctionFromFile(fname, &handle);

  char c;
  std::cin >> c;

  if (handle) DynamicLoader::releaseFunction(handle);

  BOOST_CHECK_MESSAGE(isok, "Plugin name = " << fname);
}

BOOST_AUTO_TEST_SUITE_END()
