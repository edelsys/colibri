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

#include <muqueue/asyncsubsys.h>

#include <atomic>
#include <boost/dynamic_bitset.hpp>
#include <boost/test/unit_test.hpp>
#include <iostream>

#include "muflow/functiontable.h"
#include "muflow/sparsememory.h"
#include "muflow/sparseutils.h"
#include "muflow/vertex.h"

BOOST_AUTO_TEST_SUITE(sparsemem)

using namespace fflow;

BOOST_AUTO_TEST_CASE(ConstTest) {
  auto hyperMem = SparseMemory::newSparseMemory();
  tArg c = tArg()(_hK3("Hello"), 88.8)(_hK3("World"), 99.9);
  hyperMem->addObservation(c);
  //  hyperMem->renderToHyperSpace();
}

BOOST_AUTO_TEST_SUITE_END()
