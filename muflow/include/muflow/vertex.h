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

#ifndef VERTEX_H
#define VERTEX_H

#include <functional>

#include "sparseutils.h"

namespace fflow {

/**
 * @brief The Edge struct
 *
 */
struct Edge {
 public:
  pointprec_t weight;      ///< weight of edge
  _SparseDims projection;  ///< Projection (mapping) definition
  SparseAddress origin;    ///< start point vertex address
  SparseAddress end;       ///< endpoint vertex address
};

typedef std::shared_ptr<Edge> EdgeSPtr;
typedef std::weak_ptr<Edge> EdgeWPtr;

class SparseMemory;
typedef std::shared_ptr<SparseMemory> SparseMemoryPtr;

/**
 * @brief The Vertex struct.
 * It's a evolution model of N-dim state point.
 */
struct Vertex {
  friend class SparseMemory;

 protected:
  SparseMemoryPtr spMem;
  Vertex();

 public:
  typedef std::shared_ptr<Vertex> VertexSPtr;
  typedef std::weak_ptr<Vertex> VertexWPtr;

  uint64_t id_in_mem;
  std::vector<tArg> observ;          ///< input
  tArg state;                        ///< state of unit
  std::vector<SparseAddress> edges;  ///< down flow

  // statistics
  uint64_t touchnumber;  ///< number of times vertex was updated
  time_dbl_t lasttouch;  ///< last time of vertex update
  time_dbl_t period;     ///< average period between updates

  // macro parameters
  pointprec_t T;  ///< temperature
  pointprec_t E;  ///< energy
  std::function<pointprec_t(time_dbl_t, const tArg &)> fwd_model;
  std::function<pointprec_t(const tArg &, tArg &)> back_model;

  virtual ~Vertex() {}

  SparseMemoryPtr getSparseMemPtr() { return spMem; }

  void feedback(const tArg & /*in*/, const tArg & /*_st*/, tArg & /*_state*/) {}

  void feedforward(time_dbl_t ts) { fwd_model(ts, state); }

  pointprec_t backpropagate(time_dbl_t /*ts*/, tArg &_in) {
    back_model(state, _in);
    return 0.0;
  }
};

}  // namespace fflow

#endif  // Vertex_H
