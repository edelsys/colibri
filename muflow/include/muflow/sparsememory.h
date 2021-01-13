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

#ifndef SPARSEMEMORY_H
#define SPARSEMEMORY_H

#include <forward_list>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include "functiontable.h"
#include "sparseutils.h"
#include "vertex.h"

namespace fflow {

/**
 *@brief The SparseMemory class
 *
 * this holds graph of functions
 */
class SparseMemory {
  std::list<Vertex::VertexSPtr> root_cells;                   ///< roots
  std::unordered_map<uint32_t, Vertex::VertexSPtr> vertices;  ///< all vertices
  std::unordered_map<uint32_t, EdgeSPtr> edges;               ///< all edges
  std::mutex memlock;                                         ///<

  uint64_t edgeid_in_use;
  uint64_t vertid_in_use;

  fflow::pointprec_t DETECTOR_THRESHOLD;  // threshold

  SparseMemory();

  SparseMemoryPtr myself;

 public:
  static SparseMemoryPtr newSparseMemory();

  void clear() {
    edges.clear();
    vertices.clear();
  }

  void addObservation(const tArg &);

  Vertex::VertexSPtr newVertex();

  Vertex::VertexSPtr newVertex(
      std::function<pointprec_t(time_dbl_t, const tArg &)>,
      std::function<pointprec_t(const tArg &, tArg &)>);

  uint64_t addVertex(Vertex::VertexSPtr vptr) {
    std::lock_guard<std::mutex> lock(memlock);
    vertices[++vertid_in_use] = vptr;
    return vertid_in_use;
  }

  bool rmEdge(const SparseAddress &vedg) {
    std::lock_guard<std::mutex> lock(memlock);
    for (const auto fuid : vedg.fuid_set) {
      edges.erase(fuid);
    }
    return true;
  }

  bool rmVertex(const SparseAddress &vptr) {
    std::lock_guard<std::mutex> lock(memlock);
    for (const auto fuid : vptr.fuid_set) {
      const auto &vrtxit = vertices.find(fuid);
      if (vrtxit != vertices.end()) {
        // iterate edges
        for (const auto &edgid : vrtxit->second->edges) {
          rmEdge(edgid);
        }
        vertices.erase(vrtxit);
      }
    }
    return true;
  }

  bool addEdge(EdgeSPtr eptr) {
    std::lock_guard<std::mutex> lock(memlock);
    edges[edgeid_in_use++] = eptr;
    return true;
  }

  Vertex::VertexWPtr lookup_vertex(const uint32_t);

  const std::unordered_map<uint32_t, Vertex::VertexSPtr> &getVertices() const;

  const std::unordered_map<uint32_t, EdgeSPtr> &getEdges() const;

  uint64_t feedForward(fflow::time_dbl_t);

  uint64_t innovateVertex(Vertex::VertexWPtr, const tArg &, const tArg &);

  /**
   * @brief Selects distribution for given vector with given covariance
   * @param at
   * @param range
   * @return list of vertices satisfying given range
   */
  std::forward_list<Vertex::VertexWPtr> selectAt(const tArg &,
                                                 const tArgType &);

  /**
   * @brief render memory subgraph into hyperspace defined with _sparsedims
   * with given density. (Projection)
   *
   * @param subgraph
   * @param hyperspace
   * @param dense
   * @return
   */
  std::forward_list<tArg> renderToHyperSpace(
      const std::forward_list<Vertex::VertexWPtr> &, const _sparsedims &,
      uint64_t);
};

}  // namespace fflow

#endif  // SPARSEMEMORY_H
