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

#include "muflow/sparsememory.h"

#include <muqueue/muqueue.h>
#include <muqueue/scheduler.h>

#include <iostream>

#include "muflow/muflow.h"

using namespace fflow;

SparseMemory::SparseMemory()
    : edgeid_in_use(0), vertid_in_use(0), DETECTOR_THRESHOLD(0.8) {}

void SparseMemory::addObservation(const tArg & /*vec*/) {
  // TODO:
}

/*static*/ SparseMemoryPtr SparseMemory::newSparseMemory() {
  auto spmem = std::shared_ptr<SparseMemory>(std::move(new SparseMemory()));
  spmem->myself = spmem;  // introduce self to self
  return spmem;
}

// Vertex factory
Vertex::VertexSPtr SparseMemory::newVertex() {
  Vertex::VertexSPtr vsptr = std::shared_ptr<Vertex>(new Vertex());
  vsptr->spMem = myself;
  vsptr->id_in_mem = addVertex(vsptr);
  return vsptr;
}

//
Vertex::VertexSPtr SparseMemory::newVertex(
    std::function<pointprec_t(time_dbl_t, const tArg &)> fw_model,
    std::function<pointprec_t(const tArg &, tArg &)> bck_model) {
  Vertex::VertexSPtr vsptr = std::shared_ptr<Vertex>(new Vertex());

  vsptr->spMem = myself;
  vsptr->fwd_model = fw_model;
  vsptr->back_model = bck_model;
  vsptr->id_in_mem = addVertex(vsptr);

  return vsptr;
}

std::forward_list<Vertex::VertexWPtr> SparseMemory::selectAt(
    const tArg &, const tArgType &range) {
  std::lock_guard<std::mutex> lock(memlock);

  // construct KDtree from the memory
  __UNUSED__ int dim = range.cov._dimensions.size();

  // project all vectors to subspace of type tArgType
  for (__UNUSED__ auto &pt : this->root_cells) {
    // TODO:
  }

  return std::forward_list<Vertex::VertexWPtr>();
}

std::forward_list<tArg> SparseMemory::renderToHyperSpace(
    const std::forward_list<Vertex::VertexWPtr> &, const _sparsedims &,
    uint64_t) {
  std::lock_guard<std::mutex> lock(memlock);

  return std::forward_list<tArg>();
}

const std::unordered_map<uint32_t, Vertex::VertexSPtr>
    &SparseMemory::getVertices() const {
  return vertices;
}

const std::unordered_map<uint32_t, EdgeSPtr> &SparseMemory::getEdges() const {
  return edges;
}

Vertex::VertexWPtr SparseMemory::lookup_vertex(const uint32_t fuid) {
  std::lock_guard<std::mutex> lock(memlock);
  Vertex::VertexWPtr ret;

  std::unordered_map<uint32_t, Vertex::VertexSPtr>::const_iterator fit =
      vertices.find(fuid);

  if (fit != vertices.end()) {
    ret = fit->second;
  }

  return ret;
}

// vertex  operations
uint64_t SparseMemory::feedForward(fflow::time_dbl_t ts) {
  const auto &hedges = getEdges();
  for (const auto &vertex : getVertices()) {
    Vertex::VertexSPtr vrtx = vertex.second;

    // update state
    vrtx->lasttouch = ts;
    vrtx->touchnumber++;
    vrtx->feedforward(ts);

    for (const auto &edgeaddr : vrtx->edges) {
      for (const auto &fuid : edgeaddr.fuid_set) {
        const auto &edgeit = hedges.find(fuid);
        if (edgeit != hedges.end()) {
          //          edgeit
        }
      }
    }
  }
  return 0;
}

uint64_t SparseMemory::innovateVertex(Vertex::VertexWPtr src, const tArg &mcast,
                                      const tArg &args) {
  fflow::time_dbl_t ts = fflow::now();
  std::shared_ptr<tArg> _args = std::make_shared<tArg>(args);
  std::shared_ptr<tArgType> _argtype = std::make_shared<tArgType>(mcast);
  Vertex::VertexSPtr _src = src.lock();
  fflow::pointprec_t thresh = DETECTOR_THRESHOLD;

  // multicast local
  post_function<void>([ts, thresh, this, _src, _argtype, _args](void) -> void {
    if (!_src->edges.size()) {
      FunctionTLB::ftable_list_t all =
          FunctionTLB::getTLB().look_for(*_argtype.get());

      for (const auto &f : all) {
        //        pointprec_t weight = f(ts, *_args);
        //        if (weight >= thresh) {
        Vertex::VertexSPtr vsptr =
            newVertex(f, [](const tArg &, tArg &) -> pointprec_t {
              std::cerr << "back_model\n";
              return 0.0;
            });

        _src->edges.push_back(SparseAddress(0, 0, vsptr->id_in_mem));
        //      }
      }
    }

    // run on all edges
    for (auto &edge : _src->edges) {
      // if (edge.instance_id == 0) { // local sparse memory
      for (const auto &fuid : edge.fuid_set) {
        auto vrtx = lookup_vertex(fuid).lock();
        if (vrtx) vrtx->feedforward(ts);
        // vertices[fuid]->backpropagate(ts, _args);
      }
      // }
    }
  });

  // multicast remotely
  post_function<void>([ts, _src, _argtype, _args](void) -> void {

  });

  return 0;
}
