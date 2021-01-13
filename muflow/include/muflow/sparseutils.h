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

#ifndef SPARSE_H
#define SPARSE_H

#include <cmath>
#include <cstdint>
#include <forward_list>
#include <limits>
#include <list>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "hashing.h"

namespace fflow {

using pointprec_t = std::double_t;  // precision type to use
typedef std::uint64_t fflow_id_t;   // Identificator type
typedef std::uint64_t dim_id_t;     // Dimension id type

typedef pointprec_t time_dbl_t;             // time type
typedef std::vector<dim_id_t> _sparsedims;  // finite feature subspace

typedef std::vector<char> voc_uid_t;  // vocabulary unique id
typedef std::shared_ptr<voc_uid_t> p_voc_uid_t;
typedef std::tuple<pointprec_t, p_voc_uid_t> argval;
typedef std::unordered_map<dim_id_t, argval> _sparsevector;

struct _SparseDims : public _sparsedims {};

/**
 * @brief The _SparseVector struct
 */
struct _SparseVector : public _sparsevector {
  _SparseVector projection(const _SparseDims & /*components*/) { return *this; }
  _SparseVector &operator()(dim_id_t r, pointprec_t v0,
                            p_voc_uid_t v1 = std::shared_ptr<voc_uid_t>()) {
    insert(std::pair<dim_id_t, argval>(r, argval(v0, v1)));
    return *this;
  }
};

/**
 * @brief The _SparseMatrix struct
 * This is a sparse matrix class which holds matrices
 * encoded by dimension IDs instead of dimension number.
 * The dimension Id is used as a key to sparse memory cells
 *
 * It is supposed that SparseMatrix has equal number
 * of rows and cols.
 */
struct _SparseMatrix {
  int rows;
  int cols;

  typedef std::unordered_map<dim_id_t, _SparseVector> sparsedims;
  typedef std::unordered_map<dim_id_t, _SparseVector>::const_iterator sparse_it;

  sparsedims _dimensions;

  _SparseMatrix &operator()(dim_id_t r, dim_id_t c, pointprec_t v) {
    _dimensions[r][c] = argval(v, std::shared_ptr<voc_uid_t>());
    return *this;
  }

  pointprec_t &operator()(dim_id_t r, dim_id_t c) {
    return std::get<0>(_dimensions[r][c]);
  }

  pointprec_t &operator[](const std::pair<dim_id_t, dim_id_t> &p) {
    return std::get<0>(_dimensions[p.first][p.second]);
  }

  pointprec_t FrobeniusNorm(const _SparseMatrix & /*other*/) {
    return std::numeric_limits<pointprec_t>::infinity();
  }
};

typedef _SparseVector tArg;

struct _SparseMemoryCell {
  fflow_id_t id;
  tArg vec;           // stochastic pointer vector
  _SparseMatrix cov;  // covariance

  _SparseMemoryCell(const tArg &mx) : vec(mx) {}

  _SparseMemoryCell() {}

  bool operator<(const _SparseMemoryCell &other) const {
    return (vec.size() < other.vec.size());
  }

  bool operator==(const _SparseMemoryCell &other) const {
    return (vec.size() == other.vec.size());
  }
};

typedef _SparseMemoryCell tArgType;

/**
 * @brief The SparseAddress struct
 * Can be multicast (multiple IDs in use)
 */
struct SparseAddress {
  uint32_t group_id;     // id of function flow instance
  uint32_t instance_id;  // id of function flow instance
  uint32_t fuid_set[8];  // up to 8 function ids
  int fuidptr = 0;

  SparseAddress() : group_id(-1), instance_id(-1), fuidptr(0) {}

  SparseAddress(fflow::SparseAddress const &other)
      : group_id(other.group_id), instance_id(other.instance_id), fuidptr(0) {
    std::copy(other.fuid_set, other.fuid_set + 8, fuid_set);
  }

  SparseAddress(uint32_t _muid, uint32_t _guid, uint32_t _fuid)
      : group_id(_muid), instance_id(_guid), fuidptr(0) {
    fuid_set[fuidptr] = _fuid;
  }
  virtual ~SparseAddress() {}
  bool valid(void) const { return (instance_id != uint32_t(-1)); }
};

typedef std::forward_list<SparseAddress> tSparseAddrCollection;

/**
 * @brief getCurvatureFor
 * @param l
 * @param r
 * @return
 */
_SparseMatrix getCurvatureFor(const tArg &, const tArg &);

}  // namespace fflow

#endif  // SPARSE_H
