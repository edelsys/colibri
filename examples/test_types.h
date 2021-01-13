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

#include <assert.h>
#include <glog/logging.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#define TOSTR(x) #x

// not to mess with alignments
#pragma pack(push, 1)

struct DataType0 {
  uint64_t __timestamp = 0;
  double a = 1111.2222;
  unsigned int b = 3;
  short int c = -4;

  static inline const std::string& name() {
    static const std::string name(TOSTR(DataType0));
    return name;
  }

  static inline const std::string& fields() {
    static const std::string fields("double a;uint32_t b;int16_t c;");
    return fields;
  }

  friend std::ostream& operator<<(std::ostream& o, const DataType0& d) {
    o << "{ a=" << d.a << ", b=" << d.b << ", c=" << d.c << " }";
    return o;
  }

  bool operator==(const DataType0& that) const {
    return (a == that.a && b == that.b && c == that.c);
  }

  bool operator!=(const DataType0& that) const { return !(*this == that); }

  std::vector<uint8_t> serialize() {
    std::vector<uint8_t> serialized(sizeof(DataType0));
    size_t len = 0;
    std::memcpy(&serialized[0], &__timestamp, sizeof(__timestamp));
    len += sizeof(__timestamp);
    std::memcpy(&serialized[len], &a, sizeof(a));
    len += sizeof(a);
    std::memcpy(&serialized[len], &b, sizeof(b));
    len += sizeof(b);
    std::memcpy(&serialized[len], &c, sizeof(c));
    assert(serialized.size() == sizeof(c) + len);
    VLOG(2) << name() << " builtin serialization finished";
    return serialized;
  }
};

struct DataType1 {
  float a[3] = {1.02f, -2.03f, 3.04f};
  int b[4] = {1, -2, 3, -4};

  static inline const std::string& name() {
    static const std::string name_(TOSTR(DataType1));
    return name_;
  }

  static inline const std::string& fields() {
    static const std::string fields_("float[3] a;int32_t[4] b;");
    return fields_;
  }

  friend std::ostream& operator<<(std::ostream& o, const DataType1& d) {
    o << "{ a=[ ";
    for (const auto& v : d.a) o << v << " ";
    o << "], b=[ ";
    for (const auto& v : d.b) o << v << " ";
    o << "] }";
    return o;
  }

  bool operator==(const DataType1& that) const {
    return (std::equal(std::begin(a), std::end(a), std::begin(that.a)) &&
            std::equal(std::begin(b), std::end(b), std::begin(that.b)));
  }

  bool operator!=(const DataType1& that) const { return !(*this == that); }
};

struct DataType2 {
  std::vector<double> a{2222.3333, -4444.5555, 6666.7777};
  std::vector<std::string> b{"abc", "defg", "hijkl", "m"};
  bool c = true;

  static inline const std::string& name() {
    static const std::string name_(TOSTR(DataType2));
    return name_;
  }

  static inline const std::string& fields() {
    static const std::string fields_(
        "double[3] a;char[3] b1;char[4] b2;char[5] b3;char[1] b4;bool c;");
    return fields_;
  }

  friend std::ostream& operator<<(std::ostream& o, const DataType2& d) {
    o << "{ a=[ ";
    for (const auto& v : d.a) o << v << " ";
    o << "], b=[ ";
    for (const auto& v : d.b) o << v << " ";
    o << "], c=" << d.c << " }";
    return o;
  }

  bool operator==(const DataType2& that) const {
    return (std::equal(a.begin(), a.end(), that.a.begin()) &&
            std::equal(b.begin(), b.end(), that.b.begin()) && c == that.c);
  }

  bool operator!=(const DataType2& that) const { return !(*this == that); }
};

struct DataType3 {
  DataType0 a[3];
  DataType1 b;
  DataType2 c;

  static inline const std::string& name() {
    static const std::string name_(TOSTR(DataType3));
    return name_;
  }

  static inline const std::string& fields() {
    static const std::string fields_("DataType0[3] a;DataType1 b;DataType2 c;");
    return fields_;
  }

  friend std::ostream& operator<<(std::ostream& o, const DataType3& d) {
    o << "{ a=[ ";
    for (const auto& v : d.a) o << v << " ";
    o << "], b=" << d.b << ", c=" << d.c << " }";
    return o;
  }

  bool operator==(const DataType3& that) const {
    return (std::equal(std::begin(a), std::end(a), std::begin(that.a)) &&
            b == that.b && c == that.c);
  }

  bool operator!=(const DataType3& that) const { return !(*this == that); }
};

#pragma pack(pop)

using DataType4 = std::array<DataType3, 3>;

static std::ostream& operator<<(std::ostream& o, const DataType4& d) {
  o << "[ ";
  for (const auto& v : d) o << v << " ";
  o << "]";
  return o;
}

static bool operator==(const DataType4& lhs, const DataType4& rhs) {
  return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

static bool operator!=(const DataType4& lhs, const DataType4& rhs) {
  return !(lhs == rhs);
}
