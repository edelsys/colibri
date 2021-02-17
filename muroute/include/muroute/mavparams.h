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

#pragma once

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "mavlink2/common/mavlink.h"

namespace fflow {

class MavParams {
  // global for all parameters of all components
  static std::map<std::string, uint8_t> paramIdCompId;
  static std::mutex prot;

 public:
  static const size_t max_param_id_len = 16;
  static const size_t max_param_value_len = 128;

  struct ParamUnion {
    union {
      double param_double;
      float param_float;
      int64_t param_int64;
      uint64_t param_uint64;
      int32_t param_int32;
      uint32_t param_uint32;
      int16_t param_int16;
      uint16_t param_uint16;
      int8_t param_int8;
      uint8_t param_uint8;
      uint8_t bytes[max_param_value_len];
    };
    uint8_t type;

    ParamUnion();
    uint8_t getParameterType() const { return type; }
    size_t getParamSize() const;
    std::string toReadable() const;
    bool decodeParameterValue(mavlink_param_value_t &) const;
    bool setParameterValue(float, uint8_t);
    bool setParameterValueExt(const char *v, uint8_t t) {
      (void)v;
      (void)t;
      // TODO:
      return false;
    }
  };

 public:
  MavParams() = default;
  virtual ~MavParams() = default;

 public:
  const std::map<std::string, std::string> &getParameterList() const {
    return paramValue_;
  }

  std::string getParameterId(int);
  int getParameterIdx(const std::string &);
  int getParameterType(const std::string &);
  std::string getParameterValue(const std::string &);
  int getParameterCount() const { return paramCntr_; }

  bool updateParameterValue(const std::string &, const ParamUnion &);

  bool setParameterValue(const std::string &, const std::string &, int);
  bool setParameterValue(const std::string &, const std::string &, int,
                         uint8_t);
  bool setParameterValue(const std::string &, double);
  bool setParameterValue(const std::string &, double, uint8_t);
  bool setParameterValue(const std::string &, float);
  bool setParameterValue(const std::string &, float, uint8_t);
  bool setParameterValue(const std::string &, uint64_t);
  bool setParameterValue(const std::string &, uint64_t, uint8_t);
  bool setParameterValue(const std::string &, int64_t);
  bool setParameterValue(const std::string &, int64_t, uint8_t);
  bool setParameterValue(const std::string &, uint32_t);
  bool setParameterValue(const std::string &, uint32_t, uint8_t);
  bool setParameterValue(const std::string &, int32_t);
  bool setParameterValue(const std::string &, int32_t, uint8_t);
  bool setParameterValue(const std::string &, uint16_t);
  bool setParameterValue(const std::string &, uint16_t, uint8_t);
  bool setParameterValue(const std::string &, int16_t);
  bool setParameterValue(const std::string &, int16_t, uint8_t);
  bool setParameterValue(const std::string &, uint8_t);
  bool setParameterValue(const std::string &, uint8_t, uint8_t);
  bool setParameterValue(const std::string &, int8_t);
  bool setParameterValue(const std::string &, int8_t, uint8_t);

 public:
  static std::string toStr(const char *, size_t);
  static void toParamUnion(const std::string &, int, ParamUnion &);
  static bool registerParamWithCompId(const std::string &, uint8_t);
  static uint8_t getCompIdForParam(const std::string &);

 private:
  bool setParameterValue(const std::string &, const std::string &);
  bool setParameterType(const std::string &, int);

 private:
  std::map<std::string, std::string> paramValue_;
  std::map<std::string, std::pair<int, int>> paramIdType_;
  int paramCntr_ = 0;
};

}  // namespace fflow
