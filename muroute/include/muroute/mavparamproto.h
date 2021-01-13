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

#ifndef MAVPARAMCOMPONENT_H
#define MAVPARAMCOMPONENT_H

#include <muflow/sparseutils.h>

#include "mavlink2/common/mavlink.h"
#include "muroute.h"
#include "subsystem.h"

namespace fflow {

class MavParamProto : public IMavlinkProtocol {
  static constexpr size_t proto_table_len = 6;
  fflow::RouteSystemPtr roster_;

 public:
  MavParamProto() = default;
  virtual ~MavParamProto() = default;

 public:
  void setRoster(fflow::RouteSystemPtr roster) override { roster_ = roster; }
  fflow::RouteSystemPtr getRoster() override { return roster_; }

  const fflow::message_handler_note_t *get_table() const {
    return &proto_table_[0];
  }

  size_t get_table_len() const { return proto_table_len; }

  /* ---------------------------------------
   * Protocol callback definitions
   */
  const fflow::message_handler_note_t proto_table_[proto_table_len] = {
      // BASE PARAMETER PROTOCOL
      {MAVLINK_MSG_ID_PARAM_REQUEST_LIST,
       [this](uint8_t *a, size_t b, fflow::SparseAddress c,
              BaseComponentPtr d) -> fflow::pointprec_t {
         return param_request_list_handler(a, b, c, d);
       }},
      {MAVLINK_MSG_ID_PARAM_REQUEST_READ,
       [this](uint8_t *a, size_t b, fflow::SparseAddress c,
              BaseComponentPtr d) -> fflow::pointprec_t {
         return param_request_read_handler(a, b, c, d);
       }},
      {MAVLINK_MSG_ID_PARAM_SET,
       [this](uint8_t *a, size_t b, fflow::SparseAddress c, BaseComponentPtr d)
           -> fflow::pointprec_t { return param_set_handler(a, b, c, d); }},
      // EXTENDED PARAMETER PROTOCOL
      {MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST,
       [this](uint8_t *a, size_t b, fflow::SparseAddress c,
              BaseComponentPtr d) -> fflow::pointprec_t {
         return param_request_list_handler_ext(a, b, c, d);
       }},
      {MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ,
       [this](uint8_t *a, size_t b, fflow::SparseAddress c,
              BaseComponentPtr d) -> fflow::pointprec_t {
         return param_request_read_handler_ext(a, b, c, d);
       }},
      {MAVLINK_MSG_ID_PARAM_EXT_SET,
       [this](uint8_t *a, size_t b, fflow::SparseAddress c,
              BaseComponentPtr d) -> fflow::pointprec_t {
         return param_set_handler_ext(a, b, c, d);
       }}};

 protected:
  void send_mavlink_message(mavlink_message_t &, int, int,
                            int dest_comp_id = 0);

  void send_parameter(int, int, int, int dest_comp_id = 0);
  void send_parameter(const char *, int, int, int dest_comp_id = 0);
  void send_parameters(int, int, int dest_comp_id = 0);

  virtual fflow::pointprec_t param_request_list_handler(uint8_t *, size_t,
                                                        fflow::SparseAddress,
                                                        BaseComponentPtr);
  virtual fflow::pointprec_t param_request_read_handler(uint8_t *, size_t,
                                                        fflow::SparseAddress,
                                                        BaseComponentPtr);
  virtual fflow::pointprec_t param_set_handler(uint8_t *, size_t,
                                               fflow::SparseAddress,
                                               BaseComponentPtr);

  void send_parameter_ext(int, int, int, int dest_comp_id = 0);
  void send_parameter_ext(const char *, int, int, int dest_comp_id = 0);
  void send_parameters_ext(int, int, int dest_comp_id = 0);

  virtual fflow::pointprec_t param_request_list_handler_ext(
      uint8_t *, size_t, fflow::SparseAddress, BaseComponentPtr);
  virtual fflow::pointprec_t param_request_read_handler_ext(
      uint8_t *, size_t, fflow::SparseAddress, BaseComponentPtr);
  virtual fflow::pointprec_t param_set_handler_ext(uint8_t *, size_t,
                                                   fflow::SparseAddress,
                                                   BaseComponentPtr);

  bool decodeParameterValue(mavlink_param_value_t &, int,
                            const MavParams::ParamUnion &);

 protected:
  static constexpr float epsilon = std::numeric_limits<float>::epsilon();
};

}  // namespace fflow

#endif  // MAVPARAMCOMPONENT_H
