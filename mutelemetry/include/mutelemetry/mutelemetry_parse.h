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

#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mutelemetry_parse {

struct MessageLog {
  char level_;
  uint64_t timestamp_;
  std::string msg_;
};

using LogHandler = std::function<bool(const MessageLog &)>;

struct MessageData {
  std::string name_;
  uint64_t timestamp_;
  std::vector<std::pair<std::string, double>> structured_data_;
};

using DataHandler = std::function<bool(const MessageData &)>;

// WARNING: single threaded
class MutelemetryParser {
  MutelemetryParser() : state_(STATE_HEADER) {}
  MutelemetryParser(const MutelemetryParser &) = delete;
  MutelemetryParser &operator=(const MutelemetryParser &) = delete;

 public:
  static inline MutelemetryParser &getInstance() {
    static MutelemetryParser instance_ = {};
    return instance_;
  }

 public:
  enum ParseState {
    STATE_HEADER = 0,
    STATE_FLAGS,
    STATE_DEFINITION,
    STATE_DEFINITION_OR_DATA,
    STATE_INVALID
  };

  enum FormatType {
    UINT8,
    UINT16,
    UINT32,
    UINT64,
    INT8,
    INT16,
    INT32,
    INT64,
    FLOAT,
    DOUBLE,
    BOOL,
    CHAR,
    OTHER
  };

  struct Field {
    Field() : array_size_(1) {}
    FormatType type_;
    std::string field_name_;
    std::string other_type_id_;
    int array_size_;
  };

  struct Parameter {
    std::string name_;
    union {
      int32_t val_int_;
      float val_real_;
    } value_;
    FormatType val_type_;
  };

  struct Format {
    Format() : padding_(0) {}
    std::string name_;
    std::vector<Field> fields_;
    int padding_;
  };

  struct Subscription {
    Subscription() : msg_id_(0), multi_id_(0), format_(nullptr) {}
    uint16_t msg_id_;
    uint8_t multi_id_;
    std::string message_name_;
    const Format *format_;
  };

  struct Timeseries {
    std::vector<uint64_t> timestamps_;
    std::vector<std::pair<std::string, std::vector<double>>> data_;

    std::shared_ptr<MessageData> get_latest_data(
        const std::string &name) const {
      std::shared_ptr<MessageData> md = std::make_shared<MessageData>();
      md->name_ = name;
      md->timestamp_ = timestamps_.back();
      for (const auto &el : data_)
        md->structured_data_.push_back({el.first, el.second.back()});
      return md;
    }
  };

 private:
  ParseState parse_definition(const uint8_t *);
  ParseState parse_data(const uint8_t *);
  ParseState parse_header(const uint8_t *, uint16_t size = 0);
  ParseState parse_flags(const uint8_t *, uint16_t size = 0);
  ParseState parse_info(const uint8_t *, uint16_t);
  ParseState parse_multi(const uint8_t *, uint16_t);
  ParseState parse_format(const uint8_t *, uint16_t);
  ParseState parse_param(const uint8_t *, uint16_t);
  ParseState parse_add(const uint8_t *, uint16_t);
  ParseState parse_data(const uint8_t *, uint16_t);
  ParseState parse_logged(const uint8_t *, uint16_t);
  ParseState parse_sync(const uint8_t *, uint16_t);
  ParseState parse_dropout(const uint8_t *, uint16_t);
  ParseState parse_remove(const uint8_t *, uint16_t);
  const uint8_t *parse_data_intl(Timeseries &, const Format *, const uint8_t *,
                                 size_t *);
  Timeseries create_timeseries(const Format *);

 public:
  bool parse(const uint8_t *);
  inline bool set_data_handler(const DataHandler &h) {
    data_handler_ = h;
    return true;
  }
  inline bool set_log_handler(const LogHandler &h) {
    log_handler_ = h;
    return true;
  }

 private:
  ParseState state_;
  uint64_t start_time_;
  std::unordered_map<std::string, Timeseries> timeseries_;
  std::unordered_map<std::string, Format> formats_;
  std::unordered_map<uint16_t, Subscription> subscriptions_;
  std::unordered_map<std::string, std::string> info_;
  std::unordered_map<std::string, std::list<std::string>> multi_;
  std::unordered_set<std::string> with_multi_id_;
  std::vector<Parameter> parameters_;
  std::vector<MessageLog> message_logs_;
  LogHandler log_handler_ = {([](const MessageLog &) { return true; })};
  DataHandler data_handler_ = {([](const MessageData &) { return true; })};
};

}  // namespace mutelemetry_parse
