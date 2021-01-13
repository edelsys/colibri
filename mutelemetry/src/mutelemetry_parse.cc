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

#include "mutelemetry/mutelemetry_parse.h"

#include <assert.h>
#include <glog/logging.h>

#include <boost/utility/string_view.hpp>
#include <iomanip>

#include "mutelemetry/mutelemetry_ulog.h"

//#define RELAXED_STATE_MACHINE
//#define KEEP_DATA

using namespace std;
using namespace mutelemetry_ulog;
using namespace mutelemetry_parse;

template <typename T>
static string int_to_hex(T i) {
  stringstream stream;
  stream << "0x" << setfill('0') << setw(sizeof(T) * 2) << hex << i;
  return stream.str();
}

static vector<boost::string_view> split_string(
    const boost::string_view &strToSplit, char delimeter) {
  vector<boost::string_view> splitted_strings;
  splitted_strings.reserve(4);

  size_t pos = 0;
  while (pos < strToSplit.size()) {
    size_t new_pos = strToSplit.find_first_of(delimeter, pos);
    if (new_pos == std::string::npos) {
      new_pos = strToSplit.size();
    }
    boost::string_view sv = {&strToSplit.data()[pos], new_pos - pos};
    splitted_strings.push_back(sv);
    pos = new_pos + 1;
  }
  return splitted_strings;
}

bool MutelemetryParser::parse(const uint8_t *buffer) {
  ParseState new_state = state_;

  switch (state_) {
    case STATE_HEADER:
      new_state = parse_header(buffer);
#ifndef RELAXED_STATE_MACHINE
      assert(new_state == STATE_FLAGS);
#endif
      break;

    case STATE_FLAGS:
      new_state = parse_flags(buffer);
#ifndef RELAXED_STATE_MACHINE
      assert(new_state == STATE_DEFINITION);
#endif
      break;

    case STATE_DEFINITION:
      new_state = parse_definition(buffer);
      break;
    case STATE_DEFINITION_OR_DATA:
      new_state = parse_data(buffer);
      break;

    case STATE_INVALID:
    default:
      break;
  }

  state_ = new_state;
  return state_ != STATE_INVALID;
}

MutelemetryParser::Timeseries MutelemetryParser::create_timeseries(
    const Format *format) {
  std::function<void(const Format &format, const string &prefix)> appendVector;

  Timeseries timeseries;

  appendVector = [&appendVector, this, &timeseries](const Format &format,
                                                    const string &prefix) {
    for (const auto &field : format.fields_) {
      if (boost::string_view(field.field_name_).starts_with("_padding"))
        continue;

      string new_prefix = prefix + "/" + field.field_name_;
      for (int i = 0; i < field.array_size_; i++) {
        std::string array_suffix = "";
        if (field.array_size_ > 1) {
          char buff[10];
          sprintf(buff, ".%02d", i);
          array_suffix = buff;
        }
        if (field.type_ != OTHER) {
          timeseries.data_.push_back(
              {new_prefix + array_suffix, vector<double>()});
        } else {
          appendVector(this->formats_.at(field.other_type_id_),
                       new_prefix + array_suffix);
        }
      }
    }
  };

  appendVector(*format, {});
  return timeseries;
}

MutelemetryParser::ParseState MutelemetryParser::parse_header(
    const uint8_t *buffer, uint16_t /*size*/) {
  ULogFileHeader file_header{};
  assert(state_ == STATE_HEADER);
  ParseState next_state =
#ifdef RELAXED_STATE_MACHINE
      state_;
#else
      STATE_INVALID;
#endif

  if (file_header.read_from(buffer)) {
    start_time_ = file_header.timestamp_;
    next_state = STATE_FLAGS;
  }
  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_flags(
    const uint8_t *buffer, uint16_t /*size*/) {
  assert(state_ == STATE_FLAGS);
  ParseState next_state =
#ifdef RELAXED_STATE_MACHINE
      state_;
#else
      STATE_INVALID;
#endif
  const ULogMessageHeader *hdr =
      reinterpret_cast<const ULogMessageHeader *>(buffer);
  if (ULogMessageType(hdr->type_) == ULogMessageType::B) {
    const uint8_t *message = buffer + sizeof(*hdr);
    const uint8_t *incompat_flags = message + 8;

    bool contains_appended_data = incompat_flags[0] & 0x1;
    bool has_unknown_incompat_bits = false;

    next_state = STATE_DEFINITION;

    if (incompat_flags[0] & ~0x1) has_unknown_incompat_bits = true;

    for (int i = 1; i < 8 && !has_unknown_incompat_bits; ++i)
      if (incompat_flags[i]) {
        has_unknown_incompat_bits = true;
        break;
      }

    if (has_unknown_incompat_bits)
      next_state = STATE_INVALID;
    else if (contains_appended_data) {
      uint64_t appended_offsets[3];
      memcpy(appended_offsets, message + 16, sizeof(appended_offsets));
      if (appended_offsets[0] > 0) {
        ;  // do nothing for now
      }
    }
  }
  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_info(
    const uint8_t *message, uint16_t msg_size) {
  ParseState next_state = state_;
  uint8_t key_len = message[0];
  message++;

  string raw_key(reinterpret_cast<const char *>(message), key_len);
  message += key_len;

  auto key_parts = split_string(raw_key, ' ');
  string key = key_parts[1].to_string();

  string value;
  if (key_parts[0].starts_with("char[")) {
    value =
        string(reinterpret_cast<const char *>(message), msg_size - key_len - 1);
  } else if (key_parts[0] == boost::string_view("bool")) {
    bool val = *reinterpret_cast<const bool *>(key_parts[0].data());
    value = to_string(val);
  } else if (key_parts[0] == boost::string_view("uint8_t")) {
    uint8_t val = *reinterpret_cast<const uint8_t *>(key_parts[0].data());
    value = to_string(val);
  } else if (key_parts[0] == boost::string_view("int8_t")) {
    int8_t val = *reinterpret_cast<const int8_t *>(key_parts[0].data());
    value = to_string(val);
  } else if (key_parts[0] == boost::string_view("uint16_t")) {
    uint16_t val = *reinterpret_cast<const uint16_t *>(key_parts[0].data());
    value = to_string(val);
  } else if (key_parts[0] == boost::string_view("int16_t")) {
    int16_t val = *reinterpret_cast<const int16_t *>(key_parts[0].data());
    value = to_string(val);
  } else if (key_parts[0] == boost::string_view("uint32_t")) {
    uint32_t val = *reinterpret_cast<const uint32_t *>(key_parts[0].data());
    if (key_parts[1].starts_with("ver_") &&
        key_parts[1].ends_with("_release")) {
      value = int_to_hex(val);
    } else {
      value = to_string(val);
    }
  } else if (key_parts[0] == boost::string_view("int32_t")) {
    int32_t val = *reinterpret_cast<const int32_t *>(key_parts[0].data());
    value = to_string(val);
  } else if (key_parts[0] == boost::string_view("float")) {
    float val = *reinterpret_cast<const float *>(key_parts[0].data());
    value = to_string(val);
  } else if (key_parts[0] == boost::string_view("double")) {
    double val = *reinterpret_cast<const double *>(key_parts[0].data());
    value = to_string(val);
  } else if (key_parts[0] == boost::string_view("uint64_t")) {
    uint64_t val = *reinterpret_cast<const uint64_t *>(key_parts[0].data());
    value = to_string(val);
  } else if (key_parts[0] == boost::string_view("int64_t")) {
    int64_t val = *reinterpret_cast<const int64_t *>(key_parts[0].data());
    value = to_string(val);
  }

  info_.insert({key, value});
  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_multi(
    const uint8_t * /*message*/, uint16_t /*msg_size*/) {
  ParseState next_state = state_;
  // TODO: if key doesn't exist, insert new, otherwise add value
  // to the list of values
  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_format(
    const uint8_t *message, uint16_t msg_size) {
  ParseState next_state = state_;
  string str_format(reinterpret_cast<const char *>(message), msg_size);
  size_t pos = str_format.find(':');

  if (pos == std::string::npos) {
#ifdef RELAXED_STATE_MACHINE
    return next_state;
#else
    return STATE_INVALID;
#endif
  }

  string name = str_format.substr(0, pos);
  string fields = str_format.substr(pos + 1);

  Format format;
  auto fields_split = split_string(fields, ';');
  format.fields_.reserve(fields_split.size());
  for (auto field_section : fields_split) {
    auto field_pair = split_string(field_section, ' ');
    auto field_type = field_pair.at(0);
    auto field_name = field_pair.at(1);

    Field field;
    if (field_type.starts_with("int8_t")) {
      field.type_ = INT8;
      field_type.remove_prefix(6);
    } else if (field_type.starts_with("int16_t")) {
      field.type_ = INT16;
      field_type.remove_prefix(7);
    } else if (field_type.starts_with("int32_t")) {
      field.type_ = INT32;
      field_type.remove_prefix(7);
    } else if (field_type.starts_with("int64_t")) {
      field.type_ = INT64;
      field_type.remove_prefix(7);
    } else if (field_type.starts_with("uint8_t")) {
      field.type_ = UINT8;
      field_type.remove_prefix(7);
    } else if (field_type.starts_with("uint16_t")) {
      field.type_ = UINT16;
      field_type.remove_prefix(8);
    } else if (field_type.starts_with("uint32_t")) {
      field.type_ = UINT32;
      field_type.remove_prefix(8);
    } else if (field_type.starts_with("uint64_t")) {
      field.type_ = UINT64;
      field_type.remove_prefix(8);
    } else if (field_type.starts_with("double")) {
      field.type_ = DOUBLE;
      field_type.remove_prefix(6);
    } else if (field_type.starts_with("float")) {
      field.type_ = FLOAT;
      field_type.remove_prefix(5);
    } else if (field_type.starts_with("bool")) {
      field.type_ = BOOL;
      field_type.remove_prefix(4);
    } else if (field_type.starts_with("char")) {
      field.type_ = CHAR;
      field_type.remove_prefix(4);
    } else {
      field.type_ = OTHER;

      if (field_type.ends_with("]")) {
        boost::string_view helper = field_type;
        while (!helper.ends_with("[")) {
          helper.remove_suffix(1);
        }

        helper.remove_suffix(1);
        field.other_type_id_ = helper.to_string();

        while (!field_type.starts_with("[")) {
          field_type.remove_prefix(1);
        }

      } else {
        field.other_type_id_ = field_type.to_string();
      }
    }

    field.array_size_ = 1;

    if (field_type.size() > 0 && field_type[0] == '[') {
      field_type.remove_prefix(1);
      field.array_size_ = field_type[0] - '0';
      field_type.remove_prefix(1);

      while (field_type[0] != ']') {
        field.array_size_ = 10 * field.array_size_ + field_type[0] - '0';
        field_type.remove_prefix(1);
      }
    }

    if (field.type_ == UINT64 &&
        field_name == boost::string_view("timestamp")) {
      // skip
    } else {
      field.field_name_ = field_name.to_string();
      format.fields_.push_back(field);
    }
  }

  format.name_ = name;
  formats_[name] = move(format);
  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_param(
    const uint8_t *message, uint16_t /*msg_size*/) {
  ParseState next_state = state_;
  uint8_t key_len = message[0];
  string key(reinterpret_cast<const char *>(message) + 1, key_len);

  size_t pos = key.find(' ');

  if (pos == string::npos) {
#ifndef RELAXED_STATE_MACHINE
    next_state = STATE_INVALID;
#endif
    return next_state;
  }

  string type = key.substr(0, pos);

  Parameter param;
  param.name_ = key.substr(pos + 1);

  bool is_ok = true;
  if (type == "int32_t") {
    param.value_.val_int_ =
        *reinterpret_cast<const int32_t *>(message + 1 + key_len);
    param.val_type_ = INT32;
  } else if (type == "float") {
    param.value_.val_real_ =
        *reinterpret_cast<const float *>(message + 1 + key_len);
    param.val_type_ = FLOAT;
  } else {
    is_ok = false;
#ifndef RELAXED_STATE_MACHINE
    next_state = STATE_INVALID;
#endif
  }

  if (is_ok) parameters_.push_back(param);
  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_add(
    const uint8_t *message, uint16_t msg_size) {
  ParseState next_state = STATE_DEFINITION_OR_DATA;
  Subscription sub;

  sub.multi_id_ = *reinterpret_cast<const uint8_t *>(message);
  sub.msg_id_ =
      *reinterpret_cast<const uint16_t *>(message + sizeof(sub.multi_id_));
  size_t flen = sizeof(sub.multi_id_) + sizeof(sub.msg_id_);
  message += flen;
  sub.message_name_.assign(reinterpret_cast<const char *>(message),
                           msg_size - flen);

  const auto it = formats_.find(sub.message_name_);
  if (it != formats_.end()) sub.format_ = &it->second;
  subscriptions_.insert({sub.msg_id_, sub});
  if (sub.multi_id_ > 0) with_multi_id_.insert(sub.message_name_);

  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_definition(
    const uint8_t *buffer) {
  const ULogMessageHeader *hdr =
      reinterpret_cast<const ULogMessageHeader *>(buffer);
  const uint8_t *message = buffer + sizeof(*hdr);
  assert(state_ == STATE_DEFINITION);
  ParseState next_state =
#ifdef RELAXED_STATE_MACHINE
      state_;
#else
      STATE_INVALID;
#endif

  switch (ULogMessageType(hdr->type_)) {
    case ULogMessageType::I:
      next_state = parse_info(message, hdr->size_);
      break;
    case ULogMessageType::M:
      next_state = parse_multi(message, hdr->size_);
      break;
    case ULogMessageType::F:
      next_state = parse_format(message, hdr->size_);
      break;
    case ULogMessageType::P:
      next_state = parse_param(message, hdr->size_);
      break;
    case ULogMessageType::A:
      next_state = parse_add(message, hdr->size_);
      if (next_state == STATE_DEFINITION_OR_DATA) {
        hdr = reinterpret_cast<const ULogMessageHeader *>(message + hdr->size_);
        message = reinterpret_cast<const uint8_t *>(hdr) + sizeof(*hdr);
        if (ULogMessageType(hdr->type_) != ULogMessageType::D) break;
        next_state = parse_data(message, hdr->size_);
      }
      break;
    default:
      break;
  }

  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_data(
    const uint8_t *message, uint16_t /*msg_size*/) {
  uint16_t msg_id = *reinterpret_cast<const uint16_t *>(message);
  auto sub_it = subscriptions_.find(msg_id);

  if (sub_it != subscriptions_.end()) {
    const Subscription &sub = sub_it->second;
    string ts_name = sub.message_name_;

    if (with_multi_id_.count(ts_name) > 0) {
      char buff[10];
      sprintf(buff, ".%02d", sub.multi_id_);
      ts_name += string(buff);
    }

    auto ts_it = timeseries_.find(ts_name);
    if (ts_it == timeseries_.end())
      ts_it =
          timeseries_.insert({ts_name, create_timeseries(sub.format_)}).first;

    size_t index = 0;
    Timeseries &timeseries = ts_it->second;
    message += sizeof(msg_id);
    uint64_t tval = *reinterpret_cast<const uint64_t *>(message);
#ifndef KEEP_DATA
    timeseries.timestamps_.assign(1, tval);
#else
    timeseries.timestamps_.push_back(tval);
#endif
    message += sizeof(uint64_t);
    parse_data_intl(timeseries, sub.format_, message, &index);
    data_handler_(*timeseries.get_latest_data(ts_name));
  }

  return STATE_DEFINITION_OR_DATA;
}

MutelemetryParser::ParseState MutelemetryParser::parse_logged(
    const uint8_t *message, uint16_t msg_size) {
  ParseState next_state = STATE_DEFINITION_OR_DATA;
  MessageLog msg_log;
  msg_log.level_ = static_cast<char>(message[0]);
  message += sizeof(char);
  msg_log.timestamp_ = *reinterpret_cast<const uint64_t *>(message);
  message += sizeof(uint64_t);
  msg_log.msg_.assign(reinterpret_cast<const char *>(message),
                      msg_size - (sizeof(char) + sizeof(uint64_t)));
  log_handler_(msg_log);
#ifndef KEEP_DATA
  message_logs_.push_back(std::move(msg_log));
#endif
  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_sync(
    const uint8_t * /*message*/, uint16_t /*msg_size*/) {
  ParseState next_state = STATE_DEFINITION_OR_DATA;
  // TODO:
  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_dropout(
    const uint8_t * /*message*/, uint16_t /*msg_size*/) {
  ParseState next_state = STATE_DEFINITION_OR_DATA;
  // TODO:
  return next_state;
}

MutelemetryParser::ParseState MutelemetryParser::parse_remove(
    const uint8_t *message, uint16_t /*msg_size*/) {
  subscriptions_.erase(*reinterpret_cast<const uint16_t *>(message));
  return STATE_DEFINITION_OR_DATA;
}

MutelemetryParser::ParseState MutelemetryParser::parse_data(
    const uint8_t *buffer) {
  const ULogMessageHeader *hdr =
      reinterpret_cast<const ULogMessageHeader *>(buffer);
  const uint8_t *message = buffer + sizeof(*hdr);
  assert(state_ == STATE_DEFINITION_OR_DATA);
  ParseState next_state =
#ifdef RELAXED_STATE_MACHINE
      state_;
#else
      STATE_INVALID;
#endif
  switch (ULogMessageType(hdr->type_)) {
    case ULogMessageType::A:
      next_state = parse_add(message, hdr->size_);
      if (next_state != STATE_DEFINITION_OR_DATA) break;
      hdr = reinterpret_cast<const ULogMessageHeader *>(message + hdr->size_);
      message = reinterpret_cast<const uint8_t *>(hdr) + sizeof(*hdr);
      if (ULogMessageType(hdr->type_) != ULogMessageType::D) break;
    case ULogMessageType::D:
      next_state = parse_data(message, hdr->size_);
      break;
    case ULogMessageType::L:
      next_state = parse_logged(message, hdr->size_);
      break;
    case ULogMessageType::I:
      next_state = parse_info(message, hdr->size_);
      break;
    case ULogMessageType::M:
      next_state = parse_multi(message, hdr->size_);
      break;
    case ULogMessageType::P:
      next_state = parse_param(message, hdr->size_);
      break;
    case ULogMessageType::S:
      next_state = parse_sync(message, hdr->size_);
      break;
    case ULogMessageType::O:
      next_state = parse_dropout(message, hdr->size_);
      break;
    case ULogMessageType::R:
      next_state = parse_remove(message, hdr->size_);
      break;
    default:
      break;
  }

  return next_state;
}

const uint8_t *MutelemetryParser::parse_data_intl(Timeseries &timeseries,
                                                  const Format *format,
                                                  const uint8_t *message,
                                                  size_t *index) {
  for (const auto &field : format->fields_) {
    if (boost::string_view(field.field_name_).starts_with("_padding")) {
      message += field.array_size_;
      continue;
    }

    for (int array_pos = 0; array_pos < field.array_size_; array_pos++) {
      double value = 0;
      switch (field.type_) {
        case UINT8:
          value =
              static_cast<double>(*reinterpret_cast<const uint8_t *>(message));
          message += 1;
          break;
        case INT8:
          value =
              static_cast<double>(*reinterpret_cast<const int8_t *>(message));
          message += 1;
          break;
        case UINT16:
          value =
              static_cast<double>(*reinterpret_cast<const uint16_t *>(message));
          message += 2;
          break;
        case INT16:
          value =
              static_cast<double>(*reinterpret_cast<const int16_t *>(message));
          message += 2;
          break;
        case UINT32:
          value =
              static_cast<double>(*reinterpret_cast<const uint32_t *>(message));
          message += 4;
          break;
        case INT32:
          value =
              static_cast<double>(*reinterpret_cast<const int32_t *>(message));
          message += 4;
          break;
        case UINT64:
          value =
              static_cast<double>(*reinterpret_cast<const uint64_t *>(message));
          message += 8;
          break;
        case INT64:
          value =
              static_cast<double>(*reinterpret_cast<const int64_t *>(message));
          message += 8;
          break;
        case FLOAT:
          value =
              static_cast<double>(*reinterpret_cast<const float *>(message));
          message += 4;
          break;
        case DOUBLE:
          value = (*reinterpret_cast<const double *>(message));
          message += 8;
          break;
        case CHAR:
          value = static_cast<double>(*reinterpret_cast<const char *>(message));
          message += 1;
          break;
        case BOOL:
          value = static_cast<double>(*reinterpret_cast<const bool *>(message));
          message += 1;
          break;
        case OTHER: {
          auto child_format = formats_.at(field.other_type_id_);
          message += sizeof(uint64_t);
          message = parse_data_intl(timeseries, &child_format, message, index);
        } break;
      }

      if (field.type_ != OTHER)
#ifndef KEEP_DATA
        timeseries.data_[(*index)++].second.assign(1, value);
#else
        timeseries.data_[(*index)++].second.push_back(value);
#endif
    }
  }

  return message;
}
