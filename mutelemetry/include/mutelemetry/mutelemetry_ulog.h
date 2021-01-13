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
#include <cstring>

namespace mutelemetry_ulog {

enum class ULogLevel : char {
  Emerg = 0,
  Alert,
  Crit,
  Err,
  Warning,
  Notice,
  Info,
  Debug
};

enum class ULogMessageType : uint8_t {
  B = 'B',
  I = 'I',
  M = 'M',
  F = 'F',
  P = 'P',
  A = 'A',
  D = 'D',
  L = 'L',
  S = 'S',
  O = 'O',
  R = 'R'
};

struct ULogFileHeader {
  ULogFileHeader() {}
  ULogFileHeader(uint64_t timestamp) : timestamp_(timestamp) {}
  uint8_t magic_[7] = {'U', 'L', 'o', 'g', 0x01, 0x12, 0x35};
  uint8_t version_ = 0x01;
  uint64_t timestamp_ = 0;

  void write_to(unsigned char buffer[16]) const {
    buffer[0] = magic_[0];
    buffer[1] = magic_[1];
    buffer[2] = magic_[2];
    buffer[3] = magic_[3];
    buffer[4] = magic_[4];
    buffer[5] = magic_[5];
    buffer[6] = magic_[6];
    buffer[7] = version_;
    *(reinterpret_cast<uint64_t *>(&buffer[8])) = timestamp_;
  }

  bool read_from(const uint8_t *buffer) {
    if (std::memcmp(magic_, buffer, 7) != 0) return false;
    version_ = buffer[7];
    timestamp_ = *(reinterpret_cast<const uint64_t *>(&buffer[8]));
    return true;
  }
};

static_assert(sizeof(ULogFileHeader) == 16, "ULogFileHeader bad size");

#pragma pack(push, 1)

struct ULogMessageHeader {
  uint16_t size_;
  uint8_t type_;
};

struct ULogMessageB {
  ULogMessageHeader h_;
  uint8_t compat_flags_[8];
  uint8_t incompat_flags_[8];
  uint64_t appended_offsets_[3];
};

struct ULogMessageI {
  ULogMessageHeader h_;
  uint8_t key_len_;
  char key_[255];
};

struct ULogMessageM {
  ULogMessageHeader h_;
  uint8_t is_continued_;
  uint8_t key_len_;
  char key_[255];
};

struct ULogMessageF {
  ULogMessageHeader h_;
  char format_[1500];
};

struct ULogMessageP {
  ULogMessageHeader h_;
  uint8_t key_len_;
  char key_[255];
};

struct ULogMessageA {
  ULogMessageHeader h_;
  uint8_t multi_id_;
  uint16_t msg_id_;
  char message_name_[255];
};

struct ULogMessageD {
  ULogMessageHeader h_;
  uint16_t msg_id_;
  uint8_t data_[1500];
};

struct ULogMessageL {
  ULogMessageHeader h_;
  char log_level_;
  uint64_t timestamp_;
  char message_[1500];
};

struct ULogMessageS {
  ULogMessageHeader h_;
  uint8_t sync_magic_[8];
};

struct ULogMessageO {
  ULogMessageHeader h_;
  uint16_t duration_;
};

struct ULogMessageR {
  ULogMessageHeader h_;
  uint16_t msg_id_;
};

#pragma pack(pop)

}  // namespace mutelemetry_ulog
