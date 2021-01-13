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

#include "mutelemetry/mutelemetry_tools.h"

#include "mutelemetry/mutelemetry_parse.h"
#include "mutelemetry/mutelemetry_ulog.h"

using namespace mutelemetry_tools;
using namespace mutelemetry_ulog;
using namespace mutelemetry_parse;

bool mutelemetry_tools::check_ulog_data_begin(const uint8_t *buffer) {
  const ULogMessageHeader *hdr =
      reinterpret_cast<const ULogMessageHeader *>(buffer);
  return ULogMessageType(hdr->type_) == ULogMessageType::A;
}

bool mutelemetry_tools::check_ulog_valid(const uint8_t *buffer) {
  bool valid = MutelemetryParser::getInstance().parse(buffer);

  if (valid && check_ulog_data_begin(buffer)) {
    const ULogMessageHeader *hdr =
        reinterpret_cast<const ULogMessageHeader *>(buffer);
    const uint8_t *data_buffer = buffer + sizeof(*hdr) + hdr->size_;
    valid = MutelemetryParser::getInstance().parse(data_buffer);
  }

  return valid;
}
