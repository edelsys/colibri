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

#ifndef MUCONFIG_H
#define MUCONFIG_H

#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace muconfig {

enum class TYPE : uint8_t {
  UNDEF,
  BOOL,
  INT,
  DOUBLE,
  STRING,
  TABLE,        // NI
  ARRAY,        // |
  TABLE_ARRAY,  // |
  TIME,         // |
  DATE,         // |
  DATETIME,     // V
  OFFSET,       // NI
};

enum class LANG : uint8_t {
  UNKNOWN,
  JSON,  // NI
  TOML,
  YAML,  // NI
};

struct MuConfObject {
  TYPE t_;
  std::vector<uint8_t> v_;

  MuConfObject() : t_(TYPE::UNDEF) { v_.clear(); }

  std::shared_ptr<std::string> getValueString() const;

  template <typename T>
  std::shared_ptr<T> getValueSimple() const {
    std::shared_ptr<T> objp = {};

    switch (t_) {
      case TYPE::BOOL: {
        if (typeid(T) != typeid(bool)) break;
        objp = std::make_shared<T>();
        *objp = static_cast<bool>(v_[0]);
      } break;

      case TYPE::INT: {
        if (typeid(T) != typeid(int)) break;
        objp = std::make_shared<T>();
        std::memcpy(objp.get(), v_.data(), sizeof(int));
      } break;

      case TYPE::DOUBLE: {
        if (typeid(T) != typeid(double)) break;
        objp = std::make_shared<T>();
        std::memcpy(objp.get(), v_.data(), sizeof(double));
      } break;

      default:
        std::cout << "Unsupported type\n";
        break;
    }

    return objp;
  }
};

class MuConfig {
 public:
  MuConfig() : name_("Unknown"), l_(LANG::UNKNOWN) {}
  virtual ~MuConfig() {}

 public:
  virtual bool isOk() { return false; }
  virtual bool parse() { return false; }
  virtual const std::string &getName() const { return name_; }
  virtual LANG getLang() const { return l_; }
  virtual void print() { std::cout << "\n===== " << getName() << " =====\n"; }

  virtual std::unique_ptr<MuConfObject> getObject(const std::string &key,
                                                  TYPE type) = 0;

 public:
  static std::unique_ptr<MuConfig> createConfig(
      const std::string &filename = "", LANG lang = LANG::TOML);

 protected:
  std::string name_;
  LANG l_;
};

}  // namespace muconfig

#endif  // MUCONFIG_H
