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

#include "mutelemetry/mutelemetry.h"

#include <assert.h>
#include <memory>

#include <glog/logging.h>
#include <muconfig/muconfig.h>
#include <boost/date_time.hpp>

using namespace std;
using namespace muconfig;
using namespace mutelemetry;
using namespace mutelemetry_ulog;
using namespace mutelemetry_tools;

MuTelemetry MuTelemetry::instance_ = {};

bool MuTelemetry::read_config(const string &file) {
  unique_ptr<MuConfig> cfg = MuConfig::createConfig(file);
  bool with_local_log = false;
  bool with_network = false;
  bool res = false;

  if (cfg != nullptr && cfg->isOk()) {
    const string s_with_network = "mutelemetry.with_network";
    auto o_with_network = cfg->getObject(s_with_network, TYPE::BOOL);
    if (o_with_network) {
      auto with_networkp = o_with_network->getValueSimple<bool>();
      if (with_networkp) with_network = *with_networkp;
    }

    const string s_with_local_log = "mutelemetry.with_local_log";
    auto o_with_local_log = cfg->getObject(s_with_local_log, TYPE::BOOL);
    if (o_with_local_log) {
      auto with_local_logp = o_with_local_log->getValueSimple<bool>();
      if (with_local_logp) with_local_log = *with_local_logp;
    }

    if (with_local_log) {
      string log_dir = "";
      const string s_log_dir = "mutelemetry.log_directory_path";
      auto o_log_dir = cfg->getObject(s_log_dir, TYPE::STRING);
      if (o_log_dir) {
        auto log_dirp = o_log_dir->getValueString();
        if (log_dirp) log_dir = *log_dirp;
      }

      log_dir_ = log_dir;
    }

    with_network_ = with_network;
    with_local_log_ = with_local_log;
    res = true;
  }

  return res;
}

bool MuTelemetry::init(fflow::RouteSystemPtr roster, bool rt) {
  MuTelemetry &instance = getInstance();
  if (instance.start_timestamp_ != 0) {
    LOG(INFO) << "MuTelemetry already initialized\n";
    return false;
  }

  // put component on the bus
  if (roster) {
    instance.lcomp_ = new LoggingComponent();
    if (instance.lcomp_) {
      instance.lcomp_->setRoster(roster);
      roster->addComponent(instance.lcomp_);
    }
  }

  instance.start_timestamp_ = instance.timestamp();
  if (instance.read_config()) {
    if (instance.with_network_) {
      instance.with_network_ =
          instance.streamer_.init(roster, &instance.net_queue_);
      if (instance.with_network_) {
        LOG(INFO) << "Starting MuTelemetry streaming\n";
        instance.streamer_.run(rt);
      }
    }

    if (!instance.with_network_)
      LOG(INFO) << "MuTelemetry streaming is disabled\n";

    if (instance.with_local_log_) {
      stringstream filename;
      boost::posix_time::ptime time_epoch(boost::gregorian::date(1970, 1, 1));
      boost::posix_time::ptime now =
          time_epoch +
          boost::posix_time::microseconds(instance_.start_timestamp_);

      filename << instance.log_dir_;
      if (instance.log_dir_ != "" &&
          instance.log_dir_[instance.log_dir_.length() - 1] != '/')
        filename << '/';

      filename << "mutelemetry_"
               << boost::posix_time::to_iso_extended_string(now) << ".ulg";
      instance.log_file_ = filename.str();

      instance.with_local_log_ =
          instance.logger_.init(instance.log_file_, &instance.log_queue_);
      if (instance.with_local_log_) {
        LOG(INFO) << "MuTelemetry log file: " << instance.logger_.get_logname()
                  << endl;
        instance.logger_.start();
      }
    }

    if (!instance.with_local_log_)
      LOG(INFO) << "MuTelemetry logging is disabled\n";
  }

  if (instance.is_enabled())
    instance.create_header_and_flags();
  else
    LOG(INFO) << "MuTelemetry is disabled\n";

  return instance.is_enabled();
}

void MuTelemetry::release() {
  MuTelemetry &instance = getInstance();

  if (instance.lcomp_) {
    fflow::RouteSystemPtr roster = instance.lcomp_->getRoster();
    if (roster) {
      LoggingComponent *lcomp = dynamic_cast<LoggingComponent *>(
          roster->getComponent(instance.lcomp_->getId()));
      if (lcomp) roster->removeComponent(lcomp->getId());
    }

    delete instance.lcomp_;
    instance.lcomp_ = nullptr;
  }

  // TODO: stop logger and streamer if they are running - need start/stop
  // interface for it
}

bool MuTelemetry::create_header_and_flags() {
  if (!is_enabled()) return true;

  ULogFileHeader fh(start_timestamp_);
  SerializedData fh_buffer(sizeof(fh));
  fh.write_to(fh_buffer.data());
  SerializedDataPtr fhp = make_shared<SerializedData>(move(fh_buffer));
  to_io(fhp);

  ULogMessageB mB = {};
  mB.h_.size_ = sizeof(mB) - sizeof(mB.h_);
  mB.h_.type_ = static_cast<uint8_t>(ULogMessageType::B);
  SerializedData mb_buffer(sizeof(mB));
  memcpy(mb_buffer.data(), &mB, sizeof(mB));
  SerializedDataPtr mbp = make_shared<SerializedData>(move(mb_buffer));
  to_io(mbp);

  return true;
}

bool MuTelemetry::register_param(const string &key, int32_t value) {
  if (!is_enabled()) return true;

  bool res = true;
  ULogMessageP mP = {};
  mP.h_.type_ = static_cast<uint8_t>(ULogMessageType::P);
  size_t len = sizeof(value);
  mP.key_len_ = snprintf(mP.key_, sizeof(mP.key_), "%s", key.c_str());
  size_t msg_size = sizeof(mP) - sizeof(mP.key_) + mP.key_len_;

  if (len < (sizeof(mP) - msg_size)) {
    uint8_t *buffer = reinterpret_cast<uint8_t *>(&mP);
    memcpy(&buffer[msg_size], &value, len);
    msg_size += len;
    mP.h_.size_ = msg_size - sizeof(mP.h_);
    SerializedData mp_buffer(buffer, buffer + msg_size);
    SerializedDataPtr mpp = make_shared<SerializedData>(move(mp_buffer));
    to_io(mpp);
  } else {
    res = false;
    assert(0);
  }

  return res;
}

bool MuTelemetry::register_param(const string &key, float value) {
  if (!is_enabled()) return true;

  bool res = true;
  ULogMessageP mP = {};
  mP.h_.type_ = static_cast<uint8_t>(ULogMessageType::P);
  size_t len = sizeof(value);
  mP.key_len_ = snprintf(mP.key_, sizeof(mP.key_), "%s", key.c_str());
  size_t msg_size = sizeof(mP) - sizeof(mP.key_) + mP.key_len_;

  if (len < (sizeof(mP) - msg_size)) {
    uint8_t *buffer = reinterpret_cast<uint8_t *>(&mP);
    memcpy(&buffer[msg_size], &value, len);
    msg_size += len;
    mP.h_.size_ = msg_size - sizeof(mP.h_);
    SerializedData mp_buffer(buffer, buffer + msg_size);
    SerializedDataPtr mpp = make_shared<SerializedData>(move(mp_buffer));
    to_io(mpp);
  } else {
    res = false;
    assert(0);
  }

  return res;
}

bool MuTelemetry::register_info(const string &key, const uint32_t value_) {
  if (!is_enabled()) return true;

  uint32_t value = value_;

  bool res = true;
  ULogMessageI mI = {};
  mI.h_.type_ = static_cast<uint8_t>(ULogMessageType::I);
  size_t len = sizeof(value);

  mI.key_len_ = snprintf(mI.key_, sizeof(mI.key_), "uint32_t %s", key.c_str());
  size_t msg_size = sizeof(mI) - sizeof(mI.key_) + mI.key_len_;

  if (len < (sizeof(mI) - msg_size)) {
    uint8_t *buffer = reinterpret_cast<uint8_t *>(&mI);
    memcpy(&buffer[msg_size], &value, len);
    msg_size += len;
    mI.h_.size_ = msg_size - sizeof(mI.h_);
    SerializedData mi_buffer(buffer, buffer + msg_size);
    SerializedDataPtr mip = make_shared<SerializedData>(move(mi_buffer));
    to_io(mip);
  } else {
    res = false;
    assert(0);
  }

  return res;
}

bool MuTelemetry::register_info(const string &key, const string &value) {
  if (!is_enabled()) return true;

  bool res = true;
  ULogMessageI mI = {};
  mI.h_.type_ = static_cast<uint8_t>(ULogMessageType::I);
  size_t len = value.length();
  mI.key_len_ =
      snprintf(mI.key_, sizeof(mI.key_), "char[%zu] %s", len, key.c_str());
  size_t msg_size = sizeof(mI) - sizeof(mI.key_) + mI.key_len_;

  if (len < (sizeof(mI) - msg_size)) {
    uint8_t *buffer = reinterpret_cast<uint8_t *>(&mI);
    memcpy(&buffer[msg_size], value.c_str(), len);
    msg_size += len;
    mI.h_.size_ = msg_size - sizeof(mI.h_);
    SerializedData mi_buffer(buffer, buffer + msg_size);
    SerializedDataPtr mip = make_shared<SerializedData>(move(mi_buffer));
    to_io(mip);
  } else {
    res = false;
    assert(0);
  }

  return res;
}

bool MuTelemetry::register_info_multi(const string &key, const string &value,
                                      bool is_continued) {
  if (!is_enabled()) return true;

  bool res = true;
  ULogMessageM mM = {};
  mM.h_.type_ = static_cast<uint8_t>(ULogMessageType::M);
  size_t len = value.length();
  mM.is_continued_ = is_continued;
  mM.key_len_ =
      snprintf(mM.key_, sizeof(mM.key_), "char[%zu] %s", len, key.c_str());
  size_t msg_size = sizeof(mM) - sizeof(mM.key_) + mM.key_len_;

  if (len < (sizeof(mM) - msg_size)) {
    uint8_t *buffer = reinterpret_cast<uint8_t *>(&mM);
    memcpy(&buffer[msg_size], value.c_str(), len);
    msg_size += len;
    mM.h_.size_ = msg_size - sizeof(mM.h_);
    SerializedData mm_buffer(buffer, buffer + msg_size);
    SerializedDataPtr mmp = make_shared<SerializedData>(move(mm_buffer));
    to_io(mmp);
  } else {
    res = false;
    assert(0);
  }

  return res;
}

bool MuTelemetry::register_data_format(const string &type_name,
                                       const string &fields) {
  if (!is_enabled()) return true;

  ULogMessageF mF = {};

  string format = type_name + string(":uint64_t timestamp;") + fields;
  mF.h_.type_ = static_cast<uint8_t>(ULogMessageType::F);
  int format_len =
      snprintf(mF.format_, format.size() + 1, "%s", format.c_str());
  assert(format_len == format.size());
  size_t msg_size = sizeof(mF) - sizeof(mF.format_) + format_len;
  mF.h_.size_ = msg_size - sizeof(mF.h_);

  SerializedData mf_buffer(msg_size);
  memcpy(mf_buffer.data(), &mF, msg_size);
  SerializedDataPtr mfp = make_shared<SerializedData>(move(mf_buffer));
  to_io(mfp);

  return true;
}

bool MuTelemetry::store_data_intl(const vector<uint8_t> &data,
                                  const string &type_name,
                                  const string &annotation,
                                  uint64_t timestamp) {
  if (!is_enabled()) return true;

  bool res = true;
  ULogMessageA mA = {};
  mA.h_.type_ = static_cast<uint8_t>(ULogMessageType::A);
  mA.msg_id_ = get_msg_id(type_name);
  mA.multi_id_ = get_multi_id(mA.msg_id_, annotation);

  int name_len = type_name.length();
  memcpy(mA.message_name_, type_name.c_str(), name_len);

  size_t ma_msg_size = sizeof(mA) - sizeof(mA.message_name_) + name_len;
  mA.h_.size_ = ma_msg_size - sizeof(mA.h_);

  ULogMessageD mD = {};
  mD.h_.type_ = static_cast<uint8_t>(ULogMessageType::D);
  mD.msg_id_ = mA.msg_id_;

  if (data.size() <= sizeof(mD.data_)) {
    size_t len = sizeof(timestamp);
    memcpy(&mD.data_[0], &timestamp, len);
    memcpy(&mD.data_[len], data.data() + len, data.size() - len);
    len = data.size();

    size_t md_msg_size = sizeof(mD) - sizeof(mD.data_) + len;
    mD.h_.size_ = md_msg_size - sizeof(mD.h_);

    SerializedData buffer(ma_msg_size + md_msg_size);
    memcpy(&buffer[0], &mA, ma_msg_size);
    memcpy(&buffer[ma_msg_size], &mD, md_msg_size);

    SerializedDataPtr datap = make_shared<SerializedData>(move(buffer));
    to_io(datap);
  } else {
    res = false;
    assert(0);
  }

  return res;
}

bool MuTelemetry::store_message(const string &message,
                                mutelemetry_ulog::ULogLevel level) {
  uint64_t tstmp = timestamp();

  if (!is_enabled()) return true;

  ULogMessageL mL = {};
  if (message.length() + 1 > sizeof(mL.message_)) {
    assert(0);
    return false;
  }

  mL.h_.type_ = static_cast<uint8_t>(ULogMessageType::L);
  mL.log_level_ = char(level);
  mL.timestamp_ = tstmp;

  int len = snprintf(mL.message_, sizeof(mL.message_), "%s", message.c_str());
  size_t msg_size = sizeof(mL) - sizeof(mL.message_) + len;
  mL.h_.size_ = msg_size - sizeof(mL.h_);

  SerializedData ml_buffer(msg_size);
  memcpy(ml_buffer.data(), &mL, msg_size);
  SerializedDataPtr mlp = make_shared<SerializedData>(move(ml_buffer));
  to_io(mlp);

  return true;
}
