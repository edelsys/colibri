#include "muroute/mavparams.h"

#include <assert.h>
#include <algorithm>
#include <cstring>
#include <sstream>
#include <string>

#include <glog/logging.h>

using namespace std;
using namespace fflow;

map<string, uint8_t> MavParams::paramIdCompId;
mutex MavParams::prot;

MavParams::ParamUnion::ParamUnion() : type(0) {
  memset(bytes, 0, max_param_value_len);
}

size_t MavParams::ParamUnion::getParamSize() const {
  size_t res = 0;

  switch (type) {
    case MAV_PARAM_EXT_TYPE_UINT8:
    case MAV_PARAM_EXT_TYPE_INT8:
      res = 1;
      break;
    case MAV_PARAM_EXT_TYPE_UINT16:
    case MAV_PARAM_EXT_TYPE_INT16:
      res = 2;
      break;
    case MAV_PARAM_EXT_TYPE_REAL32:
    case MAV_PARAM_EXT_TYPE_UINT32:
    case MAV_PARAM_EXT_TYPE_INT32:
      res = 4;
      break;
    case MAV_PARAM_EXT_TYPE_REAL64:
    case MAV_PARAM_EXT_TYPE_UINT64:
    case MAV_PARAM_EXT_TYPE_INT64:
      res = 8;
      break;
    case MAV_PARAM_EXT_TYPE_CUSTOM:
      res = max_param_value_len;
      break;
    default:
      break;
  }

  return res;
}

string MavParams::ParamUnion::toReadable() const {
  stringstream res;

  switch (type) {
    case MAV_PARAM_EXT_TYPE_UINT8:
      res << static_cast<unsigned>(param_uint8);
      break;
    case MAV_PARAM_EXT_TYPE_INT8:
      res << static_cast<int>(param_int8);
      break;
    case MAV_PARAM_EXT_TYPE_UINT16:
      res << param_uint16;
      break;
    case MAV_PARAM_EXT_TYPE_INT16:
      res << param_int16;
      break;
    case MAV_PARAM_EXT_TYPE_UINT32:
      res << param_uint32;
      break;
    case MAV_PARAM_EXT_TYPE_INT32:
      res << param_int32;
      break;
    case MAV_PARAM_EXT_TYPE_UINT64:
      res << param_uint64;
      break;
    case MAV_PARAM_EXT_TYPE_INT64:
      res << param_int64;
      break;
    case MAV_PARAM_EXT_TYPE_REAL32:
      res << param_float;
      break;
    case MAV_PARAM_EXT_TYPE_REAL64:
      res << param_double;
      break;
    case MAV_PARAM_EXT_TYPE_CUSTOM:
      res << toStr(reinterpret_cast<char const *>(bytes), max_param_value_len);
      break;
    default:
      break;
  }

  return res.str();
}

bool MavParams::ParamUnion::setParameterValue(float v, uint8_t t) {
  bool result = true;

  switch (t) {
    case MAV_PARAM_EXT_TYPE_UINT8:
      param_uint8 = static_cast<uint8_t>(v);
      break;
    case MAV_PARAM_EXT_TYPE_INT8:
      param_int8 = static_cast<int8_t>(v);
      break;
    case MAV_PARAM_EXT_TYPE_UINT16:
      param_uint16 = static_cast<uint16_t>(v);
      break;
    case MAV_PARAM_EXT_TYPE_INT16:
      param_int16 = static_cast<int16_t>(v);
      break;
    case MAV_PARAM_EXT_TYPE_UINT32:
      param_uint32 = static_cast<uint32_t>(v);
      break;
    case MAV_PARAM_EXT_TYPE_INT32:
      param_int32 = static_cast<int32_t>(v);
      break;
    case MAV_PARAM_EXT_TYPE_REAL32:
      param_float = v;
      break;
    case MAV_PARAM_EXT_TYPE_UINT64:
    case MAV_PARAM_EXT_TYPE_INT64:
    case MAV_PARAM_EXT_TYPE_REAL64:
    case MAV_PARAM_EXT_TYPE_CUSTOM:
    default:
      result = false;
      break;
  }

  if (result) type = t;

  return result;
}

bool MavParams::ParamUnion::decodeParameterValue(
    mavlink_param_value_t &param_value) const {
  bool result = true;

  switch (type) {
    case MAV_PARAM_TYPE_UINT8:
      param_value.param_value = param_uint8;
      break;
    case MAV_PARAM_TYPE_INT8:
      param_value.param_value = param_int8;
      break;
    case MAV_PARAM_TYPE_UINT16:
      param_value.param_value = param_uint16;
      break;
    case MAV_PARAM_TYPE_INT16:
      param_value.param_value = param_int16;
      break;
    case MAV_PARAM_TYPE_UINT32:
      param_value.param_value = param_uint32;
      break;
    case MAV_PARAM_TYPE_INT32:
      param_value.param_value = param_int32;
      break;
    case MAV_PARAM_TYPE_REAL32:
      param_value.param_value = param_float;
      break;
    case MAV_PARAM_TYPE_REAL64:
    case MAV_PARAM_TYPE_UINT64:
    case MAV_PARAM_TYPE_INT64:
    default:
      LOG(ERROR) << "PARAMETER TYPE=" << type
                 << " IS NOT SUPPORTED UNDER BASE PARAMETER PROTOCOL";
      result = false;
  }

  return result;
}

void MavParams::toParamUnion(const string &in_value, int in_type,
                             MavParams::ParamUnion &out_value) {
  memcpy(out_value.bytes, in_value.data(), in_value.size());
  out_value.type = in_type;
  assert(out_value.getParamSize() == in_value.size());
}

string MavParams::toStr(const char *buf, size_t buf_size) {
  return string(buf, find(buf, buf + buf_size, '\0'));
}

bool MavParams::registerParamWithCompId(const string &paramId, uint8_t compId) {
  lock_guard<mutex> lck(prot);
  bool result = false;
  if (paramIdCompId.find(paramId) == paramIdCompId.end() && compId > 0) {
    paramIdCompId[paramId] = compId;
    result = true;
  }
  return result;
}

uint8_t MavParams::getCompIdForParam(const std::string &paramId) {
  if (paramIdCompId.find(paramId) == paramIdCompId.end()) return 0;
  return paramIdCompId[paramId];
}

string MavParams::getParameterId(int paramIdx) {
  if (paramIdx < 0) return string();
  for (auto &it : paramIdType_) {
    if (it.second.first == paramIdx) return it.first;
  }
  return string();
}

int MavParams::getParameterIdx(const string &paramId) {
  int ret = -1;
  const auto &it = paramIdType_.find(paramId);
  if (it != paramIdType_.end()) ret = it->second.first;
  return ret;
}

int MavParams::getParameterType(const string &paramId) {
  int ret = -1;
  const auto &it = paramIdType_.find(paramId);
  if (it != paramIdType_.end()) ret = it->second.second;
  return ret;
}

string MavParams::getParameterValue(const string &paramId) {
  if (paramValue_.find(paramId) == paramValue_.end())
    return string();
  else
    return paramValue_[paramId];
}

bool MavParams::setParameterType(const string &paramId, int paramType) {
  if (paramId.empty() || paramType <= 0) {
    LOG(ERROR) << "Invalid argument - empty or not supported";
    return false;
  }

  if (paramIdType_.find(paramId) != paramIdType_.end()) {
    LOG(INFO) << "Parameter \"" << paramId << "\" type is already set";
    return true;
  }

  paramIdType_[paramId] = make_pair(++paramCntr_, paramType);
  return true;
}

bool MavParams::setParameterValue(const string &paramId, const string &value) {
  if (paramId.size() > max_param_id_len) {
    LOG(ERROR) << "Parameter name \"" << paramId << "\" is too long - max "
               << max_param_id_len << " symbols possible";
    return false;
  }

  if (value.size() > max_param_value_len) {
    LOG(ERROR) << "Parameter \"" << paramId << "\" value is too big";
    return false;
  }

  paramValue_[paramId] = value;
  return true;
}

bool MavParams::updateParameterValue(const string &param_id,
                                     const ParamUnion &u) {
  int type = getParameterType(param_id);
  if (type <= 0) return false;
  if (type != static_cast<int>(u.type)) return false;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str));
}

bool MavParams::setParameterValue(const string &paramId, const string &value,
                                  int type) {
  if (paramId.size() > max_param_id_len) {
    LOG(ERROR) << "Parameter name \"" << paramId << "\" is too long - max "
               << max_param_id_len << " symbols possible";
    return false;
  }

  if (value.size() > max_param_value_len) {
    LOG(ERROR) << "Parameter \"" << paramId << "\" value is too big";
    return false;
  }

  bool result = setParameterType(paramId, type);
  if (result) paramValue_[paramId] = value;

  return result;
}

bool MavParams::setParameterValue(const string &paramId, const string &value,
                                  int type, uint8_t compId) {
  bool result = false;
  if (setParameterValue(paramId, value, type))
    result = MavParams::registerParamWithCompId(paramId, compId);
  return result;
}

bool MavParams::setParameterValue(const string &param_id, double param_value) {
  ParamUnion u;
  u.param_double = param_value;
  u.type = MAV_PARAM_EXT_TYPE_REAL64;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_REAL64);
}

bool MavParams::setParameterValue(const string &param_id, double param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_double = param_value;
  u.type = MAV_PARAM_EXT_TYPE_REAL64;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_REAL64,
                           compId);
}

bool MavParams::setParameterValue(const string &param_id, float param_value) {
  ParamUnion u;
  u.param_float = param_value;
  u.type = MAV_PARAM_EXT_TYPE_REAL32;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_REAL32);
}

bool MavParams::setParameterValue(const string &param_id, float param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_float = param_value;
  u.type = MAV_PARAM_EXT_TYPE_REAL32;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_REAL32,
                           compId);
}

bool MavParams::setParameterValue(const string &param_id,
                                  uint64_t param_value) {
  ParamUnion u;
  u.param_uint64 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT64;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_UINT64);
}

bool MavParams::setParameterValue(const string &param_id, uint64_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint64 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT64;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_UINT64,
                           compId);
}

bool MavParams::setParameterValue(const string &param_id, int64_t param_value) {
  ParamUnion u;
  u.param_int64 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT64;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_INT64);
}

bool MavParams::setParameterValue(const string &param_id, int64_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_int64 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT64;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_INT64,
                           compId);
}

bool MavParams::setParameterValue(const string &param_id,
                                  uint32_t param_value) {
  ParamUnion u;
  u.param_uint32 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT32;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_UINT32);
}

bool MavParams::setParameterValue(const string &param_id, uint32_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint32 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT32;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_UINT32,
                           compId);
}

bool MavParams::setParameterValue(const string &param_id, int32_t param_value) {
  ParamUnion u;
  u.param_int32 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT32;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_INT32);
}

bool MavParams::setParameterValue(const string &param_id, int32_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_int32 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT32;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_INT32,
                           compId);
}

bool MavParams::setParameterValue(const string &param_id,
                                  uint16_t param_value) {
  ParamUnion u;
  u.param_uint16 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT16;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_UINT16);
}

bool MavParams::setParameterValue(const string &param_id, uint16_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint16 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT16;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_UINT16,
                           compId);
}

bool MavParams::setParameterValue(const string &param_id, int16_t param_value) {
  ParamUnion u;
  u.param_int16 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT16;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_INT16);
}

bool MavParams::setParameterValue(const string &param_id, int16_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_int16 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT16;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_INT16,
                           compId);
}

bool MavParams::setParameterValue(const string &param_id, uint8_t param_value) {
  ParamUnion u;
  u.param_uint8 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT8;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_UINT8);
}

bool MavParams::setParameterValue(const string &param_id, uint8_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint8 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT8;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_UINT8,
                           compId);
}

bool MavParams::setParameterValue(const string &param_id, int8_t param_value) {
  ParamUnion u;
  u.param_uint8 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT8;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_INT8);
}

bool MavParams::setParameterValue(const string &param_id, int8_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint8 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT8;
  string str(reinterpret_cast<char const *>(u.bytes), u.getParamSize());
  return setParameterValue(param_id, move(str), MAV_PARAM_EXT_TYPE_INT8,
                           compId);
}
