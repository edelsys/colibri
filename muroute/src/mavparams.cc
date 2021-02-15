#include "muroute/mavparams.h"

#include <algorithm>
#include <cstring>
#include <sstream>
#include <string>

#include <glog/logging.h>

using namespace std;
using namespace fflow;

map<string, uint8_t> MavParams::paramIdCompId;
mutex MavParams::prot;

void MavParams::toParamUnion(const string &in_value, int in_type,
                             MavParams::ParamUnion &out_value) {
  memcpy(out_value.bytes, in_value.data(), in_value.size());
  out_value.type = in_type;
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
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str);
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
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_REAL64);
}

bool MavParams::setParameterValue(const string &param_id, double param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_double = param_value;
  u.type = MAV_PARAM_EXT_TYPE_REAL64;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_REAL64, compId);
}

bool MavParams::setParameterValue(const string &param_id, float param_value) {
  ParamUnion u;
  u.param_float = param_value;
  u.type = MAV_PARAM_EXT_TYPE_REAL32;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_REAL32);
}

bool MavParams::setParameterValue(const string &param_id, float param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_float = param_value;
  u.type = MAV_PARAM_EXT_TYPE_REAL32;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_REAL32, compId);
}

bool MavParams::setParameterValue(const string &param_id,
                                  uint64_t param_value) {
  ParamUnion u;
  u.param_uint64 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT64;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_UINT64);
}

bool MavParams::setParameterValue(const string &param_id, uint64_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint64 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT64;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_UINT64, compId);
}

bool MavParams::setParameterValue(const string &param_id, int64_t param_value) {
  ParamUnion u;
  u.param_int64 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT64;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_INT64);
}

bool MavParams::setParameterValue(const string &param_id, int64_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_int64 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT64;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_INT64, compId);
}

bool MavParams::setParameterValue(const string &param_id,
                                  uint32_t param_value) {
  ParamUnion u;
  u.param_uint32 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT32;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_UINT32);
}

bool MavParams::setParameterValue(const string &param_id, uint32_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint32 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT32;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_UINT32, compId);
}

bool MavParams::setParameterValue(const string &param_id, int32_t param_value) {
  ParamUnion u;
  u.param_int32 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT32;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_INT32);
}

bool MavParams::setParameterValue(const string &param_id, int32_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_int32 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT32;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_INT32, compId);
}

bool MavParams::setParameterValue(const string &param_id,
                                  uint16_t param_value) {
  ParamUnion u;
  u.param_uint16 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT16;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_UINT16);
}

bool MavParams::setParameterValue(const string &param_id, uint16_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint16 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT16;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_UINT16, compId);
}

bool MavParams::setParameterValue(const string &param_id, int16_t param_value) {
  ParamUnion u;
  u.param_int16 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT16;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_INT16);
}

bool MavParams::setParameterValue(const string &param_id, int16_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_int16 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT16;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_INT16, compId);
}

bool MavParams::setParameterValue(const string &param_id, uint8_t param_value) {
  ParamUnion u;
  u.param_uint8 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT8;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_UINT8);
}

bool MavParams::setParameterValue(const string &param_id, uint8_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint8 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_UINT8;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_UINT8, compId);
}

bool MavParams::setParameterValue(const string &param_id, int8_t param_value) {
  ParamUnion u;
  u.param_uint8 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT8;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_INT8);
}

bool MavParams::setParameterValue(const string &param_id, int8_t param_value,
                                  uint8_t compId) {
  ParamUnion u;
  u.param_uint8 = param_value;
  u.type = MAV_PARAM_EXT_TYPE_INT8;
  string str(reinterpret_cast<char const *>(u.bytes), max_param_value_len);
  return setParameterValue(param_id, str, MAV_PARAM_EXT_TYPE_INT8, compId);
}
