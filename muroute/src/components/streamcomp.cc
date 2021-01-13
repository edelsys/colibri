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

#include "components/streamcomp.h"

#include <assert.h>

using namespace std;
using namespace fflow;

bool MediaComponentBus::init(RouteSystemPtr roster) {
  if (roster == nullptr) return false;
  setRoster(roster);
  getRoster()->add_protocol2(proto_table_common, proto_table_common_len);
  getRoster()->add_protocol2(get_table(), get_table_len());
  return true;
}

bool MediaComponentBus::start() {
  RouteSystemPtr roster = getRoster();
  if (!roster) return false;
  if (roster->getCBus().size() == 0) {
    LOG(ERROR) << "Unable to start MediaComponentBus - no cameras added";
    return false;
  }
  return true;
}

void MediaComponentBus::stop() {
  int compid = MAV_COMP_ID_CAMERA;

  while (compid <= MAV_COMP_ID_CAMERA6) {
    MediaComponentPtr cam_iface = getMediaComponent(compid);
    if (cam_iface && cam_iface->getRgbEnabled())
      cam_iface->setRgbEnabled(false);
    const auto &it = idToErqHandle_.find(compid);
    if (it != idToErqHandle_.end()) {
      AsyncERQPtr erq_handle = move(it->second);
      AsyncController *async_controller = nullptr;
      erq_handle->removeFrom(async_controller);
      idToErqHandle_.erase(it);
    }
    compid++;
  }
}

bool MediaComponentBus::handle_request_camera_info(
    const mavlink_command_long_t &cmd, const fflow::SparseAddress &from) {
  mavlink_message_t msg;
  bool result = false;

  if (std::abs(cmd.param1) <= epsilon) return true;

  MediaComponentPtr cam_iface = getMediaComponent(cmd.target_component);
  if (cam_iface) {
    const MediaCapsInfo &mcinfo = cam_iface->getCapsInfo();
    mavlink_msg_camera_information_pack(
        getRoster()->getMcastId(), cmd.target_component, &msg,
        uint32_t(1000 * mcinfo.time_boot_ms),
        reinterpret_cast<const uint8_t *>(mcinfo.vendor_name.c_str()),
        reinterpret_cast<const uint8_t *>(mcinfo.model_name.c_str()),
        mcinfo.firmware_version, mcinfo.focal_length, mcinfo.sensor_size_h,
        mcinfo.sensor_size_v, mcinfo.resolution_h, mcinfo.resolution_v,
        uint8_t(mcinfo.lens_id), mcinfo.flags, mcinfo.cam_definition_version,
        mcinfo.cam_definition_uri.c_str());

    // this_thread::sleep_for(chrono::milliseconds(200));
    send_mavlink_message(msg, cmd.target_component, from.instance_id);
    result = true;
  }

  return result;
}

bool MediaComponentBus::handle_request_video_stream_info(
    const mavlink_command_long_t &cmd, const fflow::SparseAddress &from) {
  mavlink_message_t msg;
  bool result = false;

  MediaComponent *cam_iface = getMediaComponent(cmd.target_component);
  if (cam_iface) {
    uint8_t stream_num = static_cast<uint8_t>(std::abs(cmd.param1));
    const MediaInfo &minfo = cam_iface->getInfo();
    if (stream_num > 0) {
      mavlink_msg_video_stream_information_pack(
          getRoster()->getMcastId(), cmd.target_component, &msg,
          cmd.target_component, 5, VIDEO_STREAM_TYPE_MPEG_TS_H264, minfo.status,
          minfo.fps, minfo.width, minfo.height, minfo.bitrate, 0, 90,
          "Stream name", minfo.uri.c_str());

      // this_thread::sleep_for(chrono::milliseconds(200));
      send_mavlink_message(msg, cmd.target_component, from.instance_id);
    } else {
      // TODO:
      assert(0);
    }
    result = true;
  }

  return result;
}

bool MediaComponentBus::handle_video_start_streaming(
    const mavlink_command_long_t &cmd, const SparseAddress &from) {
  bool result = false;
  int compid = cmd.target_component;
  MediaComponent *cam_iface = getMediaComponent(compid);

  if (cam_iface /*&& !cam_iface->getRgbEnabled()*/)
    result = cam_iface->onStartStream(from);

  return result;
}

bool MediaComponentBus::handle_video_stop_streaming(
    const mavlink_command_long_t &cmd, const SparseAddress &from) {
  bool result = false;
  int compid = int(cmd.target_component);
  MediaComponent *cam_iface = getMediaComponent(compid);

  if (cam_iface /*&& cam_iface->getRgbEnabled()*/)
    result = cam_iface->onStopStream(from);

  return result;
}

pointprec_t MediaComponentBus::command_long_handler(uint8_t *payload,
                                                    size_t /*len*/,
                                                    SparseAddress from) {
  mavlink_message_t *msg = MAVPAYLOAD_TO_MAVMSG(payload);
  mavlink_command_long_t cmd;

  mavlink_msg_command_long_decode(msg, &cmd);

  uint8_t comp_id = cmd.target_component;
  bool confirm = cmd.confirmation;
  bool result = false;

  RouteSystemPtr roster = getRoster();
  if (!roster) {
    LOG(ERROR) << "MediaComponentBus is not initialized!";
    return 1.0;
  }

  LOG(INFO) << "Command received: (sysid:" << cmd.target_system
            << " compid:" << comp_id << " msgid:" << cmd.command << ")";

  if (!roster->getCBus().has_component(comp_id)) {
    LOG(INFO) << "Camera with ID=" << comp_id << " not present";
  } else {
    switch (cmd.command) {
      case MAV_CMD_REQUEST_CAMERA_INFORMATION:
        result = handle_request_camera_info(cmd, from);
        break;
        //    case MAV_CMD_REQUEST_MESSAGE:
      case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
        result = handle_request_video_stream_info(cmd, from);
        break;
      case MAV_CMD_VIDEO_START_STREAMING:
        result = handle_video_start_streaming(cmd, from);
        break;
      case MAV_CMD_VIDEO_STOP_STREAMING:
        result = handle_video_stop_streaming(cmd, from);
        break;
      // NI
      case MAV_CMD_REQUEST_CAMERA_SETTINGS:
      case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
      case MAV_CMD_RESET_CAMERA_SETTINGS:
      case MAV_CMD_REQUEST_STORAGE_INFORMATION:
      case MAV_CMD_STORAGE_FORMAT:
      case MAV_CMD_SET_CAMERA_MODE:
      case MAV_CMD_IMAGE_START_CAPTURE:
      case MAV_CMD_IMAGE_STOP_CAPTURE:
      case MAV_CMD_VIDEO_START_CAPTURE:
      case MAV_CMD_VIDEO_STOP_CAPTURE:
      case MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
      case MAV_CMD_DO_TRIGGER_CONTROL:
        LOG(INFO) << "Camera command" << cmd.command << " unsupported";
        break;
      default:
        LOG(INFO) << "Command " << cmd.command << " unhandled";
        confirm = false;  // cancel confirmation
        break;
    }
  }

  if (confirm) {
    mavlink_message_t msg;

    mavlink_msg_command_ack_pack(
        uint8_t(roster->getMcastId()) /*system_id*/,
        uint8_t(comp_id) /* our own component_id*/, &msg /*msg*/,
        cmd.command /*command*/,
        result ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED, 0 /*progress*/,
        0 /*result_param2*/, uint8_t(roster->getMcastId()) /*target_system*/,
        from.instance_id /* target_component */);

    send_mavlink_message(msg, comp_id, from.instance_id);
  }

  return 1.0;
}

pointprec_t MediaComponentBus::param_request_read_handler(
    uint8_t *payload, size_t /*len*/, SparseAddress /*sa*/,
    fflow::BaseComponentPtr /*comp*/) {
  mavlink_message_t *msg = MAVPAYLOAD_TO_MAVMSG(payload);
  mavlink_param_request_read_t pread;

  mavlink_msg_param_request_read_decode(msg, &pread);

  uint8_t comp_id = pread.target_component;
  uint8_t sys_id = pread.target_system;

  LOG(INFO) << "MediaComponentBus::request_read_handler() ===> sysid:"
            << static_cast<int>(sys_id)
            << " compid:" << static_cast<int>(comp_id) << " param:\""
            << pread.param_id << "\"";

  // TODO:

  return 1.0;
}

pointprec_t MediaComponentBus::param_request_list_handler(
    uint8_t *payload, size_t /*len*/, SparseAddress /*sa*/,
    fflow::BaseComponentPtr /*comp*/) {
  mavlink_message_t *msg = MAVPAYLOAD_TO_MAVMSG(payload);
  mavlink_param_request_list_t plist;

  mavlink_msg_param_request_list_decode(msg, &plist);

  uint8_t comp_id = plist.target_component;
  uint8_t sys_id = plist.target_system;

  LOG(INFO) << "MediaComponentBus::request_list_handler() ===> sysid:"
            << static_cast<int>(sys_id)
            << " compid:" << static_cast<int>(comp_id);

  // TODO:

  return 1.0;
}

pointprec_t MediaComponentBus::param_set_handler(
    uint8_t *payload, size_t /*len*/, SparseAddress /*sa*/,
    fflow::BaseComponentPtr /*comp*/) {
  mavlink_message_t *msg = MAVPAYLOAD_TO_MAVMSG(payload);

  mavlink_param_set_t pset;
  mavlink_msg_param_set_decode(msg, &pset);

  uint8_t comp_id = pset.target_component;
  uint8_t sys_id = pset.target_system;

  LOG(INFO) << "MediaComponentBus::request_list_handler() ===> sysid:"
            << static_cast<int>(sys_id)
            << " compid:" << static_cast<int>(comp_id);

  // TODO:

  return 1.0;
}

bool MediaComponentBus::addMediaComponent(MediaComponentPtr cam_comp) {
  int compid = MAV_COMP_ID_CAMERA;
  RouteSystemPtr roster = getRoster();
  bool ret = false;

  if (roster) {
    while (compid <= MAV_COMP_ID_CAMERA6) {
      if (!roster->getCBus().has_component(compid)) {
        cam_comp->setId(static_cast<uint8_t>(compid));
        cam_comp->setRoster(getRoster());
        ret = roster->getCBus().add_component(cam_comp);
        break;
      }
      compid++;
    }
  }

  return ret;
}

void MediaComponentBus::removeMediaComponent(int comp_id) {
  RouteSystemPtr roster = getRoster();
  if (roster) roster->getCBus().remove_component(comp_id);
}

MediaComponentPtr MediaComponentBus::getMediaComponent(int comp_id) {
  if (comp_id < MAV_COMP_ID_CAMERA || comp_id > MAV_COMP_ID_CAMERA6 ||
      !getRoster())
    return nullptr;
  return dynamic_cast<MediaComponentPtr>(
      getRoster()->getCBus().get_component(comp_id));
}
