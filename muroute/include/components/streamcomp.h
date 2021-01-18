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

#include <muroute/componentbus.h>
#include <muroute/mavlink2/common/mavlink.h>
#include <muroute/mavparamproto.h>

#include <iostream>
#include <map>

// clang-format off
enum class InputType : uint8_t {
  RS2_D435I     = 1 << 0,
  RS2_T265      = 1 << 1,
  V4L2          = 1 << 2,
  SIM           = 1 << 3,
  RSRVD1        = 1 << 4,
  RSRVD2        = 1 << 5,
  RSRVD3        = 1 << 6,
  RSRVD4        = 1 << 7,
  RS            = (RS2_D435I|RS2_T265),
  UNDEF         = 0
};
// clang-format on

/** -----------------------------------
 *
 * @brief The CameraInfo struct
 * Field definitions are from mavlink camera protocol.
 * see /mavlink2/common/mavlink_msg_camera_information.h:217
 *
 */
struct MediaCapsInfo {
  fflow::time_dbl_t
      time_boot_ms;           // [ms] Timestamp (milliseconds since system boot)
  std::string vendor_name;    // Name of the camera vendor
  std::string model_name;     // Name of the camera model
  uint32_t firmware_version;  // Version of the camera firmware (v << 24 & 0xff
                              // = Dev, v << 16 & 0xff = Patch, v << 8 & 0xff =
                              // Minor, v & 0xff = Major)
  float focal_length;         //[mm] Focal length in mm
  float sensor_size_h;        //[mm] Image sensor size horizontal in mm
  float sensor_size_v;        //[mm] Image sensor size vertical in mm
  uint16_t resolution_h;      //[pix] Image resolution in pixels horizontal
  uint16_t resolution_v;      //[pix] Image resolution in pixels vertical
  int lens_id;                // Reserved for a lens ID
  uint32_t flags;  // CAMERA_CAP_FLAGS enum flags (bitmap) describing camera
                   // capabilities.
  uint16_t cam_definition_version;  // Camera definition version (iteration)
  std::string cam_definition_uri;   // Camera definition URI (if any, otherwise
                                    // only basic functions will be available).
};

/** -----------------------------------
 *
 * @brief The MediaInfo struct
 *
 * @todo Make this universal between video, audio or other media.
 * @todo Use k/v like structure
 *
 */
struct MediaInfo {
 public:
  int status;
  int width;
  int height;
  InputType type;
  float bitrate;
  float fps;
  std::string name;
  std::string uri;
};

/** ------------------------------------
 *
 * @brief The MediaComponent class
 *
 */
class MediaComponent : public fflow::BaseComponent {
 public:
  MediaComponent() = default;
  MediaComponent(uint8_t id) {
    if (id >= MAV_COMP_ID_CAMERA && id <= MAV_COMP_ID_CAMERA6) setId(id);
  }
  virtual ~MediaComponent() { /* stop(); */
  }

 public:
  const MediaInfo &getInfo() {
    // FIXME: must be an array for several streams
    return minfo_;
  }

  const MediaCapsInfo &getCapsInfo() { return mcinfo_; }

  virtual bool onStartStream(const fflow::SparseAddress &from) = 0;
  virtual bool onStopStream(const fflow::SparseAddress &from) = 0;

 public:
  bool getRgbEnabled() const { return rgbEnabled_; }
  void setRgbEnabled(bool rgbEnabled) { rgbEnabled_ = rgbEnabled; }

  bool startImpl() override {
    std::cout << "MediaComponent starting => id=" << static_cast<int>(getId())
              << std::endl;
    return true;
  }

  void stopImpl() override {
    std::cout << "MediaComponent stoppping => id=" << static_cast<int>(getId())
              << std::endl;
  }

 protected:
  MediaInfo minfo_;
  MediaCapsInfo mcinfo_;

 private:
  bool rgbEnabled_ = false;
};

typedef MediaComponent *MediaComponentPtr;

/** ------------------------------------
 *
 * @brief The MediaComponentBus class
 *
 *
 */
class MediaComponentBus : public fflow::BaseMavlinkProtocol {
 public:
  MediaComponentBus() : timeout_handler_(0) {}
  ~MediaComponentBus() {}

 public:
  bool init(fflow::RouteSystemPtr);
  bool addMediaComponent(MediaComponentPtr);
  void removeMediaComponent(int);
  MediaComponentPtr getMediaComponent(int);

  const fflow::message_handler_note_t *get_table() const {
    return &proto_table_[0];
  }

  size_t get_table_len() const { return proto_table_len; }

  bool start();
  void stop();

 private:
  static constexpr size_t proto_table_len = 1;

 private:
  unsigned int timeout_handler_;
  std::map<int, fflow::AsyncERQPtr> idToErqHandle_;

  fflow::message_handler_note_t proto_table_[proto_table_len] = {
      {MAVLINK_MSG_ID_COMMAND_LONG,
       [this](uint8_t *a, size_t b, fflow::SparseAddress c,
              fflow::BaseComponentPtr /*cc*/) -> fflow::pointprec_t {
         return command_long_handler(a, b, c);
       }}};

 private:
  fflow::pointprec_t command_long_handler(uint8_t *, size_t,
                                          fflow::SparseAddress);
  bool handle_request_camera_info(const mavlink_command_long_t &,
                                  const fflow::SparseAddress &);
  bool handle_request_video_stream_info(const mavlink_command_long_t &,
                                        const fflow::SparseAddress &);
  bool handle_video_start_streaming(const mavlink_command_long_t &,
                                    const fflow::SparseAddress &);
  bool handle_video_stop_streaming(const mavlink_command_long_t &,
                                   const fflow::SparseAddress &);
};
