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
#include <muroute/concqueue.h>
#include <muroute/mavlink2/common/mavlink.h>
#include <muroute/mavparamproto.h>

#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

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
 * @brief The StreamInfo struct
 *
 */
struct StreamInfo {
  uint8_t type_ = VIDEO_STREAM_TYPE_MPEG_TS_H264;
  uint16_t flags_ = VIDEO_STREAM_STATUS_FLAGS_RUNNING;
  float framerate_ = 30.0;
  uint16_t resolution_h_ = 424;
  uint16_t resolution_w_ = 240;
  uint32_t bitrate_ = 424 * 240 * 4 * 8 * 30;
  uint16_t rotation_ = 0;
  uint16_t hfov_ = 90;
  std::string name_ = "Unknown";
  std::string uri_;
};

class Stream;
using StreamPtr = std::shared_ptr<Stream>;

/** -----------------------------------
 *
 * @brief The Stream class
 *
 */
class Stream {
 public:
  Stream() : erq_handle_(nullptr), running_(false) {}
  Stream(const StreamInfo &info)
      : erq_handle_(nullptr), running_(false), info_(info) {}
  virtual ~Stream() { stop(); }

  bool is_running() const { return running_; }
  const ConcRingBuffer<std::shared_ptr<cv::Mat>> &getRgbBuffer() const {
    return rgb_buff_;
  }
  ConcRingBuffer<std::shared_ptr<cv::Mat>> &getRgbBuffer() { return rgb_buff_; }

  uint8_t getStreamType() const { return info_.type_; }
  void setStreamType(uint8_t type) { info_.type_ = type; }
  uint16_t getStreamFlags() const { return info_.flags_; }
  void setStreamFlags(uint16_t flags) { info_.flags_ = flags; }
  float getStreamFramerate() const { return info_.framerate_; }
  void setStreamFramerate(float framerate) { info_.framerate_ = framerate; }
  uint16_t getStreamHeight() const { return info_.resolution_h_; }
  void setStreamHeight(uint16_t h) { info_.resolution_h_ = h; }
  uint16_t getStreamWidth() const { return info_.resolution_w_; }
  void setStreamWidth(uint16_t w) { info_.resolution_w_ = w; }
  uint32_t getStreamBitrate() const { return info_.bitrate_; }
  void setStreamBitrate(uint32_t bitrate) { info_.bitrate_ = bitrate; }
  uint16_t getStreamRotation() const { return info_.rotation_; }
  void setStreamRotattion(uint16_t rotation) { info_.rotation_ = rotation; }
  uint16_t getStreamHFov() const { return info_.hfov_; }
  void setStreamHFov(uint16_t) { info_.hfov_; }
  const std::string &getStreamName() const { return info_.name_; }
  void setStreamName(const std::string &name) { info_.name_ = name; }
  const std::string &getStreamURI() const { return info_.uri_; }
  void setStreamURI(const std::string &uri) { info_.uri_ = uri; }

  bool start();
  void stop();

 private:
  fflow::AsyncERQPtr erq_handle_;
  bool running_;
  StreamInfo info_;
  ConcRingBuffer<std::shared_ptr<cv::Mat>> rgb_buff_;

 public:
  static StreamPtr createStream() { return std::make_shared<Stream>(); }

  static StreamPtr createStream(const StreamInfo &info) {
    return std::make_shared<Stream>(info);
  }
};

using StreamPtr = std::shared_ptr<Stream>;

/** ------------------------------------
 *
 * @brief The MediaComponent class
 *
 */
class MediaComponent : public fflow::BaseComponent {
 public:
  MediaComponent() { streams_.clear(); }
  MediaComponent(uint8_t id) {
    streams_.clear();
    if (id >= MAV_COMP_ID_CAMERA && id <= MAV_COMP_ID_CAMERA6) setId(id);
  }

  virtual ~MediaComponent() { /* stop(); */
  }

 public:
  size_t getNumberOfStreams() const { return streams_.size(); }

  const MediaCapsInfo &getCapsInfo() const { return cinfo_; }
  void setCapsInfo(const MediaCapsInfo &cinfo) { cinfo_ = cinfo; }

  bool getRgbEnabled() const { return rgbEnabled_; }
  void setRgbEnabled(bool rgbEnabled) { rgbEnabled_ = rgbEnabled; }

  const StreamPtr getStream(size_t);
  int registerStream(const StreamInfo &);
  int registerStream(const StreamPtr);
  bool startStream(uint8_t);
  bool stopStream(uint8_t);

 protected:
  // may be used for starting, for example, a preprocessing thread
  virtual bool onStartStream(const StreamPtr &) { return true; }
  virtual bool onStopStream(const StreamPtr &) { return true; }

 private:
  bool rgbEnabled_ = false;
  std::vector<StreamPtr> streams_;
  MediaCapsInfo cinfo_;
};

typedef MediaComponent *MediaComponentPtr;

/** ------------------------------------
 *
 * @brief The VideoServer class
 *
 *
 */
class VideoServer : public fflow::BaseMavlinkProtocol {
 public:
  VideoServer() : n_inuse_(0), timeout_handler_(0) {}
  virtual ~VideoServer();

 public:
  bool init(fflow::RouteSystemPtr);
  bool addMediaComponent(MediaComponentPtr);
  void removeMediaComponent(int);
  void removeMediaComponent(MediaComponentPtr);
  MediaComponentPtr getMediaComponent(int);

  const fflow::message_handler_note_t *get_table() const override {
    return &proto_table_[0];
  }

  size_t get_table_len() const override { return proto_table_len; }

  bool start();
  void stop();

 private:
  static constexpr size_t proto_table_len = 1;

 private:
  uint8_t n_inuse_;
  unsigned int timeout_handler_;

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
