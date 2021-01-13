/*
 * Copyright 2019-2020, EDEL LLC <http://www.edelsys.com>
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

#ifndef FLOWPROTO_H
#define FLOWPROTO_H

#include <netinet/in.h>
#include <stdint.h>
#include <sys/un.h>
#include <termios.h>

#include <mutex>

#ifdef __cplusplus

namespace fflow {
namespace proto {

extern "C" {
#endif

typedef enum {
  FLOWPROTO_ID_PAYLOAD = ((1 << 0)),
  FLOWPROTO_ID_ROUTEINFO = ((1 << 1))
} FLOWPROTO_ID_TYPE;

typedef enum {
  FLOWPROTO_FLAG_COMPRESS = ((1 << 0)),  ///< compression
  FLOWPROTO_FLAG_SECURE = ((1 << 1))     ///< cipher in use
} FLOWPROTO_FLAGS;

typedef enum {
  FLOWADDR_NONE = 0,
  FLOWADDR_UNICAST = 1,
  FLOWADDR_BROADCAST,
  FLOWADDR_MULTICAST,
  FLOWADDR_LOOPBACK
} FLOWADDR_TYPE;

typedef enum {
  FLOWPROTO_NONE = 0x40000,  ///< undefined
  FLOWPROTO_TIME,            ///< time synchronization
  FLOWPROTO_SYNC,            ///< ordinar function flow
  FLOWPROTO_ASYNC,           ///< synchronized function flow
  FLOWPROTO_CACHE,           ///< cache specific (invalidate)
  FLOWPROTO_INFO,            ///< routing and group information
  FLOWPROTO_INTRO,           ///< function introduction
  FLOWPROTO_LOOKUP,          ///< lookup for function (data source)
  FLOWPROTO_LINK,            ///< link functions
  FLOWPROTO_UNLINK,          ///< unlink functions
  FLOWPROTO_MIGRATE,         ///< load balancing message
} FLOWPROTO_TYPE;

#pragma pack(push, 1)
typedef struct __flowhdr_t {
  // meta 1
  uint32_t payloadid;  ///< @todo this should be used in upper layers
  uint32_t hdr_len;    ///< length of header
  uint32_t data_len;   ///< length of data in packet
  uint32_t channelid;  ///< crc
  uint32_t flags;      ///< flags

  // meta 2
  uint8_t seq;              ///< sequence
  uint32_t fragno;          ///< fragment
  uint32_t mcast_num;       ///< number of multicast recipients
  uint32_t send_timestamp;  ///<

  struct {            ///<
    uint32_t sysid;   ///< source system id
    uint32_t funcid;  ///< source unique id of function in host
  } srcaddr;          ///<

  struct {            ///<
    uint32_t sysid;   ///< destination system id
    uint32_t funcid;  ///< destination function id
  } dstaddr;          ///<

} flowhdr_t;

typedef struct __hdr_t {
  uint32_t magic;        ///< TR_MAGIC 0xfafafafa
  uint32_t multicastid;  ///< id of a multicast group
  uint32_t proto;        ///< protocol number
  uint32_t crc;          ///<
  uint32_t padding;      ///< extra space for padding or future use
} hdr_t;                 ///< udp transport header

typedef struct __muflow_packet_t {
  hdr_t hdr;
  uint8_t payload[];
} muflow_packet_t __attribute_deprecated__;

#define MUFLOW_MAX_PAYLOAD_LEN 255
#define MUFLOW_NUM_CHECKSUM_BYTES 2
#define MUFLOW_SIGNATURE_BLOCK_LEN 13

// define mavlink compitable messages structure
typedef struct __muflow_packet {
  uint16_t checksum;
  uint8_t magic;
  uint8_t len;
  uint8_t incompat_flags;
  uint8_t compat_flags;
  uint8_t seq;
  uint8_t sysid;
  uint8_t compid;
  uint32_t msgid : 24;
  uint64_t
      payload64[(MUFLOW_MAX_PAYLOAD_LEN + MUFLOW_NUM_CHECKSUM_BYTES + 7) / 8];
  uint8_t ck[2];
  uint8_t signature[MUFLOW_SIGNATURE_BLOCK_LEN];
} muflow_packet_t_new_;

#pragma pack(pop)

#define TR_MAGIC 0xfcfcfcfc
#define TR_MAX_PAYLOAD_LEN 496
#define TR_MAX_PAYLOAD_INTLEN 124

typedef struct __native_addr_t {
  uint32_t typ;
  int len;

  union {
    struct sockaddr so;
    struct sockaddr_un un;
    struct sockaddr_in in;
    struct termios ser;
  } addr;

} native_addr_t;

#ifdef __cplusplus
}  // extern C
}  // namespace proto
}  // namespace fflow
#endif

#ifdef __cplusplus
#include <array>
#include <functional>
#include <list>
#include <set>
#include <thread>
#include <unordered_set>
#include <vector>

namespace fflow {
namespace proto {

struct __muflow_packet__ {
  hdr_t *hdr;
  uint8_t *payload;
  std::vector<uint8_t> msgdata;

  template <typename C>
  C &as() {
    return reinterpret_cast<C &>(*msgdata.data());
  }

  __muflow_packet__(uint32_t type = FLOWPROTO_ID_PAYLOAD) {
    msgdata.resize(sizeof(hdr_t));
    hdr = reinterpret_cast<hdr_t *>(&msgdata[0]);
    payload = &msgdata[sizeof(hdr_t)];
    hdr->magic = TR_MAGIC;
    hdr->proto = type;
    hdr->padding = 0;
    alloc(0);
  }

  void alloc(size_t siz) {
    msgdata.resize(siz + sizeof(hdr_t));
    hdr = reinterpret_cast<hdr_t *>(&msgdata[0]);
    payload = &msgdata[sizeof(hdr_t)];
  }
};

}  // namespace proto
}  // namespace fflow

namespace fflow {
using namespace proto;

class AbstractEdgeInterface {
 public:
  typedef std::function<int(uint32_t, std::vector<uint8_t> &,
                            const native_addr_t &)>
      recv_cb_func_t;  // receive callbck function type

  std::string name;
  uint32_t edge_id;
  recv_cb_func_t recv_cb;

  std::mutex feasaddrlock;
  std::unordered_set<uint64_t> feasable_addrs;  // of type SparseAddress

  AbstractEdgeInterface();
  virtual ~AbstractEdgeInterface();
  virtual void sendtoraw(std::vector<uint8_t> &msg, const native_addr_t &to,
                         const native_addr_t &from, uint32_t from_edge_id) = 0;
  virtual bool open(const std::string &iface, uint32_t _port) = 0;

  // register data proxy function
  bool registerCallback(recv_cb_func_t f) {
    recv_cb = f;
    return true;
  }
};

class LoopBackInterface : public AbstractEdgeInterface {
 public:
  LoopBackInterface();
  virtual ~LoopBackInterface();

  virtual void sendtoraw(std::vector<uint8_t> &msg, const native_addr_t &to,
                         const native_addr_t &from, uint32_t from_edge_id);

  virtual bool open(const std::string &iface, uint32_t _port);
};

}  // namespace fflow
#endif  // __cplusplus

#endif  // FLOWPROTO_H
