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

#include <muqueue/erqperiodic.h>
#include <muroute/funudp.h>
#include <muroute/subsystem.h>
#include <mutelemetry/mutelemetry.h>

#include <atomic>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <set>

#include "muroute/mavlink2/common/mavlink.h"

using namespace std;
using namespace boost::program_options;
using namespace fflow;
using namespace mutelemetry;
using namespace mutelemetry_network;
using namespace mutelemetry_tools;

void send_mavlink_message(mavlink_message_t &, bool);
void command_start_logging(bool);
void command_stop_logging();

RouteSystemPtr roster = nullptr;
set<uint16_t> sequence_keeper;
atomic<bool> is_start_ack(false);
atomic<bool> is_stop_ack(false);
uint64_t msg_cntr = 0;

message_handler_note_t mutelemetry_proto_handlers[] = {
    {MAVLINK_MSG_ID_COMMAND_LONG,
     [](uint8_t *payload, size_t /*len*/, SparseAddress /*sa*/,
        BaseComponentPtr /*cc*/) -> fflow::pointprec_t {
       mavlink_message_t *rxmsg = MAVPAYLOAD_TO_MAVMSG(payload);
       mavlink_command_long_t cmd;
       mavlink_msg_command_long_decode(rxmsg, &cmd);
       // TODO:
       assert(0);
       return 1.0;
     }},
    {MAVLINK_MSG_ID_COMMAND_ACK /* #77 */,
     [](uint8_t *payload, size_t /*len*/, SparseAddress /*sa*/,
        BaseComponentPtr /*cc*/) -> fflow::pointprec_t {
       mavlink_message_t *rxmsg = MAVPAYLOAD_TO_MAVMSG(payload);
       mavlink_command_ack_t ack;
       mavlink_msg_command_ack_decode(rxmsg, &ack);

       if (/* ack.result == MAV_RESULT_ACCEPTED && */
           ack.result_param2 ==
           static_cast<int32_t>(MutelemetryStreamer::get_port())) {
         if (ack.command == MAV_CMD_LOGGING_START) {
           cout << "Starting ULog stream from server" << endl;
           is_start_ack = true;
         } else if (ack.command == MAV_CMD_LOGGING_STOP) {
           // cout << "Loggin was stopped" << endl;
           is_start_ack = false;
           is_stop_ack = true;
         }
       }

       return 1.0;
     }},
    {MAVLINK_MSG_ID_LOGGING_DATA /* #266 */,
     [](uint8_t *payload, size_t /*len*/, SparseAddress /*sa*/,
        BaseComponentPtr /*cc*/) -> fflow::pointprec_t {
       mavlink_message_t *rxmsg = MAVPAYLOAD_TO_MAVMSG(payload);
       mavlink_logging_data_t logd;
       mavlink_msg_logging_data_decode(rxmsg, &logd);

       cout << "ULog message num " << ++msg_cntr << " received" << endl;
       const uint8_t *buffer = logd.data;
       if (!check_ulog_valid(buffer)) {
         cout << "Validity check failed for data" << endl;
         assert(0);
       }

       // TODO: send to stdout using parser user-defined handlers
       // and/or flush to disk

       return 1.0;
     }},
    {MAVLINK_MSG_ID_LOGGING_DATA_ACKED /* #267 */,
     [](uint8_t *payload, size_t /*len*/, SparseAddress sa,
        BaseComponentPtr /*cc*/) -> fflow::pointprec_t {
       mavlink_message_t *rxmsg = MAVPAYLOAD_TO_MAVMSG(payload);
       mavlink_logging_data_acked_t logda;
       mavlink_msg_logging_data_acked_decode(rxmsg, &logda);

       uint16_t sequence = logda.sequence;
       bool is_new = false;
       if (sequence_keeper.find(sequence) == sequence_keeper.end()) {
         sequence_keeper.insert(sequence);
         is_new = true;
       }

       const uint8_t *buffer = logda.data;
       if (is_new) {
         if (!check_ulog_valid(buffer)) {
           cout << "Validity check failed for definitions" << endl;
           assert(0);
         } else {
           // TODO: flush to disk
         }
       }

       mavlink_logging_ack_t logging_ack;
       logging_ack.sequence = sequence;
       logging_ack.target_system = sa.group_id;
       logging_ack.target_component = sa.instance_id;

       mavlink_message_t msg;
       mavlink_msg_logging_ack_encode(roster->getMcastId(),
                                      roster->getMcompId(), &msg, &logging_ack);
       send_mavlink_message(msg, false);

       return 1.0;
     }},
};

void send_mavlink_message(mavlink_message_t &msg, bool bcast) {
  uint8_t buffer[500];
  uint8_t *data = buffer;
  /* size_t len = */ mavlink_msg_to_send_buffer(data, &msg);

  // send to our multicast group only
  uint32_t comp_id = bcast ? 0 : roster->getMcompId();
  auto a = SparseAddress(roster->getMcastId(), comp_id, 0);
  roster->sendmavmsg(msg, {a});
}

void command_start_logging(bool bcast) {
  mavlink_message_t msg;

  int sys_id = roster->getMcastId();
  int comp_id = roster->getMcompId();

  mavlink_msg_command_long_pack(sys_id, comp_id, &msg, sys_id, 0,
                                MAV_CMD_LOGGING_START, 0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0);

  send_mavlink_message(msg, bcast);
}

// FIXME:
void connect() {
  command_start_logging(true);
  sleep(1);
  while (!is_start_ack) {
    command_start_logging(true);
    sleep(1);
  }
}

void command_stop_logging() {
  mavlink_message_t msg;

  int sys_id = roster->getMcastId();
  int comp_id = roster->getMcompId();

  mavlink_msg_command_long_pack(sys_id, comp_id, &msg, sys_id, 0,
                                MAV_CMD_LOGGING_STOP, 0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0);

  send_mavlink_message(msg, false);
}

void setup_roster(shared_ptr<AbstractEdgeInterface> udptr) {
  roster = RouteSystem::createRouteSys();
  roster->setMcastId(1);  // ?
  roster->add_edge_transport(udptr);
  roster->add_protocol2(mutelemetry_proto_handlers,
                        sizeof(mutelemetry_proto_handlers) /
                            sizeof(mutelemetry_proto_handlers[0]));
}

void test_stop_start() {
  post_function<void>([&](void) -> void {
    while (1) {
      while (!is_stop_ack) {
        command_stop_logging();
        this_thread::sleep_for(chrono::milliseconds(50));
      }
      is_stop_ack = false;
      // cout << "Restartin log stream!" << endl;
      while (!is_start_ack) {
        command_start_logging(false);
        this_thread::sleep_for(chrono::milliseconds(50));
      }
    }
  });
}

int main(int argc, char **argv) {
  options_description options("Mutelemetry client allowed options");
  options.add_options()("help", "Print help")
      //
      ("edge-transport", value<std::string>()->default_value("EdgeUdp:lo:7788"),
       "Transport type (default EdgeUdp)");

  variables_map varmap;
  store(parse_command_line(argc, argv, options, command_line_style::unix_style),
        varmap);
  notify(varmap);

  if (varmap.count("help") > 0) {
    cout << options << "\n";
    return 1;
  }

  const string trstr = varmap["edge-transport"].as<string>();

  std::vector<std::string> vstr;
  boost::split(vstr, trstr, boost::is_any_of(":"));

  std::shared_ptr<fflow::AbstractEdgeInterface> trptr =
      fflow::createEdgeFunctionByName(vstr[0]);
  uint32_t portn = std::stoi(vstr[2]);
  std::string iface = vstr[1];

  std::cerr << " Adding transport type:" << vstr[0] << " at " << iface
            << " iparm: " << portn;

  if (!trptr->open(iface, portn)) {
    std::cerr << " ERROR! (see log files for more info)" << std::endl;
    return 1;
  } else {
    std::cerr << " OK!" << std::endl;
  }

  setup_roster(trptr);
  connect();
  // test_stop_start();
  pause();

  return 0;
}
