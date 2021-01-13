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

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PROPERTIES_TEST

#include <muflow/sparsememory.h>
#include <muqueue/asyncsubsys.h>
#include <muqueue/erqio.h>
#include <muqueue/erqperiodic.h>
#include <muroute/funudp.h>
#include <muroute/mavlink2/standard/mavlink.h>
#include <muroute/subsystem.h>

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/version.hpp>
#include <chrono>
#include <iomanip>

using namespace boost;
using namespace fflow;
using namespace fflow::proto;

/*
 To add a new protocol to a
 */
static message_handler_note_t proto_table[] = {
    {fflow::proto::FLOWPROTO_ASYNC,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {fflow::proto::FLOWPROTO_SYNC,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {fflow::proto::FLOWPROTO_TIME,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {fflow::proto::FLOWPROTO_INFO,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {fflow::proto::FLOWPROTO_LOOKUP,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {fflow::proto::FLOWPROTO_INTRO,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {fflow::proto::FLOWPROTO_LINK,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {fflow::proto::FLOWPROTO_UNLINK,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {fflow::proto::FLOWPROTO_MIGRATE,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},
};

static message_handler_note_t proto_table_2[] = {
    {MAVLINK_MSG_ID_LOCAL_POSITION_NED,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {MAVLINK_MSG_ID_HEARTBEAT,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

    {MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
     [](uint8_t *, size_t, fflow::SparseAddress,
        BaseComponentPtr) -> fflow::pointprec_t {
       std::cerr << __FILE__ << ": " << __LINE__ << " " << __PRETTY_FUNCTION__
                 << std::endl;
       return 1.0;
     }},

};

int main(int argc, char **argv) {
  auto hyperMem = SparseMemory::newSparseMemory();
  RouteSystemPtr rostersys = RouteSystem::createRouteSys();

  namespace po = boost::program_options;

  // Declare the supported options.
  po::options_description options("Allowed options");

  options.add_options()("help", "Print help")(
      "verbose", po::bool_switch()->default_value(false),
      "Verbose output, default off")
      //
      ("port", po::value<uint32_t>()->default_value(4567),
       "Udp port number to listen to")
      //
      ("iface", po::value<std::string>()->default_value("lo0"),
       "Interface name")
      //
      ("edge-transport", po::value<std::string>()->default_value("EdgeUdp"),
       "Edge transport module to use")
      //
      ("gendata", po::value<bool>()->default_value(false),
       "Generate payload data...");

  po::variables_map varmap;
  po::store(po::parse_command_line(argc, argv, options,
                                   po::command_line_style::unix_style),
            varmap);
  po::notify(varmap);

  if (varmap.count("help") > 0) {
    std::cout << options << "\n";
    return 1;
  }

  uint32_t portn = varmap["port"].as<uint32_t>();
  std::cerr << "Using port number " << portn << "\n";

  std::string trnsptrstr = varmap["edge-transport"].as<std::string>();
  std::shared_ptr<fflow::AbstractEdgeInterface> trptr =
      fflow::createEdgeFunctionByName(trnsptrstr);

  std::string iface = varmap["iface"].as<std::string>();
  trptr->open(iface, portn);

  rostersys->setMcastId(1);
  rostersys->add_edge_transport(trptr);

  rostersys->add_protocol2(
      proto_table, sizeof(proto_table) / sizeof(message_handler_note_t));

  rostersys->add_protocol2(
      proto_table_2, sizeof(proto_table_2) / sizeof(message_handler_note_t));

  bool gendata = varmap["gendata"].as<bool>();

  if (gendata) {
    std::string str = "Hello world!";
    std::vector<u_int8_t> databuf(str.begin(), str.end());

    mavlink_message_t msg;
    RouteSystem::muproto_message_pack(FLOWPROTO_INTRO, databuf, msg);

    // setup route system clock
    add_periodic<void>(
        ([msg, &rostersys](void) mutable -> void {
          mavlink_command_long_t cmd = {};

          cmd.command = MAV_CMD_NAV_TAKEOFF;
          cmd.target_system = rostersys->getMcastId();
          cmd.target_component = MAV_COMP_ID_ALL;
          cmd.param1 = 4.3;

          mavlink_msg_command_long_encode(cmd.target_system,
                                          rostersys->getMcompId(), &msg, &cmd);

          rostersys->sendmsg(
              reinterpret_cast<uint8_t *>(&cmd), msg.len, msg.msgid,
              {SparseAddress(cmd.target_system,
                             (static_cast<uint32_t>(cmd.target_component)),
                             0)});
        }),
        0.0, 0.2);
  }

  pause();

  return 0;
}
