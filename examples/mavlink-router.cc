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

#include <muflow/sparsememory.h>
#include <muqueue/erqperiodic.h>
#include <muqueue/scheduler.h>
#include <muroute/funserial.h>
#include <muroute/subsystem.h>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

int main(int argc, char **argv) {
  namespace po = boost::program_options;

  // Declare the supported options.
  po::options_description options("Allowed options");
  po::options_description general_options("General");

  general_options.add_options()("help", "Print help")(
      "verbose", po::value<bool>()->default_value(false),
      "Verbose output, default off");

  po::options_description config_options("Config");

  config_options.add_options()
      //
      ("edge-transport", po::value<std::vector<std::string>>()->multitoken(),
       "Add edge transport module in format <transport:interface:port>")
      //
      ("mcast-id", po::value<unsigned>()->default_value(4),
       "Multicast ID, equivalent of mavlink system id");

  options.add(general_options);
  options.add(config_options);

  po::variables_map varmap;
  po::store(po::parse_command_line(argc, argv, options,
                                   po::command_line_style::unix_style),
            varmap);
  po::notify(varmap);

  if (varmap.count("help") > 0) {
    std::cout << options << "\n";
    exit(0);
  }

  // create routing and transport subsystem
  fflow::RouteSystemPtr rostersys = fflow::RouteSystem::createRouteSys();
  if (rostersys == nullptr) {
    std::cerr << "Failed to create Route";
    exit(1);
  }

  // set primary system id
  rostersys->setMcastId(varmap["mcast-id"].as<unsigned>());

  // enable forwarding in roster
  rostersys->setForwarding(true);

  // setup transports
  if (varmap.count("edge-transport") > 0) {
    auto trlist = varmap["edge-transport"].as<std::vector<std::string>>();

    for (const auto &trstr : trlist) {
      std::vector<std::string> vstr;
      boost::split(vstr, trstr, boost::is_any_of(":"));

      std::shared_ptr<fflow::AbstractEdgeInterface> trptr =
          fflow::createEdgeFunctionByName(vstr[0]);

      std::string iface = vstr[1];
      uint32_t portn = std::stoi(vstr[2]);

      if (trptr->open(iface, portn)) {
        std::cout << "Transport type:" << vstr[0] << " at " << iface << ":"
                  << portn << std::endl;
        rostersys->add_edge_transport(trptr);
      } else {
        assert(0);
      }
    }
  }

  // wait
  pause();
}
