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

#include <muconfig/muconfig.h>

#include <iostream>
#include <memory>
#include <string>

using namespace std;
using namespace muconfig;

int main(int argc, char** argv) {
  const int total_entries = 7;
  int entry_cntr = 0;

  if (argc < 2) {
    cout << "Usage: " << argv[0] << " filename" << endl;
    return 1;
  }

  unique_ptr<MuConfig> cfg = MuConfig::createConfig(argv[1]);
  if (cfg == nullptr || !cfg->isOk()) {
    cerr << "Test failed: invalid config file: " << argv[1] << endl;
    return 1;
  }

  cfg->print();

  const string s_local = "muroute.telemetry.with_local_log";
  auto local = cfg->getObject(s_local, TYPE::BOOL);
  if (local) {
    std::shared_ptr<bool> v1 = local->getValueSimple<bool>();
    if (v1) {
      ++entry_cntr;
      cout << s_local << "=" << *v1 << endl;
    }
  }

  const string s_network = "muroute.telemetry.with_network";
  auto network = cfg->getObject(s_network, TYPE::BOOL);
  if (network) {
    auto v2 = network->getValueSimple<bool>();
    if (v2) {
      ++entry_cntr;
      cout << s_network << "=" << *v2 << endl;
    }
  }

  const string s_tport = "muroute.telemetry.telemetry_port";
  auto tport = cfg->getObject(s_tport, TYPE::INT);
  if (tport) {
    std::shared_ptr<int> v3 = tport->getValueSimple<int>();
    if (v3) {
      ++entry_cntr;
      cout << s_tport << "=" << *v3 << endl;
    }
  }

  const string s_threshold = "muflow.sparse_mem.detector_threshold";
  auto threshold = cfg->getObject(s_threshold, TYPE::DOUBLE);
  if (threshold) {
    auto v4 = threshold->getValueSimple<double>();
    if (v4) {
      ++entry_cntr;
      cout << s_threshold << "=" << *v4 << endl;
    }
  }

  const string s_ni = "muroute.network_interface";
  auto ni = cfg->getObject(s_ni, TYPE::STRING);
  if (ni) {
    auto v5 = ni->getValueString();
    if (v5) {
      ++entry_cntr;
      cout << s_ni << "=" << *v5 << endl;
    }
  }

  unique_ptr<MuConfig> cfg2 = MuConfig::createConfig(cfg->getName());
  if (cfg2 == nullptr || !cfg2->isOk()) {
    cerr << "Test failed: coudn't create another config from " << cfg->getName()
         << endl;
    return 1;
  }

  const string s_log_dir = "muroute.telemetry.log_directory_path";
  auto log_dir = cfg2->getObject(s_log_dir, TYPE::STRING);
  if (log_dir) {
    auto v6 = log_dir->getValueString();
    if (v6) {
      ++entry_cntr;
      cout << s_log_dir << "=" << *v6 << endl;
    }
  }

  const string s_cap = "muqueue.scheduler.queue_capacity";
  auto cap = cfg2->getObject(s_cap, TYPE::INT);
  if (cap) {
    auto v7 = cap->getValueSimple<int>();
    if (v7) {
      ++entry_cntr;
      cout << s_cap << "=" << *v7 << endl;
    }
  }

  if (entry_cntr != total_entries) {
    cerr << "Test failed: read " << entry_cntr << " instead of "
         << total_entries << endl;
    return 1;
  }

  cout << "Test passed\n";

  return 0;
}
