![C/C++](https://github.com/edelsys/colibri/workflows/C/C++/badge.svg)

## Introduction
The library provides an API for companion systems to communicate with MAVLink-based robotics systems.
It can be used for example within obstacle avoidance, and route planning algorithms.

## Code structure
The code tree is organized as a set of modules.
They all are prefixed with a 'mu' word for a historical reason and it doesn't reflect anything for the moment.

### muconfig
This is a singleton component which mainly used to work with 'toml'-formatted configuration files.
It is designed to be completely independent from other components and can be used in any c++ project independently.

### muqueue
This implements an API to work and design event-based and async control flows.
At the backend currently, it is  based on the 'libev' framework and provides high-level and convenient wrappers 
to 'libev' structures along with threading and queueing capabilities.
Mainly the routing and transport subsystems depend on it. It is recommended to use 'muqueue' framework for control 
flow even for your own async functions for better control over load-balancing, getting runtime 
statistics and priorities.

### muflow
This is kept for future purposes as some concepts of graph-like memory should be implemented here.
For the moment there are just some data structures used in other components.

### muroute
This implements a mavlink protocol and provides a great capability for applications to be smoothly integrated into state-of-the-art robotic systems using mavlink2. It utilizes the concept of a programmatic component bus and has great scalability. Components on the bus are transparently mapped into mavlink components. It is designed as a plugin-based system, implements message routing, and potentially can be used with non-mavlink protocols. There are modules for UDP, TCP, and serial transport already available. For specific applications, it can be extended with custom transport modules or protocols.

### mutelemetry
See mutelemetry's [README.md](mutelemetry/README.md) for details.

## Getting Started
In the root library folder execute:

```bash
#!bash
# git clone --recurse-submodules <repo>
mkdir build
cd build
# if you want to build packages type
cmake .. -DCPACK_BINARY_DEB=ON \
  -DCPACK_DEBIAN_PACKAGE_ARCHITECTURE=amd64 \
  -DCMAKE_BUILD_TYPE=Release
## or just for defaults
#cmake ..

# Build package
cmake --build build  --config Release --target package
```

Prerequisites
* [Boost](http://www.boost.org/users/download/) (Ubuntu: sudo apt-get install libboost-all-dev)
* [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.0 (Ubuntu: sudo apt-get install cmake)
* [libev](libev.schmorp.de) (Ubuntu: sudo apt-get install libev-dev)
* clang-format (Ubuntu: sudo apt-get install libev-dev)

See the [INSTALL](INSTALL.md) file for more detailed installation instructions.
Please see the [examples](examples/) directory and the USAGE file for examples on how to use.

## Features
* Component subsytem
* Mavlink2 routing and messaging
* Function workqueues
* Convenient event wrappers

## Contributing
We appreciate any contribution, from fixing a comment to implementing complex designs.
Being a contributor to this project, you agree and confirm that:
  * You did your work - no plagiarism allowed
      Any plagiarized work will not be merged.
  * Your work will be distributed under MIT License once your pull request is merged
  * You submitted work fulfils or mostly fulfils our styles and standards
    
## License
Colibri is open source under the MIT license, see the [LICENSE](LICENSE) files.
