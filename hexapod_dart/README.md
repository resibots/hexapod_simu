# hexapod_dart

#### Here we keep the [DART] integration for our hexapods.

## Quick Start Guide

**UNDER CONSTRUCTION**

## How to compile

### Dependencies

- [hexapod_controller]: Simple sinusoidal gait controller
    - Get the code with `git clone https://github.com/resibots/hexapod_common.git`
    - Go to the `hexapod_controller` folder
    - Configure with `./waf configure --prefix=path_to_install`
    - Compile with `./waf build`
    - Install with `./waf install`
- [DART]: DART (Dynamic Animation and Robotics Toolkit) is a collaborative, cross-platform, open source library that provides data structures and algorithms for kinematic and dynamic applications in robotics and computer animation.
    - See the [official instructions](https://github.com/dartsim/dart/wiki/DART%205.1%20Installation%20for%20Ubuntu) for installation.
    - We use version 5.1 without bullet integration
- [Eigen]: Linear Algebra C++ Library

### Compile and install

- cd to `hexapod_dart` folder
- Configure with `./waf configure --prefix=path_to_install`
    - Optionally add `--controller=path_to_find_hexapod_controller`
- Compile with `./waf build`
- Install with `./waf install`

## How to use it in other projects

### Using the WAF build system

Add hexapod_dart as an external library using the following script:

```python

```

Then in your C++ code you would have something like the following:

```cpp
// previous includes
#include <hexapod_dart_simu.hpp>

// rest of code

HexapodDARTSimu simu(controller_parameters, robot_ptr);
simu.run(duration_in_secs);

// rest of code
```


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[DART]: http://dartsim.github.io/
[hexapod_controller]: https://github.com/resibots/hexapod_common
[Eigen]: http://eigen.tuxfamily.org/