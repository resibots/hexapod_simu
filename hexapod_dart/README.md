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
#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty hexapod_dart detection
"""

import os
from waflib.Configure import conf


def options(opt):
	opt.add_option('--hexapod_dart', type='string', help='path to hexapod_dart', dest='hexapod_dart')

@conf
def check_hexapod_dart(conf):
	includes_check = ['/usr/local/include/hexapod_dart', '/usr/include/hexapod_dart']
	libs_check = ['/usr/local/lib', '/usr/lib']

	# You can customize where you want to check
	# e.g. here we search also in a folder defined by an environmental variable
	if 'RESIBOTS_DIR' in os.environ:
		includes_check = [os.environ['RESIBOTS_DIR'] + '/include/hexapod_dart'] + includes_check
		libs_check = [os.environ['RESIBOTS_DIR'] + '/lib'] + libs_check

	if conf.options.hexapod_dart:
		includes_check = [conf.options.hexapod_dart + '/include/hexapod_dart']
		libs_check = [conf.options.hexapod_dart + '/lib']

	try:
		conf.start_msg('Checking for hexapod_dart includes')
		res = conf.find_file('hexapod.hpp', includes_check)
		res = res and conf.find_file('hexapod_control.hpp', includes_check)
		res = res and conf.find_file('hexapod_dart_simu.hpp', includes_check)
		conf.end_msg('ok')
		conf.start_msg('Checking for hexapod_dart libs')
		res = res and conf.find_file('libhexapod_dart.a', libs_check)
		conf.end_msg('ok')
		conf.env.INCLUDES_HEXAPOD_DART = includes_check
		conf.env.LIBPATH_HEXAPOD_DART = libs_check
		conf.env.LIB_HEXAPOD_DART = ['hexapod_dart']
		conf.start_msg('Checking for hexapod_dart graphics libs')
		res = res and conf.find_file('libhexapod_dart_graphic.a', libs_check)
		conf.end_msg('ok')
		conf.env.LIB_HEXAPOD_DART.append('hexapod_dart_graphic')
	except:
		conf.end_msg('Not found', 'RED')
		return
	return 1

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