# hexapod_robdyn

#### Here we keep the [robdyn] integration for our hexapods.

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
- [robdyn]: Dynamic simulator
    - Get the code: `git clone https://github.com/resibots/robdyn.git`
    - Configure for compilation/installation: `./waf configure --prefix=path_to_install`
    - Compile with `./waf build`
    - Install with `./waf install`
    - For more advanced options, look at [robdyn]'s repo.
- [ODE]: Open Dynamics Engine
- [Boost]: C++ Template Libraries
- [Eigen]: Linear Algebra C++ Library

### Compile and install

- cd to `hexapod_robdyn` folder
- Configure with `./waf configure --prefix=path_to_install`
    - Optionally add `--controller=path_to_find_hexapod_controller`
- Compile with `./waf build`
- Install with `./waf install`

## How to use it in other projects

### Using the WAF build system

Add hexapod_robdyn as an external library using the following script:

```python
#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty hexapod_robdyn detection
"""

import os
from waflib.Configure import conf


def options(opt):
	opt.add_option('--hexapod_robdyn', type='string', help='path to hexapod_robdyn', dest='hexapod_robdyn')

@conf
def check_hexapod_robdyn(conf):
	includes_check = ['/usr/local/include/hexapod_robdyn', '/usr/include/hexapod_robdyn']
	libs_check = ['/usr/local/lib', '/usr/lib']

	# You can customize where you want to check
	# e.g. here we search also in a folder defined by an environmental variable
	if 'RESIBOTS_DIR' in os.environ:
		includes_check = [os.environ['RESIBOTS_DIR'] + '/include/hexapod_robdyn'] + includes_check
		libs_check = [os.environ['RESIBOTS_DIR'] + '/lib'] + libs_check

	if conf.options.controller:
		includes_check = [conf.options.controller + '/include/hexapod_robdyn']
		libs_check = [conf.options.controller + '/lib']

	try:
		conf.start_msg('Checking for hexapod_robdyn includes')
		res = conf.find_file('hexapod.hpp', includes_check)
		res = res and conf.find_file('simu.hpp', includes_check)
		conf.end_msg('ok')
		conf.start_msg('Checking for hexapod_robdyn libs')
		res = res and conf.find_file('libhexapod_robdyn.a', libs_check)
		conf.end_msg('ok')
		conf.env.INCLUDES_HEXAPOD_CONTROLLER = includes_check
		conf.env.LIBPATH_HEXAPOD_CONTROLLER = libs_check
		conf.env.LIB_HEXAPOD_CONTROLLER = ['hexapod_robdyn']
		conf.start_msg('Checking for hexapod_robdyn graphics libs')
		res = res and conf.find_file('libhexapod_robdyn_graphic.a', libs_check)
		conf.end_msg('ok')
		conf.env.LIB_HEXAPOD_CONTROLLER.append('hexapod_robdyn_graphic')
	except:
		conf.end_msg('Not found', 'RED')
		return
	return 1
```

Then in your C++ code you would have something like the following:

```cpp
// previous includes
#include <simu.hpp>

// rest of code

Simu simu(controller_parameters, robot_ptr);
simu.run(duration_in_secs);

// rest of code
```


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[robdyn]: https://github.com/resibots/robdyn
[hexapod_controller]: https://github.com/resibots/hexapod_common
[ODE]: http://www.ode.org
[Boost]: http://www.boost.org
[Eigen]: http://eigen.tuxfamily.org/
