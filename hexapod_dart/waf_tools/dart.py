#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty DART detection
"""

import os
from waflib.Configure import conf


def options(opt):
	opt.add_option('--dart', type='string', help='path to dart sim', dest='dart')

@conf
def check_dart(conf):
	includes_check = ['/usr/local/include', '/usr/include']
	libs_check = ['/usr/local/lib', '/usr/lib']
	if 'RESIBOTS_DIR' in os.environ:
		includes_check = [os.environ['RESIBOTS_DIR'] + '/include'] + includes_check
		libs_check = [os.environ['RESIBOTS_DIR'] + '/lib'] + libs_check

	if conf.options.controller:
		includes_check = [conf.options.controller + '/include']
		libs_check = [conf.options.controller + '/lib']

	# DART requires some of bullets includes (if installed with bullet enabled)
	includes_check = includes_check + ['/usr/local/include/bullet', '/usr/include/bullet']

	try:
		conf.start_msg('Checking for DART includes')
		res = conf.find_file('dart/dart.h', includes_check)
		res = res and conf.find_file('dart/dart-core.h', includes_check)
		conf.end_msg('ok')
		conf.start_msg('Checking for optional Bullet includes')
		res = res and conf.find_file('btBulletCollisionCommon.h', includes_check)
		conf.end_msg('ok')
		conf.start_msg('Checking for DART libs')
		res = res and conf.find_file('libdart.so', libs_check)
		res = res and conf.find_file('libdart-core.so', libs_check)
		conf.end_msg('ok')
		conf.env.INCLUDES_DART = includes_check
		conf.env.LIBPATH_DART = libs_check
		conf.env.LIB_DART = ['dart', 'dart-core']
		conf.start_msg('Checking for DART OSG includes')
		res = res and conf.find_file('osgDart/osgDart.h', includes_check)
		conf.end_msg('ok')
		conf.start_msg('Checking for DART OSG libs')
		res = res and conf.find_file('libosgDart.so', libs_check)
		conf.end_msg('ok')
		conf.env.LIB_DART.append('osgDart')
		conf.get_env()['BUILD_GRAPHIC'] = True
	except:
		conf.end_msg('Not found', 'RED')
		return
	return 1
