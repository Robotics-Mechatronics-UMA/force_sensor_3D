#!/usr/bin/env python

from setuptools import setup

setup(
    name='3-axis_force_sensor_ESP32',
    version='0.0.0',
    packages=['3-axis_force_sensor_ESP32'],
    scripts=['scripts/force_ros_node.py'],
    package_dir={'': 'scripts'},
)