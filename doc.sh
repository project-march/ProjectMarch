#!/usr/bin/env bash

packages="march_hardware march_hardware_builder march_hardware_interface march_temperature_sensor_controller"

mkdir -p docs/html
for package in $packages; do
    catkin document $package --no-deps
    mv docs/$package/html docs/html/$package
done