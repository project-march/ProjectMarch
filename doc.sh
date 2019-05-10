#!/usr/bin/env bash

# @TODO(Isha) add documentation for march_simulation

packages="march_fake_sensor_data"

mkdir -p docs/html
for package in $packages; do
    catkin document $package --no-deps
    mv docs/$package/html docs/html/$package
done