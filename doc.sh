#!/usr/bin/env bash

packages="march_fake_sensor_data march_simulation"

mkdir -p docs/html
for package in $packages; do
    catkin document $package --no-deps
    mv docs/$package/html docs/html/$package
done