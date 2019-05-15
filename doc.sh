#!/usr/bin/env bash

packages="march_rqt_gait_generator"

mkdir -p docs/html
for package in $packages; do
    catkin document $package --no-deps
    mv docs/$package/html docs/html/$package
done