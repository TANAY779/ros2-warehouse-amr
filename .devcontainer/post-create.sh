#!/usr/bin/env bash
# Runs once after the dev container is created (working directory: the colcon workspace).
# The dependency steps match the build-test job in .github/workflows/ci.yml.
set -euo pipefail

vcs import --input dependencies.repos src
sudo apt-get update
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

pre-commit install
