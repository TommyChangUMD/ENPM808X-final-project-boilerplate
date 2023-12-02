#!/bin/bash
#
# A convenient script to create documentation
#
set -xue -o pipefail

# 1. run my_model's "docs" target
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select my_model \
       --cmake-target "docs"

# 2. show the coverage report
open src/my_model/docs/html/index.html
