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
echo "open src/my_model/docs/html/index.html"

# 2. run my_controller's "docs" target
## TODO
##echo "open src/my_controller/docs/html/index.html"

# 3. combine all docs
## TODO
##echo "open src/docs/html/index.html"

