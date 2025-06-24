#! /bin/zsh

colcon build --symlink-install --merge-install && source install/setup.zsh && echo "Rebuild done." && ros2 run pathfinder pathfinder