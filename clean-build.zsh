#! /bin/zsh

rm -rf build/ install/ log/ && echo "Cleaned build, install, and log directories." && \
colcon build --symlink-install --merge-install && source install/setup.zsh && echo "Rebuild done." 