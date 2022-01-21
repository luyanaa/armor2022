#!/bin/bash
#
# 获取并编译ncnn


git clone https://e.coding.net/tj-robomaster/armor/ncnn.git -b 20211208-mirrored --depth 1
# 改一下submodule的网址
cd ncnn
git submodule update --init
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DNCNN_VULKAN=ON -DNCNN_SYSTEM_GLSLANG=OFF -DNCNN_BUILD_EXAMPLES=ON ..
make -j$(nproc) install
cd ..
