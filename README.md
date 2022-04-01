# 环境配置方法

1. ~~配置科学上网环境，保证能访问GitHub~~使用hub.fastgit.org镜像加速，避免直接访问GitHub
2. 修改Ubuntu软件源，改成速度快的源
3. 执行env.sh脚本安装需要的软件包和相机sdk：`sudo bash ./env.sh`
6. `sh ncnn.sh`编译ncnn
7. `mkdir -p build && cd build && cmake .. && make`编译
8. 从[文件网盘/自瞄模型](https://tj-robomaster.coding.net/p/armor/files/all/DF4)把模型（*.bin, *.param）下到build中
9. 在build目录中运行`sudo ./attack_mind`

# 分支说明

| 分支名        | 说明                     |
|---------------|------------------------|
| master        | 步兵代码with曼德卫视相机 |
| master-dahua  | 已废弃                   |
| shaobing      | 下哨兵代码               |
| shaobingshang | 上哨兵代码               |
| compen        | 已废弃                   |
| curve         | 已废弃                   |

1. 哨兵代码与步兵代码为什么分开？

以前是不分开的，但是比赛时发现哨兵由于位置较高，视野中目标装甲板的形状与步兵有所不同，导致在装甲板识别和判定部分有所区别，故开了两个分支

# 目录结构说明

| 文件           | 说明                     | 备注                               |
|----------------|--------------------------|------------------------------------|
| daemon/        | 守护进程                 | 详见内部的README文件               |
| data/          | 程序运行时数据           | 详见内部的README文件               |
| doc/           | 开发文档                 | 详见内部的README文件               |
| Model/         | 数字分类器模型           |                                    |
| info/          | 风车代码使用             |                                    |
| pics/          | 风车代码使用             |                                    |
| tools/         | 开发小工具存放地         |                                    |
| include/       | C++头文件                | h/hpp格式                          |
| source/        | C++源代码文件            | hpp格式                            |
| main.cpp       | C++源代码文件            | cpp格式，主程序入口                 |
| .clang-format  | 格式化配置               |                                    |
| .gitignore     |                          |                                    |
| config.toml    | 程序运行时配置文件       |                                    |
| env.sh         | 环境配置脚本             |                                    |
| CMakeLists.txt | 编译脚本                 |                                    |
| README.md      | 说明文档                 |                                    |
