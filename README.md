# mowerpilot

This is the repo designated for preparing for an unattended mower. It uses Extended Kalman Filter 

## prerequisite

### Ubuntu 19.04 编译环境

通过如下命令安装编译环境(包含cross compile工具)

```bash
apt-get -y update && apt-get -y upgrade
apt-get install -y python3 gcc g++ make cmake autoconf clang python-pip
pip install future
apt-get install -y gcc-arm-linux-gnueabi g++-arm-linux-gnueabi gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf libc6-armel-cross libc6-dev-armel-cross binutils-arm-linux-gnueabi libncurses5-dev 
```

### Mac 编译环境 (不含cross compile工具)

通过Mac App Store安装Xcode

通过以下命令安装brew

```bash
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

安装cmake

```bash
brew install cmake
```

## build （适用于macos和linux)

假设mowerpilot在/home/build/mower目录下，打开Terminal，

```bash
cd /home/build/mower
cd build
cmake ..
cd ..
cmake --build build --target Main -- -j 4
```

build目录下就会生成一个可执行的Main程序，文件名也是Main

## run/demo

在mower目录下执行

```bash
build/Main
```

改程序就会读取/home/build/mower目录下的.log文件并根据log文件初始化Extended Kalman Filter，然后进行Update和Predict

## 当前问题

log文件的GPS和Gyro等数据为从手机记录的，与示例程序所需的传感器数据的格式不一致，因此，当前的程序仅仅是在流程上执行了Filter的初始化、更新和预测步骤，所属出的Predict数据由于log数据原因，无物理意义上的参考价值。

## Extended Kalman Filter基本用法

```C++
#include <AP_NavEKF2/AP_NavEKF2.h>
```

```C++
NavEKF2 navEKF2; //声明一个NavEKF2变量
```

```C++
//初始化Filter，初始化的时候，需要提供IMU, GPS, Magnetometer数据，Barometer数据可不提供
navEKF2.InitialiseFilter(last_time_usec, 10, 1, imuData, gpsData, magnetoData, baroData);
```

```C++
//执行Filter的Update步骤，需要提供IMU, GPS, Magnetometer数据，Air Speed, Range Beacon和Barometer可不提供
navEKF2.UpdateFilter(last_time_usec, 0, imuData, gpsData, magnetoData, airSpdData, rngBcnData, baroData);
```

```C++
navEKF2.get* //调用以get开头的方法获取计算后的数据
```