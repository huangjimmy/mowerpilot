# Log
## 2019-04-15
今日升级了macos和Xcode以后，编译出现以下错误

```c++
In file included from /Users/jimmy/Documents/workspace/ekf2/libraries/AP_Common/../../libraries/AP_Param/AP_Param.h:23:
/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/include/c++/v1/string.h:76:13: error: functions that differ only in their return type cannot be overloaded
const char* strchr(const char* __s, int __c) {return __libcpp_strchr(__s, __c);}
      ~~~~~ ^
/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.14.sdk/usr/include/string.h:76:7: note: previous declaration is here
char    *strchr(const char *__s, int __c);
~~~~~~~~~^
```

后来发现不能在AP_Common.h中添加如下代码，原本以为这样能使代码在windows，macos和linux都能编译通过，没想到如果添加下列代码后mac下也有编译问题
```c++
#ifndef __GNUC__
#define __attribute__(x)
#endif
```

### 今日找到了EKF所需的输入参数的含义

资料来源 https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml


#### last_update_ms和last_update_usec
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.

根据资料来源，数据类型应该为uint64_t，不过实际上代码使用的是uint32_t

#### GPS含义
lat latitude WGS 84坐标系下的latitude乘以1e7


lng longitude WGS 84坐标系的longitude乘以1e7


alt altitude 海拔高度，单位毫米(mm)


ground_speed 地速 单位 m/s


course ground_course 小车运动的方向，单位为"度°" 0°为正北 正东为90°，从硬件读取到的数据的单位为可能为0.01°，也可能为1°，计算所需的单位是1°



velocity x = north 方向的速度 y east方向的速度 z 向下(重力方向)的速度 单位都是m/s


从硬件读取到GPS信息为ground speed, velocity需要根据velocity和course换算，换算公式为 gps_heading=ground_couse转换为弧度 z = 0, x = ground_speed*cos(gps_heading), y =  ground_course*sin(gps_heading)

#### IMU的含义

* 此项仍为TODO *

body frame指的是这个坐标系 http://planning.cs.uiuc.edu/node101.html#fig:yawpitchroll

delAng 为读取的gyro传感器数据


定义
AP_NavEKF2_core.h的411行和AP_NavEKF3_core.h的440行
```c++
    struct imu_elements {
        Vector3f    delAng;         // IMU delta angle measurements in body frame (rad)
        Vector3f    delVel;         // IMU delta velocity measurements in body frame (m/sec)
        float       delAngDT;       // time interval over which delAng has been measured (sec)
        float       delVelDT;       // time interval over which delVelDT has been measured (sec)
        uint32_t    time_ms;        // measurement timestamp (msec)
    };
```

### EKF的计算逻辑

* TODO *

Extended Kalman Filter的对应代码分布在

AP_NavEKF2_{MagFusion, Measurements,Outputs,PosVelFusion}.cpp中

AP_NavEKF2_{AirDataFusion, OptFlowFusion,RngBcnFusion}.cpp

其中，AP_NavEKF2_core.h中383到402行有以下定义
```c++
    // the states are available in two forms, either as a Vector31, or
    // broken down as individual elements. Both are equivalent (same
    // memory)
    struct state_elements {
        Vector3f    angErr;         // 0..2
        Vector3f    velocity;       // 3..5
        Vector3f    position;       // 6..8
        Vector3f    gyro_bias;      // 9..11
        Vector3f    gyro_scale;     // 12..14
        float       accel_zbias;    // 15
        Vector3f    earth_magfield; // 16..18
        Vector3f    body_magfield;  // 19..21
        Vector2f    wind_vel;       // 22..23
        Quaternion  quat;           // 24..27
    };

    union {
        Vector28 statesArray;
        struct state_elements stateStruct;
    };
```

404到408行定义，如下，猜测，EKF的计算结果为velocity和position
```c++
    struct output_elements {
        Quaternion  quat;           // 0..3
        Vector3f    velocity;       // 4..6
        Vector3f    position;       // 7..9
    };
```