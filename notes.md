## OpenCV

```c++
#define _UI_IMG_WIDTH_ 640
#define _UI_IMG_HEIGHT_ 320
pre_img_right = Mat::zeros(_UI_IMG_HEIGHT_, _UI_IMG_WIDTH_, CV_8UC3);
Scalar color(0, 255, 255);
Scalar(0, 255, 255);
mat4map_.setTo(Scalar(0, 0, 0));
img_left.release();

std::string filename;
filename = std::to_string(frame->frame_id_) + ".jpg";
img_left = imread(ConfigSim::log_dir_ + ".images/1/" + filename);
img_left.copyTo(mat4show_.rowRange(0, _UI_IMG_HEIGHT_).colRange(0, _UI_IMG_WIDTH_));

void SimViewer::ShowFrame() {
    Mat mat4show_resized;
    resize(mat4show_, mat4show_resized, Size(), 0.8, 0.8);
    imshow("simulation", mat4show_resized);
    waitKey(1);
}

using namespace cv;
line(img_left, Point(0, 0), Point(50, 0), Scalar(0, 255, 0), 1);

char tmp[100] = {0};
sprintf(tmp, "%.2lf", park.park_confidences_[i]);
Scalar text_color = park.park_confidences_[i] > 0.1 ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
putText(img, tmp, Point, FONT_HERSHEY_COMPLEX_SMALL, 0.65, text_color, 1);

circle(mat4map_, center, 3, Scalar(255, 0, 255), -1);
```








## C++

### 单例设计模式

- 类的构造函数和析构函数是私有的
- 包含一个私有的指向静态类本身的一个指针
- 静态类只能访问静态类成员函数，静态类成员函数只能访问静态成员变量，所以类的方法和属性都是静态的
- 调用通过 类名::方法 的形式进行调用，由于构造函数是私有的，不能创建类的实例

```c++
#ifndef XSD_SIM_CONFIG_H
#define XSD_SIM_CONFIG_H

#include <string>
#include "Yaml.hpp"

class ConfigSim {
private:
    ConfigSim(){};
    ~ConfigSim(){};
    static ConfigSim* instance_;

public:
    static ConfigSim* get_instance();
    static void ParseConfigFile(const char* config_file_path);

    // dataset
    static std::string log_dir_;
    static std::string ground_truth_dir_;
    static std::string locate_calib_dir_;
    static std::string analysis_result_dir_;
    static std::string video_save_dir_;

    // visualization
    static int fps_;
    static int draw_tracked_freespace_;
    static int draw_single_freespace_;
    static bool enable_viewer_;
    static bool save_video_;
    static bool show_result_;

    // debug
    static int start_frame_;
    static bool use_log_;
};

#endif //XSD_SIM_CONFIG_H
```

- 静态类的成员在类内声明，必须要在内外进行定义（初始化），不需要static关键字

```c++
#include "config.h"

ConfigSim *ConfigSim::instance_;

std::string ConfigSim::log_dir_;
std::string ConfigSim::ground_truth_dir_;
std::string ConfigSim::locate_calib_dir_;
std::string ConfigSim::analysis_result_dir_;
std::string ConfigSim::video_save_dir_;

int ConfigSim::fps_;
int ConfigSim::draw_tracked_freespace_;
int ConfigSim::draw_single_freespaces_;
bool ConfigSim::enable_viewer_;
bool ConfigSim::save_video_;
bool ConfigSim::show_result_;

// debug
int ConfigSim::start_frame_;
bool ConfigSim::use_log_;

ConfigSim *ConfigSim::get_instance() {
    if (instance_ == nullptr) instance_ = new ConfigSim();
    return instance_;
}

void ConfigSim::ParseConfigFile(const char *config_file_path) {
    // ...
}
```



### 典型类的设计

- 头文件中的`static`关键字的作用是  这个头文件中定义的运算符重载函数和一些数学函数仅在包含该头文件的源文件中有效，名字很相似
- 重载`+ - * /` 可以在类外进行设计， 重载`= *= /=`可以在类内进行
- 一些函数可以在内外设计，也可以在类内设计，调用方式不同， 类外部  `xsd::Norm(pt)`，类内部 `pt.norm()`

```c++
//
// Created by heqz1 on 2021/10/11.
//

#ifndef XSD_SIM_POINT_H
#define XSD_SIM_POINT_H

#include <cmath>

namespace xsd {

    #define EPSILON 1e-5f

    class Point2f {
    public:
        float x_;
        float y_;

        // construction
        Point2f() {
            x_ = 0.f;
            y_ = 0.f;
        }
        Point2f(int x, int y) {
            x_ = (float)x;
            y_ = (float)y;
        }
        Point2f(double x, double y) {
            x_ = (float)x;
            y_ = (float)y;
        }
        Point2f(float x, float y) {
            x_ = x;
            y_ = y;
        }
        Point2f(const Point2f& pt) {
            x_ = pt.x_;
            y_ = pt.y_;
        }

        // operator(*=,/=,=)
        void operator=(const Point2f& pt) {
            x_ = pt.x_;
            y_ = pt.y_;
        }
        void operator*=(int scale) {
            x_ *= (float)scale;
            y_ *= (float)scale;
        }
        void operator*=(float scale) {
            x_ *= (float)scale;
            y_ *= (float)scale;
        }
        void operator*=(double scale) {
            x_ *= scale;
            y_ *= scale;
        }
        void operator/=(int scale) {
            x_ /= (float)scale;
            y_ /= (float)scale;
        }
        void operator/=(float scale) {
            x_ /= scale;
            y_ /= scale;
        }
        void operator/=(double scale) {
            x_ /= scale;
            y_ /= scale;
        }

        // operator(dot, cross)
        float dot(const Point2f& pt) { return (x_ * pt.x_ + y_ * pt.y_); }
        float cross(const Point2f& pt) { return (x_ * pt.y_ - y_ * pt.x_); }
    };

    // operator(+, -, *, /)
    static inline Point2f operator+(const Point2f& a, const Point2f& b) { return Point2f(a.x_ + b.x_, a.y_ + b.y_); }
    static inline Point2f operator-(const Point2f& a, const Point2f& b) { return Point2f(a.x_ - b.x_, a.y_ - b.y_); }
    static inline Point2f operator-(const Point2f& a) { return Point2f(-a.x_, -a.y_); }
    static inline Point2f operator*(const Point2f& a, int b) { return Point2f(a.x_ * b, a.y_ * b); }
    static inline Point2f operator*(const Point2f& a, float b) { return Point2f(a.x_ * b, a.y_ * b); }
    static inline Point2f operator*(const Point2f& a, double b) { return Point2f(a.x_ * b, a.y_ * b); }
    static inline Point2f operator/(const Point2f& a, int b) { return Point2f(a.x_ / b, a.y_ / b); }
    static inline Point2f operator/(const Point2f& a, float b) { return Point2f(a.x_ / b, a.y_ / b); }
    static inline Point2f operator/(const Point2f& a, double b) { return Point2f(a.x_ / b, a.y_ / b); }

    // norm
    static inline float Norm(const Point2f& pt) { return std::sqrt(pt.x_ * pt.x_ + pt.y_ * pt.y_); }

    // distance
    static inline float DistancePtoP(const Point2f& a, const Point2f& b) { return Norm(a - b);}
    static inline float DistancePtoL(const Point2f& pt, const Point2f& v1, const Point2f& v2) {
        if (Norm(v2 - v1) <= EPSILON) {
            return DistancePtoP(pt, v1);
        }
        Point2f dpt = v2 - v1;
        return fabs(dpt.cross(v2 - v1)) / Norm(v2 - v1);
    }
}

#endif //XSD_SIM_POINT_H
```



### other

```c++
// 预编译的时候原样替换，所以注意有些括号不可以省略
// SIGN(3) --> ((3) < 0 ? -1 : ((3) > 0))
#define SIGN(x) ((x) < 0 ? -1 : ((x) > 0))

// struct 默认 public, class 默认 private 注意初始化列表的方式，符合Point2f构造函数的参数形式既可以初始化
struct KeyPointT {
    Point2f position_;
    float sigma_;
    KeyPointT() : position_(0, 0), sigma_(_XSD_MAX_SIGMA_) {}
};

// 静态成员变量属于类的，可以通过 类名::成员变量 访问
// 类中的子类或者结构体，也是属于类的， 可以通过 类名::子类名 来进行定义变量
// 结果体这种与类成员思想类似，可以这样调用
enum XsdMsgType {
    XSD_MSG_HEART_BEAT = 0,
    XSD_MSG_POSE = 1,
    XSD_MSG_JPEG,
    XSD_MSG_YUV,
    XSD_MSG_CNN_OUT,
    XSD_MSG_CAN_INFO,
    XSD_MSG_FRAME_PACKAGE,  // include channel_id, frame_id, cnn_out, pose_compensated
    XSD_MSG_TRACKED_PARK,  // XSD OUTPUT
    XSD_MSG_LOCAL_FILE,  // need to be sent in the first time client connect to server
};

XsdMsgType::XSD_MSG_LOCAL_FILE;

// union
struct XsdCanMsgT {
    uint16_t can_id_;
    uint64_t time_stamp_;
    union {
        xp_can_adcan_xpu_avm_apa_ctrl_t apa_ctrl_;
        xp_can_adcan_xpu_veh_location_t veh_location_;
        xp_can_adcan_avm_scu_parking_info_t parking_info_;
        xp_can_adcan_avm_scu_free_space_t freespace_;
    } msg_;
    uint16_t calib_status_;
};

// 不能在子类的初始化列表中初始化父类的成员变量
// 可以在子类的初始化列表中用参数初始化父类，父类有该参数对应的构造函数
// 例子： GlobalTracker 继承父类 Tracker
GlobalTracker::GlobalTracker(const TrackerParam& _param)
  : Tracker(_param)
{
  trackID_ = 0;
  init_ = true;
  startTracking_ = false;
  rng_ = cv::RNG(12345);
}

```







## GIT

```bash
git status
git checkout <file>
git add <file>

git commit -s -m ""
git log
git commit --amend
git push --force
git cherry-pick <commit_id>
git pull
git checkout <branch>
git push origin <local_branch>:<remote_brance>
git checkout -b local_branch remote_branch
git branch -r

git add --all
git rm -r --cached <folder>
```



## CMAKE

### 常规配置

- find_package会在 `/usr/local/share` 等目录搜索 *.cmake文件，里面定义了安装库的相关cmake变量，如 ${OpenCV_LIBS}
- 如果不使用find_package的方式，可以通过定义头文件目录`include_directories`，库的目录`link_directories`，然后`target_link_libraries`

```cmake
project(JPDA_tracker)

cmake_minimum_required( VERSION 3.10 )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories()

# link_directories()

file(GLOB SRC
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

add_executable(JPDA_tracker ${SRC})

target_link_libraries(JPDA_tracker PUBLIC ${OpenCV_LIBS})

# target_link_libraries(JPDA_tracker PUBLIC
#         opencv_calib3d
#         opencv_core
#         opencv_features2d
#         opencv_flann
#         opencv_highgui
#         opencv_imgcodecs
#         opencv_imgproc
#         opencv_ml
#         opencv_objdetect
#         opencv_photo
#         opencv_shape
#         opencv_stitching
#         opencv_superres
#         opencv_video
#         opencv_videoio
#         opencv_videostab
#         zlib
# )
```



```cmake
cmake_minimum_required( VERSION 3.20 )

project(XSD_SIM)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -O0 -g")
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")
set(CMAKE_BUILD_TYPE debug)

set(OpenCV_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../3rd_party/opencv/include/)
set(json_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../3rd_party/json/)
set(yaml_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../3rd_party/yaml/)

# opencv
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#message(STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})
#message(STATUS "OpenCV_LIBS: " ${OpenCV_LIBS})

# output
set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/output)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})

# XSD
include_directories(
        ${CMAKE_SOURCE_DIR}/../
        ${CMAKE_SOURCE_DIR}/../communication/
        ${OpenCV_INCLUDE_DIR}
        ${json_INCLUDE_DIR}
        ${yaml_INCLUDE_DIR}
)

link_directories(${CMAKE_SOURCE_DIR}/../3rd_party/opencv/lib/)

file(GLOB SRC
        ${CMAKE_SOURCE_DIR}/xsd_simulation.cpp
        ${CMAKE_SOURCE_DIR}/json_data_tool.cpp
        ${CMAKE_SOURCE_DIR}/config.cpp
        ${CMAKE_SOURCE_DIR}/../3rd_party/yaml/Yaml.cpp)

add_executable(XSD_SIM ${SRC})

target_link_libraries(XSD_SIM PUBLIC
        opencv_calib3d
        opencv_core
        opencv_features2d
        opencv_flann
        opencv_highgui
        opencv_imgcodecs
        opencv_imgproc
        opencv_ml
        opencv_objdetect
        opencv_photo
        opencv_shape
        opencv_stitching
        opencv_superres
        opencv_video
        opencv_videoio
        opencv_videostab
        zlib
)

#target_link_libraries(XSD_SIM PUBLIC ${OpenCV_LIBS})
```





### 交叉编译配置

```cmake
cmake_minimum_required(VERSION 3.5)

#1.toolchain related
set(TOOLCHAIN_BASE ${CMAKE_SOURCE_DIR}/../../../XP_D55/toolchain)

set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BASE}/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++)
set(CMAKE_C_COMPILER ${TOOLCHAIN_BASE}/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc)

# set(CMAKE_CXX_FLAGS "-Wall -Werror -std=c++14 -O2 -g")
# set(CMAKE_CXX_FLAGS "-Wall -std=c++14 -O2 -g")
# add_definitions(-std=c++14)
# add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
# add_definitions("-Wno-psabi")

include_directories(
        ${TOOLCHAIN_BASE}/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/lib/gcc/arm-linux-gnueabihf/5.3.1/include
        ${TOOLCHAIN_BASE}/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/arm-linux-gnueabihf/include/c++/5.3.1
)
link_directories(
        ${TOOLCHAIN_BASE}/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/arm-linux-gnueabihf/libc/lib/
)

project(XSD_APP CXX C)

set(CMAKE_CXX_FLAGS "-Wall -std=c++14 -O2 -g -s")
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/output)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})

SET(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")

# xsd
include_directories(
        ${CMAKE_SOURCE_DIR}/../
        ${CMAKE_SOURCE_DIR}/../../
        ${CMAKE_SOURCE_DIR}/../algorithm/
        ${CMAKE_SOURCE_DIR}/../../xp_runtime/include/
        ${CMAKE_SOURCE_DIR}/../3rd_party/opencv/include/
        ${CMAKE_SOURCE_DIR}/../3rd_party/yaml-cpp/include/
        ${CMAKE_SOURCE_DIR}/../calibration/
        ${CMAKE_SOURCE_DIR}/../calibration/algorithm/
)

file(GLOB SRC
      ${CMAKE_SOURCE_DIR}/../algorithm/*.cpp
      ${CMAKE_SOURCE_DIR}/../calibration/*.cpp
      ${CMAKE_SOURCE_DIR}/../3rd_party/yaml/*.cpp
      ${CMAKE_SOURCE_DIR}/../3rd_party/yaml-cpp/*.cpp
      ${CMAKE_SOURCE_DIR}/../communication/*.cpp
      ${CMAKE_SOURCE_DIR}/../common/*.c
      ${CMAKE_SOURCE_DIR}/../common/*.cpp
      ${CMAKE_SOURCE_DIR}/../calibration/algorithm/*.cpp

      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_timer.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_can_adcan.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_can_api.c
      #${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_can_service.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_ipc_socket.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_network_api.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_queue.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_sysmem.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_task.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_postproc_api.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_selfcheck.c
      ${CMAKE_SOURCE_DIR}/../../xp_runtime/src/xp_mutex.c
      )

link_directories(
        ${CMAKE_SOURCE_DIR}/../3rd_party/opencv/lib
)
add_executable(XSD_APP 
        ${SRC} 
        ${CMAKE_SOURCE_DIR}/xsd_app.cpp
        )

# set_target_properties(XSD_APP PROPERTIES LINK_FLAGS_RELEASE -s)


target_link_libraries(${PROJECT_NAME}
        PUBLIC
        opencv_calib3d #1.9m
        opencv_features2d #1.1m
        #opencv_flann
        opencv_imgproc #4.3m
        opencv_core #3.8m
        #opencv_imgcodecs
	${CMAKE_SOURCE_DIR}/../3rd_party/yaml-cpp/lib/libyaml-cpp.a  #0.73
        stdc++ 
        m 
        -lrt
        -lc
        -lzlib
        -lpthread
        )

# target_link_libraries(XSD_APP PUBLIC pthread -lm -lrt
#         )

```



## 操作系统

### 信号





## nolhmann json





















































