cmake_minimum_required(VERSION 3.8)
project(erob_soem_demo)

# 设置默认的构建类型为调试模式
set(CMAKE_BUILD_TYPE Debug)

project(erob_soem_demo LANGUAGES C CXX)
find_package(Threads REQUIRED)

include_directories(/home/erobman/SOEM_ethercat/install/SOEM/include/soem)
link_directories(/home/erobman/SOEM_ethercat/install/SOEM/lib)

add_executable(eRob_PT eRob_PT.cpp friction_identify.cpp yzdn_math.cpp lpf.cpp motor_control.cpp tf_2rd.cpp freq_scan.cpp slop.cpp)
# 设置头文件的搜索路径为当前目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eRob_PT soem)
install(TARGETS eRob_PT DESTINATION bin)