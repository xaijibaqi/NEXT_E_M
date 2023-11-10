cmake_minimum_required(VERSION 3.26)
project(NEXT_E_M)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_library(MVS_LIB libMvCameraControl.so /opt/MVS/lib/64/)
find_library(eigan  Dense /home/m/eigan/Dense)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(NEXT_E_M)
include_directories (/usr/include)
include_directories(/hmoe/m/eigan)

add_executable(NEXT_E_M main.cpp
        include/SERIAL.h
        include/ARMOUR.h
        include/ROBOT.h
        include/Kalman_filtering.h
        src/SERIAL.cpp
        src/ARMOUR.cpp
        src/ROBOT.cpp
        src/Kalman_filtering.cpp
        hik_camera/include/HikCam.hpp
        hik_camera/HikCam.cpp
)

target_link_libraries(NEXT_E_M ${OpenCV_LIBS})
target_link_libraries(NEXT_E_M ${MVS_LIB})
target_link_libraries(NEXT_E_M  ${CERES_LIBRARIES} -lserialport)