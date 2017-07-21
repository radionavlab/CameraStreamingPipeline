cmake_minimum_required(VERSION 3.5)
project(camera_streaming_pipeline)

set(CMAKE_BUILD_TYPE Debug)
set(CATKIN_BUILD 1)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix -DCATKIN_BUILD")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/modules/")

find_package(catkin REQUIRED COMPONENTS
    image_transport
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    cv_bridge
)

find_package(Pylon REQUIRED)
find_package(OpenCV REQUIRED)

add_service_files(
    FILES
    RequestRawImages.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PYLON_INCLUDE_DIR}
)

# Header files
file(
    GLOB_RECURSE
    INCLUDE_FILES
    include/*
)

# Source files
file(
    GLOB_RECURSE
    SRC_FILES
    src/*
)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${INCLUDE_FILES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${PYLON_LIBRARIES} ${catkin_LIBRARIES})

