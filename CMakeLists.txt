cmake_minimum_required(VERSION 3.5)
project(camera_streaming_pipeline)

# set(CMAKE_BUILD_TYPE Debug)
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
    RectifyImage.srv
    RequestBestImage.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} ${PYLON_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE IMAGE_STREAMER_SRC src/ImageStreamer/*)
file(GLOB_RECURSE IMAGE_STREAMER_INCLUDE include/ImageStreamer/*)
set(EXEC_NAME image_streamer)
add_executable( ${EXEC_NAME} ${IMAGE_STREAMER_SRC} ${IMAGE_STREAMER_INCLUDE})
add_dependencies(${EXEC_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${EXEC_NAME} ${PYLON_LIBRARIES} ${catkin_LIBRARIES})

file(GLOB_RECURSE IMAGE_RECTIFIER_SRC src/ImageRectifier/*)
file(GLOB_RECURSE IMAGE_RECTIFIER_INCLUDE include/ImageRectifier/*)
set(EXEC_NAME image_rectifier)
add_executable( ${EXEC_NAME} ${IMAGE_RECTIFIER_SRC} ${IMAGE_RECTIFIER_INCLUDE})
add_dependencies(${EXEC_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${EXEC_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})

file(GLOB_RECURSE IMAGE_QUALITY_SELECTOR_SRC src/ImageQualitySelector/*)
file(GLOB_RECURSE IMAGE_QUALITY_SELECTOR_INCLUDE include/ImageQualitySelector/*)
set(EXEC_NAME image_quality_selector)

file(GLOB_RECURSE PIPELINE_CONTROLLER_SRC src/PipelineController/*)
file(GLOB_RECURSE PIPELINE_CONTROLLER_INCLUDE include/PipelineController/*)
set(EXEC_NAME pipeline_controller)
add_executable( ${EXEC_NAME} ${PIPELINE_CONTROLLER_SRC} ${PIPELINE_CONTROLLER_INCLUDE})
add_dependencies(${EXEC_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${EXEC_NAME} ${catkin_LIBRARIES})


