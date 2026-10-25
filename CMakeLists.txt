cmake_minimum_required(VERSION 3.16)
project(MotionPlannerGUI LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/model/RobotModel.cpp
    src/model/PlannerWrapper.cpp
    src/model/TrajectoryGenerator.cpp
    src/view/ControlPanel.cpp
    src/view/VisualizationWidget.cpp
    src/view/JointPlotWidget.cpp
    src/controller/PlannerController.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Widgets
    ${OMPL_LIBRARIES}
    Eigen3::Eigen
)
