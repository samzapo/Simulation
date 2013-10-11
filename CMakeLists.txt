cmake_minimum_required(VERSION 2.8.3)

project(Sim)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules CMakeModules)
set(THIRDPARTY ~samzapo/ThirdParty)

if(CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        message(STATUS "C++11 activated.")
        add_definitions("-std=gnu++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        add_definitions("-std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")
   endif()
else(CMAKE_COMPILER_IS_GNUCXX)
   add_definitions("-std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX)

find_package(Boost REQUIRED)
find_package(OSG REQUIRED)
option(BUILD_SHARED_LIBS "Build as a shared library?" ON)



# you may have to add additional includes in here...

include_directories(include /usr/include /usr/local/include include ${Boost_INCLUDE_DIR} ${THIRDPARTY}/Ravelin/include ${THIRDPARTY}/Opt/include /usr/local/Cellar/qhull/2012.1/include/libqhull ${THIRDPARTY}/Moby/include /usr/include/libxml2 ${Boost_INCLUDE_DIR} ${OSG_INCLUDE_DIR})
link_directories(/usr/local/lib /usr/lib ${THIRDPARTY}/Ravelin/build  ${THIRDPARTY}/Opt/build)

set(sim_src src/Simulation.cpp src/Integration.cpp src/Collision.cpp src/RigidBodyDynamics.cpp)
#set(vis_src src/Visualization.cpp )

add_executable(${PROJECT_NAME}_test ${sim_src})
#add_executable(${PROJECT_NAME}_vis ${vis_src})

target_link_libraries(${PROJECT_NAME}_test Ravelin glut GLU)
#target_link_libraries(${PROJECT_NAME}_vis glut GLU)
