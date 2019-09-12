#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(Cheetah-Vision)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
    #pointcloud
    add_executable(cheetah-vision cheetah_vision.cpp ../example.hpp)
    set_property(TARGET cheetah-vision PROPERTY CXX_STANDARD 11)
    target_link_libraries(cheetah-vision ${DEPENDENCIES})
    target_link_libraries(cheetah-vision lcm)
    find_package( OpenCV REQUIRED )
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(cheetah-vision ${OpenCV_LIBS})
    include_directories(cheetah-vision ../)
    set_target_properties (cheetah-vision PROPERTIES FOLDER Examples)
    install(TARGETS cheetah-vision RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
