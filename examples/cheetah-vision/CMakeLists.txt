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
    add_executable(cheetah-localization cheetah_localization.cpp ../example.hpp)
    add_executable(cheetah-pointcloud cheetah_pointcloud.cpp ../example.hpp)

    set_property(TARGET cheetah-vision PROPERTY CXX_STANDARD 11)
    set_property(TARGET cheetah-localization PROPERTY CXX_STANDARD 11)
    set_property(TARGET cheetah-pointcloud PROPERTY CXX_STANDARD 11)

    target_link_libraries(cheetah-vision ${DEPENDENCIES})
    target_link_libraries(cheetah-vision lcm)
    target_link_libraries(cheetah-localization ${DEPENDENCIES})
    target_link_libraries(cheetah-localization lcm)
    target_link_libraries(cheetah-pointcloud ${DEPENDENCIES})
    target_link_libraries(cheetah-pointcloud lcm)

    find_package( OpenCV REQUIRED )
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(cheetah-vision ${OpenCV_LIBS})
    target_link_libraries(cheetah-pointcloud ${OpenCV_LIBS})

    include_directories(cheetah-vision ../)
    include_directories(cheetah-localization ../)
    include_directories(cheetah-pointcloud ../)

    set_target_properties (cheetah-vision PROPERTIES FOLDER Examples)
    set_target_properties (cheetah-localization PROPERTIES FOLDER Examples)
    set_target_properties (cheetah-pointcloud PROPERTIES FOLDER Examples)

    install(TARGETS cheetah-vision RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
