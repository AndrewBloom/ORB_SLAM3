cmake_minimum_required(VERSION 3.8)
project(OrbSlam3)

if (ANDROID_NDK)
    set(CMAKE_FIND_ROOT_PATH "${CMAKE_FIND_ROOT_PATH};/")
# this is the right way to link to boost
    set(CMAKE_FIND_DEBUG_MODE FALSE)
    set(Boost_DEBUG 0)
    set(BOOST_ROOT ${BOOST_PATH}/${CMAKE_ANDROID_ARCH_ABI})
    message("LOOKING FOR BOOST ROOT IN " ${BOOST_ROOT})
    set(BOOST_INCLUDEDIR ${BOOST_PATH}/${CMAKE_ANDROID_ARCH_ABI}/include/boost-1_82)
    set(BOOST_LIBRARYDIR ${BOOST_PATH}/${CMAKE_ANDROID_ARCH_ABI}/lib)
    set(Boost_LIB_PREFIX lib)

    set(Boost_ADDITIONAL_VERSIONS "1.82.0" "1.82")
    set(Boost_NO_SYSTEM_PATHS TRUE)
    set(Boost_COMPILER "-clang-darwin")
    set(Boost_USE_STATIC_LIBS        ON)
    set(Boost_USE_MULTITHREADED      ON)
    set(Boost_USE_STATIC_RUNTIME    OFF)
    find_package( Boost 1.82.0 COMPONENTS serialization )

    if(Boost_FOUND)
        message("Boost_FOUND is True")
        include_directories(${Boost_INCLUDE_DIRS})
    else()
        message(FATAL_ERROR "Boost_FOUND is False")
    endif()
endif()



LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

if (ANDROID_NDK)
find_package(Eigen3 3.1.0 REQUIRED PATHS ${Eigen3_BINARY_DIR})
if(Eigen3_FOUND)
    message("Eigen3_FOUND is True")
    message("${CMAKE_SOURCE_DIR}/eigen")
    include_directories("${CMAKE_SOURCE_DIR}/eigen")
endif(Eigen3_FOUND)
else(ANDROID_NDK)
    find_package(Eigen3 3.1.0 REQUIRED)
    find_package(Pangolin REQUIRED)
endif(ANDROID_NDK)
find_package(realsense2)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

if (ANDROID_NDK)
    include_directories( ${OPENSSL_INCLUDE_DIR} )
endif()

add_subdirectory(Thirdparty/DBoW2 Thirdparty/DBoW2/build)
add_subdirectory(Thirdparty/g2o Thirdparty/g2o/build)
add_subdirectory(Thirdparty/Sophus Thirdparty/Sophus/build)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++11 or C++0x support
if (NOT ANDROID_NDK)
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    if (COMPILER_SUPPORTS_CXX14)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
        add_definitions(-DCOMPILEDWITHC14)
        message(STATUS "Using flag -std=c++14.")
    elseif (COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
        add_definitions(-DCOMPILEDWITHC11)
        message(STATUS "Using flag -std=c++11.")
    else ()
        message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
    endif ()
else (NOT ANDROID_NDK)
    # android should use c++17 flag from gradle
    add_definitions(-DCOMPILEDWITHC14)
endif (NOT ANDROID_NDK)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if (NOT ANDROID_NDK)
    add_definitions(-DPANGOLIN_UI)
endif()

add_library(${PROJECT_NAME} SHARED
src/OrbSlam3.cpp
src/FeaturesDrawer.cpp
src/System.cc
src/SystemUI.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc

src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
        include/public/OrbSlam3.h
include/System.h
include/SystemUI.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h)

if(ANDROID_NDK)
target_sources(${PROJECT_NAME}
PRIVATE
include/DummyFrameDrawer.h
include/DummyMapDrawer.h
include/DummyViewer.h
)
else()
target_sources(${PROJECT_NAME}
PRIVATE
src/FrameDrawer.cc
src/MapDrawer.cc
src/Viewer.cc
include/FrameDrawer.h
include/MapDrawer.h
include/Viewer.h
)
endif()

if (ANDROID_NDK)
#[[    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
    set(CMAKE_FIND_DEBUG_MODE TRUE)
    find_library(lib_boost_serialisation
            NAMES libboost_serialization-clang-darwin-mt-a64-1_82.a
            PATHS ${BOOST4ANDROID}/${CMAKE_ANDROID_ARCH_ABI}/lib)]]

    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

    message("LOOKING FOR OPEN_SSL LIBRARY IN " ${OPENSSL_LIB_DIR}/${CMAKE_ANDROID_ARCH_ABI})
    find_library(lib_crypto NAMES crypto
            HINTS ${OPENSSL_LIB_DIR}/${CMAKE_ANDROID_ARCH_ABI})
    set(lib_dbow2 DBoW2)
else()
    set(lib_boost_serialisation
            -lboost_serialisation)
    set(lib_crypto
            -lcrypto)
    set(lib_dbow2 ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)
endif()

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${lib_dbow2}
${g2o_LIBRARY_OUTPUT_DIRECTORY}/libg2o.so
${lib_boost_serialisation}
${lib_crypto}
)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()

if (NOT ANDROID_NDK)
    add_subdirectory(Examples Examples/build)
endif()

