cmake_minimum_required(VERSION 3.1)

execute_process (COMMAND gfortran -g -shared -fPIC ../src/fitpack/fpcurf.f ../src/fitpack/fpback.f 
	../src/fitpack/fpbspl.f ../src/fitpack/fpgivs.f ../src/fitpack/fpdisc.f ../src/fitpack/fprati.f 
	../src/fitpack/fpknot.f ../src/fitpack/fprota.f ../src/fitpack/splev.f ../src/fitpack/splder.f 
	-o bin/libfitpack_mod.so) 

set (WITH_GUI true CACHE BOOL "If true, will build the Qt GUI.")

if(${WITH_GUI})
	add_definitions(-DWITH_GUI)
endif()

enable_language(C)
enable_language(CXX)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)
#set (CMAKE_POSITION_INDEPENDENT_CODE ON) # Qt complains if this is set and not -fPIC below...
set (CMAKE_CXX_FLAGS "-fPIC -lpthread -fopenmp -Wno-unknown-pragmas -Wall -D_GLIBCXX_PARALLEL")

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_INSTALL_RPATH "/opt/lib;/usr/local/lib")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set (CMAKE_CXX_COMPILER_LAUNCHER ccache)

if(${WITH_GUI})
	set (CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}:/usr/lib/x86_64-linux-gnu/cmake/Qt5")
endif()

set (CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/dji")

set (Boost_DEBUG OFF)
set (Boost_USE_STATIC_LIBS OFF)
set (Boost_USE_STATIC_RUNTIME OFF)
set (Boost_NO_SYSTEM_PATHS ON)
set (Boost_USE_MULTITHREADED ON)

set (EIGEN_INCLUDEDIR "/usr/include/eigen3")
set (BOOST_INCLUDEDIR "/usr/include")
set (BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")
set (JSON_INCLUDEDIR "/usr/include/jsoncpp")

set (THREADS_PREFER_PTHREAD_FLAG ON)

find_package (Threads REQUIRED)
find_package (Eigen3 REQUIRED)
#find_package (Boost 1.58 COMPONENTS system filesystem date_time thread REQUIRED)
#find_package (GDAL 2.2 REQUIRED)
#find_package (CGAL REQUIRED COMPONENTS Core)
#find_package (PCL 1.8 REQUIRED)
#find_package (OpenCV REQUIRED)
find_package (DJIOSDK REQUIRED)
#if(${WITH_GUI})
#	find_package(Qt5 REQUIRED Core Widgets Gui)
#endif()

#find_library (LIBLAS_LIBRARY las)
#find_library (JSONCPP_LIBRARY jsoncpp)
#find_library (CRYPTO_LIBRARY crypto libcrypto PATHS ${CRYPTO_LIBRARYDIR})
#find_library (GEOS_LIBRARY geos PATHS ${GEOS_LIBRARYDIR})
#find_library (PROJ_LIBRARY proj)
#find_library (GMP_LIBRARY gmp)
#find_library (GLFW_LIBRARY glfw)
#find_library (GL_LIBRARY GL)
#find_library (GLEW_LIBRARY GLEW)
#find_library (GLU_LIBRARY GLU)
#find_library (I2C_LIBRARY i2c)
find_library (FITPACK fitpack)

project (test)

# Configure directories###################################################################################

include_directories(BEFORE SYSTEM ./include ./src /opt/dji/include
	${GEOS_INCLUDEDIR} ${OSGEO_INCLUDEDIR} ${GDAL_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS} ${CRYPTO_INCLUDEDIR} ${EIGEN_INCLUDEDIR} ${JSON_INCLUDEDIR} ${EIGEN3_INCLUDE_DIR})
	
if(${WITH_GUI})
	include_directories(./build/sim1_autogen/include /opt/dji/lib 
		${Qt5_DIR}/include ${Qt5Core_INCLUDE_DIRS} 
		${Qt5Widgets_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS})
endif()
 
link_directories(. build/bin ${Boost_LIBRARY_DIRS} ${GDAL_LIBRARYDIR} ${PCL_LIBRARY_DIRS})

if(${WITH_GUI})
	set (CMAKE_AUTOUIC ON)
	set (CMAKE_AUTOMOC ON)
	set (CMAKE_INCLUDE_CURRENT_DIR ON)
	set (CMAKE_AUTOMOC_OPTIONS "-Isrc/viewer")
endif()

# Build support libraries ################################################################################

 
add_library (uav_serial SHARED src/comm/serial.cpp)

add_library (uav_i2c SHARED src/comm/i2c.cpp)
if (I2C_LIBRARY)
	target_link_libraries (uav_i2c i2c)
endif()

#add_library (uav_imu SHARED src/sensor/minimu9v5.cpp)
#target_link_libraries (uav_imu uav_i2c)

#add_library (uav_sf30c SHARED src/sensor/lwsf30c.cpp)
#target_link_libraries (uav_sf30c uav_serial)

add_library (uav_teensy SHARED src/sensor/teensy.cpp)
target_link_libraries (uav_teensy uav_serial)

#add_library (uav_encoder SHARED src/sensor/ads1115.cpp)
#target_link_libraries (uav_encoder uav_i2c)
 
#add_executable (controller src/controller.cpp src/platform/djiplatform.cpp)
#target_link_libraries (controller uav_teensy djiosdk-core) # uav_imu uav_teensy uav_encoder)

add_executable(pipeline src/pipeline.cpp)
target_link_libraries (pipeline las fitpack_mod)

add_executable (curve src/curve.cpp)
target_link_libraries (curve fitpack_mod)


