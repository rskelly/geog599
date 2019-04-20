cmake_minimum_required(VERSION 3.1)

enable_language(C)
enable_language(CXX)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_CXX_FLAGS "-fPIC -lpthread -fopenmp -Wno-unknown-pragmas -Wall -D_GLIBCXX_PARALLEL")

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_INSTALL_RPATH "/usr/local/lib")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set (CMAKE_CXX_COMPILER_LAUNCHER ccache)

set (CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}:/usr/lib/x86_64-linux-gnu/cmake/Qt5")

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
find_package (Qt5 REQUIRED Core Widgets Gui)
find_library (FITPACK fitpack) # Fortran lib; see top of this file.

project (test)

# Configure directories###################################################################################

include_directories(BEFORE SYSTEM /opt/dji/include ./include ./src  
	${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDEDIR} ${EIGEN3_INCLUDE_DIR})
	
include_directories(./build/pipeline_autogen/include 
	${Qt5_DIR}/include ${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS})
 
link_directories(./build/bin ${Boost_LIBRARY_DIRS})

set (CMAKE_AUTOUIC ON)
set (CMAKE_AUTOMOC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)
set (CMAKE_AUTOMOC_OPTIONS "-Isrc/ui")

# Build support libraries ################################################################################

enable_language (Fortran)
file (GLOB FSRC src/fitpack/*.f)
add_library (fitpack_mod SHARED ${FSRC}) 

add_library (deboor_smooth SHARED src/deboor/chol1d.f src/deboor/setupq.f src/deboor/smooth.f)
 
add_executable(pipeline src/pipeline.cpp src/ui/profile.cpp)
target_link_libraries (pipeline las deboor_smooth fitpack_mod  Qt5::Gui Qt5::Core Qt5::Widgets)


