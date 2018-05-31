cmake_minimum_required(VERSION 3.1)

enable_language(C)
enable_language(CXX)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread -fopenmp -Wno-unknown-pragmas -Wall -D_GLIBCXX_PARALLEL")

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_INSTALL_RPATH "/opt/lib;/usr/local/lib")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set (CMAKE_CXX_COMPILER_LAUNCHER ccache)

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
#find_package (Boost 1.58 COMPONENTS system filesystem date_time thread REQUIRED)
find_package (GDAL 2.2 REQUIRED)
find_package (CGAL REQUIRED COMPONENTS Core)
#find_package (PCL 1.8 REQUIRED)
#find_package (OpenCV REQUIRED)

#find_library (LIBLAS_LIBRARY las)
#find_library (JSONCPP_LIBRARY jsoncpp)
#find_library (CRYPTO_LIBRARY crypto libcrypto PATHS ${CRYPTO_LIBRARYDIR})
#find_library (GEOS_LIBRARY geos PATHS ${GEOS_LIBRARYDIR})
#find_library (PROJ_LIBRARY proj)
find_library (GMP_LIBRARY gmp)
#find_library (GLFW_LIBRARY glfw)
#find_library (GL_LIBRARY GL)
#find_library (GLEW_LIBRARY GLEW)

project (test)

# Configure directories###################################################################################

include_directories(BEFORE SYSTEM ./include 
	${GEOS_INCLUDEDIR} ${OSGEO_INCLUDEDIR} ${GDAL_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS} ${CRYPTO_INCLUDEDIR} ${EIGEN_INCLUDEDIR} ${JSON_INCLUDEDIR}) 
link_directories(. ${Boost_LIBRARY_DIRS} ${GDAL_LIBRARYDIR} ${PCL_LIBRARY_DIRS})

# Build support libraries ################################################################################

add_executable (sim1 src/sim1.cpp src/sim/platform.cpp src/sim/rangefinder.cpp src/util.cpp src/sim/terrain.cpp src/sim/gimbal.cpp)
target_link_libraries (sim1 ${GMP_LIBRARY} ${GDAL_LIBRARY} ${CGAL_LIBRARY} ${GL_LIBRARY}) # ${GLFW_LIBRARY} ${GLEW_LIBRARY})
