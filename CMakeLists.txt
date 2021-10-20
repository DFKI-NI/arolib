# CMake required version
cmake_minimum_required(VERSION 3.2.0)
#cmake_minimum_required(VERSION 2.8.7)

enable_testing()

# Project name
project (AroLib VERSION 0.1.0 LANGUAGES CXX)

message(STATUS "CMAKE_C_COMPILER = ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER = ${CMAKE_CXX_COMPILER}")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type  -pthread") # -pthread option for DEBUG

# Appends the cmake/modules path inside the MAKE_MODULE_PATH variable which stores the
# directories of additional CMake modules (eg MacroOutOfSourceBuild.cmake):
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})


SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib )
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../obj )
SET(INCLUDE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/include)
SET(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
SET(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
SET(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

foreach(p LIB BIN INCLUDE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    message(STATUS "-> ${var}: ${${var}}")
  endif()
endforeach()

file(MAKE_DIRECTORY ${INCLUDE_OUTPUT_PATH})
#include_directories(${INCLUDE_OUTPUT_PATH})

# Need to enable c compilation for hdf5 cmake tests
ENABLE_LANGUAGE(C)

# -------------- Boost -------------- #
find_package(Boost 1.53 REQUIRED COMPONENTS system filesystem unit_test_framework)
include_directories(${Boost_INCLUDE_DIRS})
message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")

# -------------- HDF5 -------------- #
include_directories(3rdparty/HighFive/include)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
include_directories(${HDF5_INCLUDE_DIRS})
link_directories(${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES})
message(STATUS "HDF5_FOUND = ${HDF5_FOUND}")

# -------------- GDAL -------------- #
set(GDAL_CONFIG GDAL_CONFIG-NOTFOUND)
set(GDAL_FIND_QUIETLY TRUE)
include(gdal)
if(NOT GDAL_FOUND)
    message(FATAL_ERROR "GDAL NOT FOUND")
endif()
include_directories(${GDAL_INCLUDE_DIR})
#FindGDAL()

# --------------------------------------- #

set(AROLIB_LIB_PREFIX "AROLIB_")
set(AROLIB_LIB_ANALYSIS "analysis")
set(AROLIB_LIB_CARTOGRAPHY "cartography")
set(AROLIB_LIB_COMPONENTS "components")
set(AROLIB_LIB_GEOMETRY "geometry")
set(AROLIB_LIB_IO "io")
set(AROLIB_LIB_MISC "misc")
set(AROLIB_LIB_PLANNING "planning")
set(AROLIB_LIB_PROCESSING "processing")
set(AROLIB_LIB_TYPES "types")

## any way to do this nicer??
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/types/include)
include_directories(${CMAKE_SOURCE_DIR}/misc/include)
include_directories(${CMAKE_SOURCE_DIR}/io/include)
include_directories(${CMAKE_SOURCE_DIR}/geometry/include)
include_directories(${CMAKE_SOURCE_DIR}/planning/include)
include_directories(${CMAKE_SOURCE_DIR}/processing/include)
include_directories(${CMAKE_SOURCE_DIR}/cartography/include)
include_directories(${CMAKE_SOURCE_DIR}/communication/include)
include_directories(${CMAKE_SOURCE_DIR}/communication/protobuf/build/cpp)
include_directories(${CMAKE_SOURCE_DIR}/components/include)
include_directories(${CMAKE_SOURCE_DIR}/analysis/include)

add_subdirectory(types)
add_subdirectory(misc)
add_subdirectory(geometry)
add_subdirectory(cartography)
add_subdirectory(planning)
add_subdirectory(processing)
add_subdirectory(io)
add_subdirectory(components)
add_subdirectory(analysis)

#for debug
message(STATUS "INCLUDE DIRECTORIES:")
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "   '${dir}'")
endforeach()

##-------------------- arolib package -------------------##

set (AROLIB_BASE_NAMESPACE "arolib::")
set(AROLIB_LIB_NAMES
    ${AROLIB_LIB_ANALYSIS}
    ${AROLIB_LIB_CARTOGRAPHY}
    ${AROLIB_LIB_COMPONENTS}
    ${AROLIB_LIB_GEOMETRY}
    ${AROLIB_LIB_IO}
    ${AROLIB_LIB_MISC}
    ${AROLIB_LIB_PLANNING}
    ${AROLIB_LIB_PROCESSING}
    ${AROLIB_LIB_TYPES})
MESSAGE(STATUS "AROLIB_LIB_NAMES = ${AROLIB_LIB_NAMES}")

# Include module with fuction 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

# install /.../include/*.hpp

# === Install the include headers ===
foreach(libname ${AROLIB_LIB_NAMES})
    install(
        DIRECTORY ${CMAKE_SOURCE_DIR}/${libname}/include/arolib/${libname}/
        DESTINATION include/arolib/${libname}
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.icc" PATTERN "*.tcc" PATTERN "*.inl" PATTERN "LICENSE*" PATTERN "License*" PATTERN "license*"
    )
endforeach()

# === Install the HighFive include headers ===
install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/3rdparty/HighFive/include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.icc"
)

foreach(libname ${AROLIB_LIB_NAMES})
    list( APPEND TARGETS_EXPORT_NAMES ${AROLIB_LIB_PREFIX}${libname}_targets )
    list( APPEND LIBS_EXPORT_ALIASES ${AROLIB_BASE_NAMESPACE}${libname}::${AROLIB_LIB_PREFIX}${libname} )
endforeach()
MESSAGE(STATUS "TARGETS_EXPORT_NAMES = ${TARGETS_EXPORT_NAMES}")

# Configure '<PROJECT-NAME>Config.cmake'
# Use variables:
#   * TARGETS_EXPORT_NAME
#   * PROJECT_NAME
configure_package_config_file(
    "cmake/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

# generate arolibConfigVersion.cmake
write_basic_package_version_file("${PROJECT_NAME}ConfigVersion.cmake"
        VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)

# install arolibConfigVersion.cmake, arolibConfig.cmake

# === Export CMake Project ===
# EXPORT: generates and installs a file containing code to import targets from the installation tree into another CMAKE project
foreach(libname ${AROLIB_LIB_NAMES})
    install(
        EXPORT ${AROLIB_LIB_PREFIX}${libname}_targets
        NAMESPACE ${AROLIB_BASE_NAMESPACE}${libname}::
        DESTINATION lib/cmake/${PROJECT_NAME}
        COMPONENT ${libname}
    )
endforeach()


# install <project>ConfigVersion.cmake and <project>Config.cmake
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# === Compile the module with examples ===
add_subdirectory(examples)
