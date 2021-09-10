set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_CARTOGRAPHY}")

add_library(${AROLIB_LIB_NAME} SHARED
    src/gridmap_layout.cpp
    src/gridmap_rgba32.cpp
    src/cellsrange.cpp
    src/cellsrangeset.cpp
    src/gridcellsinfomanager.cpp
    src/sharedgridsmanager.cpp
    include/arolib/cartography/gridmap_layout.hpp
    include/arolib/cartography/gridmap.hpp
    include/arolib/cartography/gridmap_numeric.hpp
    include/arolib/cartography/gridmap_rgba32.hpp
    include/arolib/cartography/cellsrange.hpp
    include/arolib/cartography/cellsrangeset.hpp
    include/arolib/cartography/common.hpp
    include/arolib/cartography/gridcellsinfomanager.hpp
    include/arolib/cartography/sharedgridsmanager.hpp
    include/arolib/cartography/impl/gridmap.tcc
    include/arolib/cartography/impl/gridmap_numeric.tcc
)

#set_property(TARGET ${AROLIB_LIB_CARTOGRAPHY} PROPERTY CXX_STANDARD 11)  # 98, 11 or 14
#set_property(TARGET ${AROLIB_LIB_CARTOGRAPHY} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${AROLIB_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    INTERFACE
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${AROLIB_LIB_NAME}
    INTERFACE
      png
      ${GDAL_LIBRARY}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_TYPES}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_GEOMETRY}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_MISC}
)

install(TARGETS ${AROLIB_LIB_NAME}
        EXPORT ${AROLIB_LIB_NAME}_targets
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)

#    if(NOT GDAL_FOUND)
#        #find GDAL
#        set(GDAL_CONFIG GDAL_CONFIG-NOTFOUND)
#        set(GDAL_FIND_QUIETLY TRUE)
#        include(${CMAKE_CURRENT_LIST_DIR}/../gdal.cmake)
#        if(NOT GDAL_FOUND)
#            message(FATAL_ERROR "GDAL NOT FOUND")
#        endif()
#        message(STATUS "GDAL included!")
#        #FindGDAL()
#    endif()
#    include_directories(${GDAL_INCLUDE_DIR})

add_subdirectory(test)
