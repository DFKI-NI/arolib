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

#set_target_properties(${AROLIB_LIB_NAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

target_include_directories(${AROLIB_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    INTERFACE
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${AROLIB_LIB_NAME}
  PUBLIC
    png
    ${AROLIB_LIB_PREFIX}${AROLIB_LIB_TYPES}
    ${AROLIB_LIB_PREFIX}${AROLIB_LIB_MISC}
    ${GDAL_LIBRARY}
    ${AROLIB_LIB_PREFIX}${AROLIB_LIB_GEOMETRY}
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
