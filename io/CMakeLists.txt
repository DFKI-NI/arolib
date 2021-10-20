set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_IO}")

add_library(${AROLIB_LIB_NAME} SHARED
    src/io_hdf5.cpp
    src/io_xml.cpp
    src/io_kml.cpp
    src/io_common.cpp
    src/arodocument.cpp
    src/arotagsmanager.cpp
    src/xmlindocument.cpp
    src/xmloutdocument.cpp
    src/aroxmlindocument.cpp
    src/aroxmloutdocument.cpp
    src/arokmlindocument.cpp
    src/arokmloutdocument.cpp
    include/arolib/io/io_xml.hpp
    include/arolib/io/io_kml.hpp
    include/arolib/io/io_hdf5.hpp
    include/arolib/io/io_common.hpp
    include/arolib/io/arodocument.hpp
    include/arolib/io/arotagsmanager.hpp
    include/arolib/io/xmlindocument.hpp
    include/arolib/io/xmloutdocument.hpp
    include/arolib/io/aroxmlindocument.hpp
    include/arolib/io/aroxmloutdocument.hpp
    include/arolib/io/kmltags.hpp
    include/arolib/io/arokmlindocument.hpp
    include/arolib/io/arokmloutdocument.hpp
)

#set_property(TARGET ${AROLIB_LIB_PREFIX}io PROPERTY CXX_STANDARD 11)  # 98, 11 or 14
#set_property(TARGET ${AROLIB_LIB_PREFIX}io PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${AROLIB_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${AROLIB_LIB_NAME}
    INTERFACE
        ${Boost_LIBRARIES}
        ${AROLIB_LIB_PREFIX}${AROLIB_LIB_TYPES}
        ${AROLIB_LIB_PREFIX}${AROLIB_LIB_GEOMETRY}
        ${AROLIB_LIB_PREFIX}${AROLIB_LIB_PLANNING}
        ${AROLIB_LIB_PREFIX}${AROLIB_LIB_MISC}
)


install(TARGETS ${AROLIB_LIB_NAME}
        EXPORT ${AROLIB_LIB_NAME}_targets
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)

add_subdirectory(test)
