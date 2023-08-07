set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_MISC}")

add_library(${AROLIB_LIB_NAME} SHARED
  src/basic_responses.cpp
  src/logger.cpp
  src/loggingcomponent.cpp
  src/base64Utility.cpp
  src/container_helper.cpp
  src/datetime.cpp
  src/time_helper.cpp
  src/basicconversions.cpp
  src/randomgeneration.cpp
  src/color_helper.cpp
  src/filesystem_helper.cpp
  src/mappable_parameters.cpp
  include/arolib/misc/basic_responses.h
  include/arolib/misc/logger.h
  include/arolib/misc/loggingcomponent.h
  include/arolib/misc/base64Utility.hpp
  include/arolib/misc/container_helper.h
  include/arolib/misc/datetime.hpp
  include/arolib/misc/time_helper.hpp
  include/arolib/misc/basicconversions.hpp
  include/arolib/misc/randomgeneration.hpp
  include/arolib/misc/color_helper.hpp
  include/arolib/misc/filesystem_helper.h
  include/arolib/misc/mappable_parameters.h
)

#set_target_properties(${AROLIB_LIB_NAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

target_include_directories(${AROLIB_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${AROLIB_LIB_NAME}
    PUBLIC
      cryptopp
      ${Boost_LIBRARIES}
)

install(TARGETS ${AROLIB_LIB_NAME}
        EXPORT ${AROLIB_LIB_NAME}_targets
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)

add_subdirectory(test)
