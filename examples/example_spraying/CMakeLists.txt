ADD_EXECUTABLE( example_spraying example_spraying.cpp )

set_target_properties(example_spraying PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

target_link_libraries( example_spraying AROLIB_types AROLIB_misc AROLIB_geometry AROLIB_planning AROLIB_components AROLIB_cartography )

# install(TARGETS example_spraying
#         EXPORT example_spraying_targets
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION lib)

add_test(NAME integration/example_spraying COMMAND example_spraying)

