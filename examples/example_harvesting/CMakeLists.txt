ADD_EXECUTABLE( example_harvesting example_harvesting.cpp )

target_link_libraries( example_harvesting AROLIB_types AROLIB_misc AROLIB_geometry AROLIB_planning AROLIB_components AROLIB_cartography )

# install(TARGETS example_harvesting
#         EXPORT example_harvesting_targets
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION lib)

add_test(NAME integration/example_harvesting COMMAND example_harvesting)

