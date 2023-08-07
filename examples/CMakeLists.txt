project( arolib_examples )

add_subdirectory( example_harvesting )
add_subdirectory( example_spraying )

# target_link_libraries( ${PROJECT_NAME} AROLIB_types AROLIB_io AROLIB_geometry AROLIB_planning AROLIB_components )

# install(TARGETS arolib_example
#         EXPORT arolib_example_targets
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION lib)
