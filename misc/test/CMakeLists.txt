# Add new test suites to this list
##################################
set(test_suits test_mock test_mappable_parameters test_basicconversions)
##################################

set(test_module unittest_${AROLIB_LIB_NAME})


add_executable(${test_module} main.cpp)
target_link_libraries(${test_module} ${Boost_LIBRARIES} AROLIB_misc)

foreach(test_suit ${test_suits})
    add_test(NAME ${test_module}/${test_suit} COMMAND ${test_module} -t ${test_suit})
endforeach()
