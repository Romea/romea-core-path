get_filename_component(TEST_WITH_DATA_TEST_DIR "data" ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_add_gtest(${PROJECT_NAME}_test_path test_path.cpp )
target_link_libraries(${PROJECT_NAME}_test_path ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_test_path_matching test_path_matching.cpp )
target_link_libraries(${PROJECT_NAME}_test_path_matching ${PROJECT_NAME})


