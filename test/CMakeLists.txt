find_package(GTest REQUIRED)

get_filename_component(TEST_WITH_DATA_TEST_DIR "data" ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(${PROJECT_NAME}_test_path test_path.cpp )
target_link_libraries(${PROJECT_NAME}_test_path ${PROJECT_NAME} ${catkin_LIBRARIES} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_path PRIVATE -std=c++17)
add_test(test_path ${PROJECT_NAME}_test_path)

add_executable(${PROJECT_NAME}_test_path_matching test_path_matching.cpp )
target_link_libraries(${PROJECT_NAME}_test_path_matching ${PROJECT_NAME} ${catkin_LIBRARIES} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_path_matching PRIVATE -std=c++17)
add_test(test_path_matching ${PROJECT_NAME}_test_path_matching)


