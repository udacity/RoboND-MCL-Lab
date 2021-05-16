cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(robotND)

#setPaths
set(ROBOT_MCL_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/robotMcl)
set(TEST_CASES_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/testCases)
set(CONFIG_INCLIDE_DIR ${PROJECT_SOURCE_DIR}/src/_generated)

message(${ROBOT_MCL_INCLIDE_DIR})
# configure a header file to pass some of the CMake settings
# to the source code
make_directory("${CONFIG_INCLIDE_DIR}")
configure_file ("${PROJECT_SOURCE_DIR}/PfConfig.h.in" 
                "${CONFIG_INCLIDE_DIR}/PfConfig.h")

#set Header cfg
set(HEADER_FILES ${PROJECT_SOURCE_DIR}/src/_generated/PfConfig.h)
set(HEADER_FILES_INCLUDE_DIRS  ${CONFIG_INCLIDE_DIR})

# Unit testing mode or Unit testing
option (USE_UNIT_TESTING "Use Unit testing Mode" OFF)

if(USE_UNIT_TESTING)
    message("Use unit testing Mode")
    
    #Find Gtest lib and include the directories
    find_package(GTest REQUIRED)
    #Enable Testing
    enable_testing()

    #Add robotMcl sub-directory
    add_subdirectory(src/robotMcl)
    list(APPEND HEADER_FILES_INCLUDE_DIRS  "${ROBOT_MCL_INCLIDE_DIR}")

    #Add robotMcl sub-directory
    add_subdirectory(src/testCases)
    list(APPEND EXTRA_INCLUDES "${TEST_CASES_INCLIDE_DIR}")

    #compile the source code and link it with external libraries
    add_executable(gtest src/main_test.cpp)
    target_include_directories(gtest PUBLIC ${EXTRA_INCLUDES})
    target_link_libraries(gtest PUBLIC testCases)
    target_link_libraries(gtest PUBLIC robotMcl)
    target_link_libraries(gtest PUBLIC GTest::gtest GTest::gtest_main)

    #add the testing
    add_test(NAME gtest COMMAND gtest)

else()
    message("MCL-APP")

    #find Matplot package
    find_package(Matplot++ REQUIRED)

    #for windows its required to prvode the include folder
    if(WIN32)
    set(MATPLOT_INCLUDE_DIR ${Matplot++_DIR}/../../../include)
    list(APPEND HEADER_FILES "${MATPLOT_INCLUDE_DIR}/matplot/matplot.h")
    list(APPEND HEADER_FILES_INCLUDE_DIRS  "${MATPLOT_INCLUDE_DIR}")
    endif()

    #Add robotMcl sub-directory
    add_subdirectory(src/robotMcl)
    list(APPEND EXTRA_INCLUDES "${ROBOT_MCL_INCLIDE_DIR}")

    #compile the source code and link it with external libraries
    add_executable(${PROJECT_NAME} src/main.cpp)
    target_include_directories(${PROJECT_NAME} PUBLIC ${EXTRA_INCLUDES})
    target_link_libraries(${PROJECT_NAME} PUBLIC robotMcl)
    target_link_libraries(${PROJECT_NAME} PUBLIC Matplot++::matplot)
endif()




     
