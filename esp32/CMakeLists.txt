cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if($ENV{ENV_RELEASE_BUILD})
    set(BASE_VER "rel")
else()
    set(BASE_VER "dev")
endif()

execute_process(
    COMMAND git describe --tags --dirty --always --long
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(GIT_DESCRIBE)
    set(PROJECT_VER "${BASE_VER}-${GIT_DESCRIBE}")
else()
    set(PROJECT_VER "${BASE_VER}")
endif()

project(treat_dispenser)

# Copy binary with version in filename
add_custom_target(versioned_binary ALL
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/treat_dispenser.bin ${CMAKE_BINARY_DIR}/treat_dispenser-${PROJECT_VER}.bin
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/treat_dispenser.elf ${CMAKE_BINARY_DIR}/treat_dispenser-${PROJECT_VER}.elf
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/treat_dispenser.map ${CMAKE_BINARY_DIR}/treat_dispenser-${PROJECT_VER}.map
    DEPENDS app
    COMMENT "Creating versioned binaries: treat_dispenser-${PROJECT_VER}.[bin|elf|map]"
)
