find_package(PkgConfig REQUIRED)

pkg_check_modules(LIB_LELY liblely-coapp)

add_custom_command(
    OUTPUT "${CMAKE_BINARY_DIR}/config/master.dcf"
    DEPENDS "${CMAKE_CURRENT_LIST_DIR}/config/config.yml" "${CMAKE_CURRENT_LIST_DIR}/config/zeroerr.eds"
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/config"
    COMMAND mkdir -p "${CMAKE_BINARY_DIR}/config"
    COMMAND python3 "${CMAKE_CURRENT_LIST_DIR}/scripts/generate_config.py" "${CMAKE_CURRENT_LIST_DIR}/config/config.yml" "${CMAKE_BINARY_DIR}/config"
    COMMAND dcfgen --remote-pdo --no-strict --directory "${CMAKE_BINARY_DIR}/config" "${CMAKE_BINARY_DIR}/config/bus.yml"
)

add_custom_target(dcf_file
    DEPENDS "${CMAKE_BINARY_DIR}/config/master.dcf"
)

aux_source_directory(src HW_INTERFACE_SRCS)
add_library(hw_interface ${HW_INTERFACE_SRCS})
add_dependencies(hw_interface dcf_file)
target_include_directories(hw_interface PUBLIC include)
target_include_directories(hw_interface PRIVATE ${LIB_LELY_INCLUDE_DIRS} "${CMAKE_BINARY_DIR}/config")
target_compile_definitions(hw_interface PRIVATE ${LIB_LELY_CFLAGS_OTHER})
target_link_libraries(hw_interface PRIVATE ${LIB_LELY_LIBRARIES})

install(
    FILES "${CMAKE_BINARY_DIR}/config/master.dcf" "${CMAKE_BINARY_DIR}/config/99-candlelight.rules"
    DESTINATION config
)

install(
    DIRECTORY "${CMAKE_BINARY_DIR}/config"
    DESTINATION "."
    FILES_MATCHING PATTERN "*.bin"
)

install(
    PROGRAMS "${CMAKE_CURRENT_LIST_DIR}/scripts/setup_can.sh"
    DESTINATION scripts
)
