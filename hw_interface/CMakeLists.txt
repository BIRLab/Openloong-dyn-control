find_package(PkgConfig REQUIRED)

pkg_check_modules(LIB_LELY liblely-coapp)

set(CONFIG_OUT_DIR "${CMAKE_BINARY_DIR}/../config")

add_custom_command(
    OUTPUT "${CONFIG_OUT_DIR}/master.dcf"
    DEPENDS "${CMAKE_CURRENT_LIST_DIR}/config/config.yml" "${CMAKE_CURRENT_LIST_DIR}/config/zeroerr.eds" "${CMAKE_CURRENT_LIST_DIR}/scripts/generate_config.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/config"
    COMMAND mkdir -p "${CONFIG_OUT_DIR}"
    COMMAND python3 "${CMAKE_CURRENT_LIST_DIR}/scripts/generate_config.py" "${CMAKE_CURRENT_LIST_DIR}/config/config.yml" "${CONFIG_OUT_DIR}"
    COMMAND dcfgen --remote-pdo --no-strict --directory "${CONFIG_OUT_DIR}" "${CONFIG_OUT_DIR}/bus.yml"
)

add_custom_target(dcf_file
    DEPENDS "${CONFIG_OUT_DIR}/master.dcf"
)

aux_source_directory(src HW_INTERFACE_SRCS)
add_library(hw_interface ${HW_INTERFACE_SRCS})
add_dependencies(hw_interface dcf_file)
target_include_directories(hw_interface PUBLIC include)
target_include_directories(hw_interface PRIVATE ${LIB_LELY_INCLUDE_DIRS} "${CONFIG_OUT_DIR}")
target_compile_definitions(hw_interface PRIVATE ${LIB_LELY_CFLAGS_OTHER})
target_link_libraries(hw_interface PRIVATE ${LIB_LELY_LIBRARIES})

install(
    FILES "${CONFIG_OUT_DIR}/master.dcf"
    DESTINATION config
)

install(
    DIRECTORY "${CONFIG_OUT_DIR}"
    DESTINATION "."
    FILES_MATCHING PATTERN "*.bin"
)
