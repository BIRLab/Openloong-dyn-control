aux_source_directory(src HW_INTERFACE_SRCS)
add_library(hw_interface ${HW_INTERFACE_SRCS})
target_include_directories(hw_interface PUBLIC include)
