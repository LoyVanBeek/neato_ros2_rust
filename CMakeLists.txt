cmake_minimum_required(VERSION 3.5)
project(neato_ros2_rust NONE)

find_package(ament_cmake REQUIRED)
find_package(rclrs_common REQUIRED)
find_package(rclrs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(_native_libraries_dirs "")
foreach(_native_library ${rclrs_LIBRARIES})
    get_filename_component(_native_library_dir ${_native_library} DIRECTORY)
    if(NOT _native_library_dir STREQUAL "")
        set(_native_libraries_dirs "${_native_libraries_dirs}\n'-Lnative=${_native_library_dir}',")
    endif()
endforeach()
foreach(_native_library ${rclrs_common_LIBRARIES})
    get_filename_component(_native_library_dir ${_native_library} DIRECTORY)
    if(NOT _native_library_dir STREQUAL "")
        set(_native_libraries_dirs "${_native_libraries_dirs}\n'-Lnative=${_native_library_dir}',")
    endif()
endforeach()
foreach(_native_library ${std_msgs_LIBRARIES})
    get_filename_component(_native_library_dir ${_native_library} DIRECTORY)
    if(NOT _native_library_dir STREQUAL "")
        set(_native_libraries_dirs "${_native_libraries_dirs}\n'-Lnative=${_native_library_dir}',")
    endif()
endforeach()
foreach(_native_library ${sensor_msgs_LIBRARIES})
    get_filename_component(_native_library_dir ${_native_library} DIRECTORY)
    if(NOT _native_library_dir STREQUAL "")
        set(_native_libraries_dirs "${_native_libraries_dirs}\n'-Lnative=${_native_library_dir}',")
    endif()
endforeach()

set(_crates_dependencies "")
foreach(_crate_dependency ${rclrs_CRATES})
    set(_crates_dependencies "${_crates_dependencies}\n[dependencies.rclrs]\npath = '${_crate_dependency}'\n")
endforeach()
foreach(_crate_dependency ${rclrs_common_CRATES})
    set(_crates_dependencies "${_crates_dependencies}\n[dependencies.rclrs_common]\npath = '${_crate_dependency}'\n")
endforeach()
foreach(_crate_dependency ${std_msgs_CRATES})
    set(_crates_dependencies "${_crates_dependencies}\n[dependencies.std_msgs]\npath = '${_crate_dependency}'\n")
endforeach()
foreach(_crate_dependency ${sensor_msgs_CRATES})
    set(_crates_dependencies "${_crates_dependencies}\n[dependencies.sensor_msgs]\npath = '${_crate_dependency}'\n")
endforeach()

include(ExternalProject)

configure_file(${CMAKE_SOURCE_DIR}/src/neato_ros2_rust.rs ${CMAKE_BINARY_DIR}/src/neato_ros2_rust.rs COPYONLY)

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/.cargo")
file(WRITE "${CMAKE_BINARY_DIR}/.cargo/config"
"\
[build]\n\
target-dir = 'ament_cargo/${PROJECT_NAME}/target'\n\
rustflags = [\
${_native_libraries_dirs}
]\n\
"
)

file(COPY "${CMAKE_SOURCE_DIR}/Cargo.toml" DESTINATION "${CMAKE_BINARY_DIR}/")
file(APPEND "${CMAKE_BINARY_DIR}/Cargo.toml" "${_crates_dependencies}")

add_custom_command(
    OUTPUT
        ${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target/release/neato_ros2_rust
    COMMAND cargo build --release --manifest-path "${CMAKE_BINARY_DIR}/Cargo.toml"
    DEPENDS
        ${CMAKE_BINARY_DIR}/src/neato_ros2_rust.rs
)

add_custom_target(
    build_crate ALL
    DEPENDS
        ${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target/release/neato_ros2_rust
)

install(FILES
    ${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target/release/neato_ros2_rust
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()