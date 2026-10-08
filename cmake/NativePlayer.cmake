# Native simulator players use portable dependencies and CPU inference.
set(NUCLEAR_LINK_TYPE
    STATIC
    CACHE STRING "Link native player statically" FORCE
)
enable_testing()
list(PREPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/build-deps/install")
list(APPEND CMAKE_INSTALL_RPATH "${PROJECT_SOURCE_DIR}/build-deps/install/lib")
if(APPLE)
  # Darwin exposes modern IPv6 packet-info options only with RFC 3542 enabled.
  add_compile_definitions(__APPLE_USE_RFC_3542)
  list(APPEND CMAKE_PREFIX_PATH /opt/homebrew /usr/local)
endif()
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp AND NOT TARGET yaml-cpp)
  add_library(yaml-cpp ALIAS yaml-cpp::yaml-cpp)
endif()

if(NUBOTS_NATIVE_PLAYER)
  add_compile_definitions(NUBOTS_NATIVE_PLAYER)
  set(NUCLEAR_ROLES_DIR
      "roles/native-player"
      CACHE PATH "Native player roles" FORCE
  )
  if(NOT NUSIM_ROOT)
    get_filename_component(NUSIM_ROOT "${PROJECT_SOURCE_DIR}/../NUSim" ABSOLUTE)
  endif()
  list(PREPEND CMAKE_PREFIX_PATH "${NUSIM_ROOT}/mujoco/.deps/install")
  list(APPEND CMAKE_INSTALL_RPATH "${NUSIM_ROOT}/mujoco/.deps/install/lib")
endif()
