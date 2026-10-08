find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
set(NUSIM_BUILD
    "${NUSIM_ROOT}/mujoco/build-native"
    CACHE PATH "Configured NUSim build with generated IDL"
)
file(GLOB idl_packages "${NUSIM_BUILD}/idl_gen/*")
add_library(nusim_wire STATIC IMPORTED GLOBAL)
set_target_properties(nusim_wire PROPERTIES IMPORTED_LOCATION "${NUSIM_BUILD}/module/SdkBridge/libk1sim_idl.a")
nuclear_module(
  SOURCES "${NUSIM_ROOT}/mujoco/module/SdkBridge/src/DdsParticipant.cpp" LIBRARIES nusim_wire fastrtps fastcdr
)
target_include_directories(inputNUSimPlayer PUBLIC "${NUSIM_ROOT}/mujoco")
foreach(pkg ${idl_packages})
  target_compile_options(inputNUSimPlayer PUBLIC "-iquote${pkg}")
endforeach()
