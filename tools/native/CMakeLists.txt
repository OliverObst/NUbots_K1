find_package(onnxruntime REQUIRED)
add_executable(test_native_portability test_portability.cpp)
target_link_libraries(test_native_portability PRIVATE nuclear::utility)
target_include_directories(test_native_portability PRIVATE "${PROJECT_SOURCE_DIR}/shared")
add_test(NAME test_native_portability COMMAND test_native_portability)

add_executable(check_policy_inference check_policy_inference.cpp)
target_link_libraries(check_policy_inference PRIVATE nuclear::utility onnxruntime::onnxruntime)
set_target_properties(check_policy_inference PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")

add_test(
  NAME test_policy_inference_contract
  COMMAND
    check_policy_inference "${PROJECT_BINARY_DIR}/k1_walk_v3_20260812_102M.onnx"
    "${PROJECT_BINARY_DIR}/k1_kick_v7_20260728_2504M.onnx" "${PROJECT_BINARY_DIR}/k1_getup_v21_20260725_984M.onnx"
)
