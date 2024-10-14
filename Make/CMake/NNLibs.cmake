add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES 
  IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/onnxruntime18/lib/libonnxruntime.so")
target_include_directories(onnxruntime INTERFACE "${BHUMAN_PREFIX}/Util/onnxruntime18/include")
get_target_property(IMPORT_LOC onnxruntime IMPORTED_LOCATION)
if(NOT IMPORT_LOC)
    message(FATAL_ERROR "ONNX Runtime not found")
endif()
