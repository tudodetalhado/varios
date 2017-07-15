# Create a default-layouted shared library target.
# Usage create_shared_library(lib_foo, Sources, Includes,...)
function(create_imported_shared_library TARGET_NAME DEBUG_DLL DEBUG_LIB RELEASE_DLL RELEASE_LIB)
  if(NOT TARGET ${TARGET_NAME})
    add_library(${TARGET_NAME} SHARED IMPORTED)
    set_property(TARGET ${TARGET_NAME} PROPERTY IMPORTED_LOCATION_DEBUG ${DEBUG_DLL})
    set_property(TARGET ${TARGET_NAME} PROPERTY IMPORTED_IMPLIB_DEBUG ${DEBUG_LIB})
    set_property(TARGET ${TARGET_NAME} PROPERTY IMPORTED_LOCATION_RELEASE ${RELEASE_DLL})
    set_property(TARGET ${TARGET_NAME} PROPERTY IMPORTED_IMPLIB_RELEASE ${RELEASE_LIB})
  endif(NOT TARGET ${TARGET_NAME})
endfunction(create_imported_shared_library)