find_path(CANLIB_INCLUDE_DIR NAMES canlib.h)
find_library(CANLIB_LIBRARY NAMES canlib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(canlib DEFAULT_MSG CANLIB_LIBRARY CANLIB_INCLUDE_DIR)

if(CANLIB_FOUND AND NOT TARGET canlib)
  add_library(canlib UNKNOWN IMPORTED)
  set_target_properties(canlib PROPERTIES
    IMPORTED_LOCATION "${CANLIB_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CANLIB_INCLUDE_DIR}")
endif()

mark_as_advanced(CANLIB_INCLUDE_DIR CANLIB_LIBRARY)