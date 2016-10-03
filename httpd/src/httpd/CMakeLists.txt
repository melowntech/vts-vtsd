# test program
define_module(BINARY httpd
  DEPENDS
  httplib http service

  Boost_SERIALIZATION
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  )

set(httpd_SOURCES
  main.cpp
  )

add_executable(vts-httpd ${httpd_SOURCES} ${httpd_BROWSER_SOURCES})
target_link_libraries(vts-httpd ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vts-httpd ${MODULE_DEFINITIONS})
buildsys_binary(vts-httpd)
set_target_version(vts-httpd ${httpd_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS vts-httpd RUNTIME DESTINATION bin
  COMPONENT main)
