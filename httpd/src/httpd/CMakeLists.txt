# test program
define_module(BINARY httpd
  DEPENDS
  vts-libs-core http service

  Boost_SERIALIZATION
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  )

set(httpd_SOURCES
  sink.hpp sink.cpp
  fileclass.hpp fileclass.cpp
  config.hpp config.cpp

  delivery/variables.hpp
  delivery/vts/driver.hpp delivery/vts/driver.cpp
  delivery/vts0/driver.hpp delivery/vts0/driver.cpp
  delivery/tileset/driver.hpp delivery/tileset/driver.cpp
  delivery/driver.hpp

  main.cpp
  )

add_executable(vtshttpd ${httpd_SOURCES} ${httpd_BROWSER_SOURCES})
target_link_libraries(vtshttpd ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vtshttpd ${MODULE_DEFINITIONS})
buildsys_binary(vtshttpd)
set_target_version(vtshttpd ${httpd_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS vtshttpd RUNTIME DESTINATION bin
  COMPONENT main)
