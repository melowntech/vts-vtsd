# vtsd - vts delivery daemon
define_module(BINARY vtsd
  DEPENDS
  vts-libs-core>=2.4 http service

  Boost_SERIALIZATION
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  )

set(vtsd_SOURCES
  sink.hpp sink.cpp
  fileclass.hpp fileclass.cpp
  config.hpp config.cpp

  delivery/vts/driver.hpp delivery/vts/driver.cpp
  delivery/vts0/driver.hpp delivery/vts0/driver.cpp
  delivery/tileset/driver.hpp delivery/tileset/driver.cpp
  delivery/driver.hpp
  delivery/cache.hpp delivery/cache.cpp

  main.cpp
  )

add_executable(vtsd ${vtsd_SOURCES})
target_link_libraries(vtsd ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vtsd ${MODULE_DEFINITIONS})
buildsys_binary(vtsd)
set_target_version(vtsd ${vts-vtsd_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS vtsd RUNTIME DESTINATION bin
  COMPONENT main)
