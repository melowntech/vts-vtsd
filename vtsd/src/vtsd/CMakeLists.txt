
# vtsd - vts delivery daemon
define_module(BINARY vtsd
  DEPENDS
  vts-libs-core>=2.4 http>=1.3 service>=1.7
  slpk>=1.2 imgproc>=1.23

  Boost_SERIALIZATION
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  )

# common stuff in one library
set(vtsd-internals_SOURCES
  sink.hpp sink.cpp
  fileclass.hpp fileclass.cpp
  config.hpp config.cpp

  delivery/cache.hpp delivery/cache.cpp

  # common daemon implementation
  daemon.hpp daemon.cpp

  delivery/driver.hpp

  # all driver implementations follow:

  # VTS
  delivery/vts/driver.hpp delivery/vts/driver.cpp
  delivery/vts0/driver.hpp delivery/vts0/driver.cpp

  # SLPK
  delivery/slpk/driver.hpp delivery/slpk/driver.cpp
  )

if(VTSLIBS_HAS_TILESTORAGE)
  list(APPEND vtsd-internals_SOURCES
    delivery/tileset/driver.hpp delivery/tileset/driver.cpp
    )
elseif(VTSD_NEEDS_TILESTORAGE)
  message(FATAL_ERROR "Building VTSD for legacy system where tilestorage "
    "support is mandatory. Please go to the src/vts-libs directory and "
    " checkout the <ts-legacy> branch first.")
endif()

add_library(vtsd-internals ${vtsd-internals_SOURCES})
target_link_libraries(vtsd-internals ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vtsd-internals ${MODULE_DEFINITIONS})
buildsys_library(vtsd-internals)

# vtsd daemon
add_executable(vtsd main.cpp)
target_link_libraries(vtsd vtsd-internals)
buildsys_target_compile_definitions(vtsd ${MODULE_DEFINITIONS})
buildsys_binary(vtsd)
set_target_version(vtsd ${vts-vtsd_VERSION})

# i3sd daemon
add_executable(i3sd i3sd.cpp)
target_link_libraries(i3sd vtsd-internals)
buildsys_target_compile_definitions(i3sd ${MODULE_DEFINITIONS})
buildsys_binary(i3sd)
set_target_version(i3sd ${vts-vtsd_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS vtsd
  RUNTIME DESTINATION bin
  COMPONENT main)

install(TARGETS i3sd
  RUNTIME DESTINATION bin
  COMPONENT main)
