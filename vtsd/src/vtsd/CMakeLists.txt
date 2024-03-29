# vtsd - vts delivery daemon
define_module(BINARY vtsd
  DEPENDS
  vts-libs-core>=2.18 http>=1.8 service>=1.7
  slpk>=1.2 imgproc>=1.23 utility>=1.30
  3dtiles>=1.1

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

  delivery/driver.hpp delivery/driver.cpp

  # all driver implementations follow:

  # VTS
  delivery/vts/support.hpp delivery/vts/support.cpp
  delivery/vts/driver.hpp delivery/vts/driver.cpp
  delivery/vts/mapconfig.hpp delivery/vts/mapconfig.cpp
  delivery/vts/tdt2vts.hpp delivery/vts/tdt2vts.cpp
  delivery/vts/tdt2vts/convertors.hpp delivery/vts/tdt2vts/convertors.cpp
  delivery/vts/tdt2vts/metabuilder.hpp delivery/vts/tdt2vts/metabuilder.cpp
  delivery/vts/po.hpp delivery/vts/tdt2vts/po.hpp

  # SLPK
  delivery/slpk/driver.hpp delivery/slpk/driver.cpp
  )

set(tdt2vts_BROWSER_SOURCES
  delivery/vts/tdt2vts/support.hpp delivery/vts/tdt2vts/support.cpp
  )

file_to_cpp(tdt2vts_BROWSER_SOURCES
  vts2tdt::browser::index_html
  delivery/vts/tdt2vts/index.html)

file_to_cpp(i3s_BROWSER_SOURCES
  i3s::browser::browser_html
  delivery/slpk/browser/browser.html)

set(vtsd-internals_SOURCES ${vtsd-internals_SOURCES}
  ${tdt2vts_BROWSER_SOURCES} ${i3s_BROWSER_SOURCES})

add_library(vtsd-internals ${vtsd-internals_SOURCES})
target_link_libraries(vtsd-internals ${MODULE_LIBRARIES})
target_compile_definitions(vtsd-internals PRIVATE ${MODULE_DEFINITIONS})
buildsys_library(vtsd-internals)

# vtsd daemon
add_executable(vtsd main.cpp)
target_link_libraries(vtsd vtsd-internals)
target_compile_definitions(vtsd PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(vtsd)
set_target_version(vtsd ${vts-vtsd_VERSION})

# i3sd daemon
add_executable(i3sd i3sd.cpp)
target_link_libraries(i3sd vtsd-internals)
target_compile_definitions(i3sd PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(i3sd)
set_target_version(i3sd ${vts-vtsd_VERSION})

message(STATUS "vts-vtsd_VERSION: ${vts-vtsd_VERSION}")

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
