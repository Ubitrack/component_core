set(the_description "The UbiTrack IOComponents")

if(HAVE_OPENCV)
  ut_add_component(utiocomponents DEPS utcore utdataflow utvision)
  ut_component_include_directories(${UBITRACK_CORE_DEPS_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR})
else(HAVE_OPENCV)
  ut_add_component(utiocomponents DEPS utcore utdataflow)
  ut_component_include_directories(${UBITRACK_CORE_DEPS_INCLUDE_DIR})
endif(HAVE_OPENCV)

ut_glob_component_sources("HEADERS" "*.h" "SOURCES" "*.cpp")

if(ENABLE_EXPAND_MULTICOMPONENTS)
    ut_create_multi_component(${OPENCV_LIBRARIES})
else(ENABLE_EXPAND_MULTICOMPONENTS)
    ut_create_single_component(${OPENCV_LIBRARIES})
endif(ENABLE_EXPAND_MULTICOMPONENTS)