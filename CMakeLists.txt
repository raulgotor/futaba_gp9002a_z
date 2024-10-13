if (CONFIG_LIB_GP9002A)
  zephyr_include_directories(include)
  zephyr_library_sources(gp9002a.c)
endif()
