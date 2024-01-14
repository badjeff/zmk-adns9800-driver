zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ADNS9800 src/adns9800.c)
zephyr_library_sources_ifdef(CONFIG_ADNS9800 src/adns9800_priv.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
