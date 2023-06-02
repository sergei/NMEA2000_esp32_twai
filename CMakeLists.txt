FILE(GLOB_RECURSE sources ./*.*)
idf_component_register(SRCS ${sources} INCLUDE_DIRS .
REQUIRES NMEA2000
)
# Remove pr change this line to adjust the size of log component
target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")