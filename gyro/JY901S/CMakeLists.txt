add_library(SensorGyroJY901S STATIC
    "./JY901S.cpp"
)

target_include_directories(SensorGyroJY901S
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(SensorGyroJY901S PUBLIC FreeRTOS)
target_link_libraries(SensorGyroJY901S PUBLIC Math::Geometry)
target_link_libraries(SensorGyroJY901S PUBLIC protocol::UartRxSync)
target_link_libraries(SensorGyroJY901S PUBLIC utils)

# alias for external use
add_library(SensorGyro::JY901S ALIAS SensorGyroJY901S)
