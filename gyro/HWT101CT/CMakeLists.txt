add_library(SensorGyroHWT101CT STATIC
    "./HWT101CT.cpp"
)

target_include_directories(SensorGyroHWT101CT
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(SensorGyroHWT101CT PUBLIC protocol::UartRxSync)
target_link_libraries(SensorGyroHWT101CT PUBLIC utils)

# alias for external use
add_library(SensorGyro::HWT101CT ALIAS SensorGyroHWT101CT)
