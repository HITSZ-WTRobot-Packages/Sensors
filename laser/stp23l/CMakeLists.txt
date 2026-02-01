add_library(SensorLaserSTP23L STATIC
    "./STP23L.cpp"
)

target_include_directories(SensorLaserSTP23L
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(SensorLaserSTP23L PUBLIC protocol::UartRxSync)

# alias for external use
add_library(SensorLaser::STP23L ALIAS SensorLaserSTP23L)
