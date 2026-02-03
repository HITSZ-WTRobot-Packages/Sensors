add_library(SensorLaserDT35 STATIC
    "./DT35.cpp"
)

target_include_directories(SensorLaserDT35
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(SensorLaserDT35 PUBLIC protocol::UartRxSync)

# alias for external use
add_library(SensorLaser::DT35 ALIAS SensorLaserDT35)
