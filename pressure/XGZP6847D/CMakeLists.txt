add_library(SensorPressureXGZP6847D STATIC
    "./XGZP6847DDevice.cpp"
)

target_include_directories(SensorPressureXGZP6847D
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(SensorPressureXGZP6847D PUBLIC services::I2CUpdateManager)
target_link_libraries(SensorPressureXGZP6847D PUBLIC stm32cubemx)

# alias for external use
add_library(SensorPressure::XGZP6847D ALIAS SensorPressureXGZP6847D)
