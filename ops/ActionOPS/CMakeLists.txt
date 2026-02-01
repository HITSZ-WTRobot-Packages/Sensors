add_library(SensorOPSActionOPS STATIC
    "./ActionOPS.cpp"
)

target_include_directories(SensorOPSActionOPS
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(SensorOPSActionOPS PUBLIC protocol::UartRxSync)
target_link_libraries(SensorOPSActionOPS PUBLIC FreeRTOS)

# alias for external use
add_library(SensorOPS::ActionOPS ALIAS SensorOPSActionOPS)
