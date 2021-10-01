add_library(crazyflie_pdr MODULE crazyflie_pdr.h crazyflie_pdr.cpp)
target_link_libraries(crazyflie_pdr
  argos3core_simulator
  argos3plugin_simulator_crazyflie
  argos3plugin_simulator_genericrobot
  )

target_link_libraries(crazyflie_pdr socket_link spdlog::spdlog)


include_directories(${CMAKE_SOURCE_DIR}/socket_link)


