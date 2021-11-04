add_library(crazyflie_controller MODULE 
  crazyflie_controller.h 
  crazyflie_controller.cpp
  command.h
  drone_registry.h
  socket_link.cpp
  socket_link.h
  state.h
  state_machine.cpp
  state_machine.h
  json.hpp
  )


target_link_libraries(crazyflie_controller
  argos3core_simulator
  argos3plugin_simulator_crazyflie
  argos3plugin_simulator_genericrobot
  )

target_link_libraries(crazyflie_controller spdlog::spdlog)





