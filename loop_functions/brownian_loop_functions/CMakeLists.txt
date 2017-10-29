add_library(brownian_loop_functions MODULE
  brownian_loop_functions.h
  brownian_loop_functions.cpp)

target_link_libraries(brownian_loop_functions
  epuck_brownian
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)

