add_library(epuck_brownian SHARED
  epuck_brownian.h
  epuck_brownian.cpp)

target_link_libraries(epuck_brownian
  argos3core_simulator
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)

