add_library(socket_link SHARED socket_link.h socket_link.cpp)

include_directories(${CMAKE_SOURCE_DIR}/controllers/crazyflie_pdr)
target_link_libraries(socket_link PRIVATE spdlog::spdlog)



