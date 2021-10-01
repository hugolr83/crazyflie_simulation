#ifndef STATE_H
#define STATE_H

#include <cstdint>

enum State : std::uint_fast8_t {
  WAITING,
  STARTING,
  NAVIGATING,
  CRASHED,
  LANDING,

};

#endif