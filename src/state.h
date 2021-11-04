#ifndef STATE_H
#define STATE_H

#include <cstdint>

enum State : std::uint_fast8_t {
  NOT_READY = 0,
  READY,
  TAKING_OFF,
  LANDING,
  HOVERING,
  EXPLORATION,
  RETURNING_BASE,
  CRASHED
};

#endif