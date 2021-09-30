#ifndef COMMAND_H
#define COMMAND_H

#include <cstdint>

enum Command : std::uint_fast8_t {
    TAKE_OFF = 0,
    LAND,
    START_EXPLORATION ,
    RETURN_TO_BASE,
    IDENTIFY,
    UNKNOWN,
};

#endif
