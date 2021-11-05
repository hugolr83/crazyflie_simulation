#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "state.h"
#include <argos3/core/utility/math/vector3.h>
#include <spdlog/spdlog.h>

#define ALTITUDE 1.0
#define MAX_SPEED 0.1
#define MAX_FORCE 0.4
#define DISTANCE_WALL_THRESHOLD 30.0
#define MAX_DISTANCE 40.0
#define RETURN_BASE_THRESHOLD 0.2

struct Action {
  argos::CVector3 next_position;
  bool is_absolute;
};

struct RangeData {
  double d1; // left
  double d2; // back
  double d3; // right
  double d4; // rup
};

class StateMachine {
public:
  StateMachine();
  void SetState(State state);
  void SetInitialPosition(argos::CVector3 position) {
    initial_position = position;
  };
  State GetState() const;
  Action DoState(argos::CVector3 position, argos::CVector3 target,
                 RangeData range_data);

private:
  State state;
  argos::CVector3 Seek(argos::CVector3 desired_velocity);
  argos::CVector3 Flee(argos::CVector3 desired_velocity);
  argos::CVector3 Limit(argos::CVector3 vector, double max);
  argos::CVector3 GetDesiredVelocity(argos::CVector3 position,
                                     argos::CVector3 target,
                                     RangeData range_data);
  argos::CVector3 Update(argos::CVector3 desired_velocity);
  argos::CVector3 current_velocity;
  argos::CVector3 initial_position;
  argos::CVector3 landing_position;
  bool is_landing;
};

#endif