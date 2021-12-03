#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "state.h"
#include <argos3/core/utility/math/vector3.h>
#include <spdlog/spdlog.h>

constexpr double altitude = 0.2f;
constexpr double max_speed = 0.05f;
constexpr double max_force = 0.1f;
constexpr double distance_wall_threshold = 30.0f;
constexpr double return_base_threshold = 0.03f;
constexpr int battery_level_threshold = 30;

struct Action {
  argos::CVector3 next_position;
  argos::CRadians yaw;
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
                                     RangeData range_data, int battery_charge);
  argos::CVector3 Update(argos::CVector3 desired_velocity);
  argos::CVector3 current_velocity;
  argos::CVector3 initial_position;
  argos::CVector3 landing_position;
  bool is_landing;
};

#endif