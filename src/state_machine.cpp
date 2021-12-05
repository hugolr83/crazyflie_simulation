#include "state_machine.h"

StateMachine::StateMachine()
    : state(State::NOT_READY), current_velocity(0.0, 0.0, 0.0),
      is_landing(false) {
  spdlog::info("State machine ctor\n");
}

void StateMachine::SetState(State newState) { state = newState; }

State StateMachine::GetState() const { return state; }

// return what to do next
Action StateMachine::DoState(argos::CVector3 position, argos::CVector3 target,
                             RangeData range_data, int battery_charge) {

  argos::CVector3 next_position = argos::CVector3::ZERO;
  Action action{next_position, argos::CRadians::ZERO, false};

  switch (state) {
  case NOT_READY:
    if (battery_charge > battery_level_threshold) {
      state = READY;
      break;
    }

    break;

  case READY:
    if (battery_charge < battery_level_threshold) {
      state = NOT_READY;
      break;
    }
    break;

  case TAKING_OFF:
    action.next_position.SetZ(altitude);
    action.is_absolute = true;
    break;

  case LANDING:
    if (!is_landing) {
      landing_position = position;
      landing_position.SetZ(land_level);
      is_landing = true;
    }

    action.next_position = landing_position;

    action.is_absolute = true;

    if (position.GetZ() < land_level_threshold) {
      state = READY;
      break;
    }
    break;

  case HOVERING:
    state = EXPLORATION;
    break;

  case EXPLORATION: {
    if (battery_charge < battery_level_threshold) {
      state = RETURNING_BASE;
      break;
    }

    if (position.GetZ() < altitude - land_level) {
      action.is_absolute = true;
      action.next_position =
          argos::CVector3(position.GetX(), position.GetY(), altitude);
      break;
    }

    argos::CVector3 desired_velocity =
        GetDesiredVelocity(position, target, range_data);
    action.next_position = Update(desired_velocity);
    action.yaw = desired_velocity.GetYAngle();
    action.next_position.SetZ(0.0);

    if (position.GetZ() > altitude) {
      action.next_position.SetZ(altitude - position.GetZ());
    }
  } break;

  case RETURNING_BASE: {
    if (battery_charge < battery_level_threshold) {
      state = LANDING;
      break;
    }

    argos::CVector3 desired_velocity =
        GetDesiredVelocity(position, initial_position, range_data);
    action.next_position = Update(desired_velocity);
    action.yaw = desired_velocity.GetYAngle();
    action.next_position.SetZ(0.0);

    if (position.GetZ() > altitude) {
      action.next_position.SetZ(altitude - position.GetZ());
    }
    if ((position.GetX() <= initial_position.GetX() + return_base_threshold &&
         position.GetX() >= initial_position.GetX() - return_base_threshold) &&
        (position.GetY() <= initial_position.GetY() + return_base_threshold &&
         position.GetY() >= initial_position.GetY() - return_base_threshold)) {
      state = LANDING;
    }
  } break;

  case CRASHED:
    spdlog::warn("Crashed \n");
    break;

  default:
    break;
  }

  return action;
}

/*
Compute the desired velocity :
If there is no obstacle, the desired velocity will point to the target.
otherwise it will point so that it goes away from the obstacle.
*/
argos::CVector3 StateMachine::GetDesiredVelocity(argos::CVector3 position,
                                                 argos::CVector3 target,
                                                 RangeData range_data) {
  argos::CVector3 desired_velocity;

  double d1 = range_data.d1;
  double d2 = range_data.d2;
  double d3 = range_data.d3;
  double d4 = range_data.d4;

  if (d1 != -2 && d1 <= distance_wall_threshold) {
    desired_velocity = argos::CVector3(current_velocity.GetX(), max_speed, 0.0);

  } else if (d2 != -2 && d2 <= distance_wall_threshold) {
    desired_velocity =
        argos::CVector3(-max_speed, current_velocity.GetY(), 0.0);

  } else if (d3 != -2 && d3 <= distance_wall_threshold) {
    desired_velocity =
        argos::CVector3(current_velocity.GetX(), -max_speed, 0.0);

  } else if (d4 != -2 && d4 <= distance_wall_threshold) {
    desired_velocity = argos::CVector3(max_speed, current_velocity.GetY(), 0.0);

  } else {
    desired_velocity = target - position;
  }

  return desired_velocity;
}

argos::CVector3 StateMachine::Update(argos::CVector3 desired_velocity) {
  argos::CVector3 steering_force = Seek(desired_velocity);
  current_velocity += steering_force;
  current_velocity = Limit(current_velocity, max_speed);
  return current_velocity;
}

argos::CVector3 StateMachine::Seek(argos::CVector3 desired_velocity) {
  desired_velocity = desired_velocity.Normalize();
  desired_velocity *= max_speed;

  argos::CVector3 steering = desired_velocity - current_velocity;
  steering = Limit(steering, max_force);
  return steering;
}

argos::CVector3 StateMachine::Flee(argos::CVector3 desired_velocity) {
  return Seek(desired_velocity).Negate();
}

argos::CVector3 StateMachine::Limit(argos::CVector3 vector, double max) {
  if (vector.Length() > max) {
    vector = vector.Normalize();
    vector = vector * max;
  }

  return vector;
}
