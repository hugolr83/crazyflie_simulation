#include "state_machine.h"

StateMachine::StateMachine()
    : state(State::NOT_READY), current_velocity(0.0, 0.0, 0.0) {
  spdlog::info("State machine ctor\n");
}

void StateMachine::SetState(State newState) { state = newState; }

State StateMachine::GetState() const { return state; }

// return what to do next
Action StateMachine::DoState(argos::CVector3 position, argos::CVector3 target,
                             RangeData range_data) {

  argos::CVector3 next_position = argos::CVector3::ZERO;
  Action action{next_position, false};

  switch (state) {
  case NOT_READY:
    // noop
    break;

  case READY:
    // noop
    break;

  case TAKING_OFF:
    action.next_position.SetZ(ALTITUDE);
    break;

  case LANDING:
    action.next_position.SetZ(0.0);
    action.is_absolute = true;
    break;

  case HOVERING:
    state = EXPLORATION;
    break;

  case EXPLORATION: {
    argos::CVector3 desired_velocity =
        GetDesiredVelocity(position, target, range_data);
    action.next_position = Update(desired_velocity);
    action.next_position.SetZ(ALTITUDE);
  } break;

  case RETURNING_BASE: {
    argos::CVector3 desired_velocity =
        GetDesiredVelocity(position, initial_position, range_data);
    action.next_position = Update(desired_velocity);

    if ((position.GetX() + RETURN_BASE_THRESHOLD >= initial_position.GetX() &&
         position.GetX() - RETURN_BASE_THRESHOLD <= initial_position.GetX()) &&
        (position.GetY() + RETURN_BASE_THRESHOLD <= initial_position.GetY() &&
         position.GetY() - RETURN_BASE_THRESHOLD >= initial_position.GetY())) {

      spdlog::info("returning to base done !!!");
      action.next_position.SetZ(-ALTITUDE);
      state = READY;
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
If there is no obstable, the desired velocity will point to the target.
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

  if (d1 != -2 && d1 <= DISTANCE_WALL_THRESHOLD) {
    desired_velocity = argos::CVector3(current_velocity.GetX(), MAX_SPEED, 0.0);

  } else if (d2 != -2 && d2 <= DISTANCE_WALL_THRESHOLD) {
    desired_velocity =
        argos::CVector3(-MAX_SPEED, current_velocity.GetY(), 0.0);

  } else if (d3 != -2 && d3 <= DISTANCE_WALL_THRESHOLD) {
    desired_velocity =
        argos::CVector3(current_velocity.GetX(), -MAX_SPEED, 0.0);

  } else if (d4 != -2 && d4 <= DISTANCE_WALL_THRESHOLD) {
    desired_velocity = argos::CVector3(MAX_SPEED, current_velocity.GetY(), 0.0);

  } else {
    desired_velocity = target - position;
  }

  return desired_velocity;
}

argos::CVector3 StateMachine::Update(argos::CVector3 desired_velocity) {
  argos::CVector3 steering_force = Seek(desired_velocity);
  current_velocity += steering_force;
  current_velocity = Limit(current_velocity, MAX_SPEED);
  return current_velocity;
  //   current_position += current_velocity;
  //   position_actuator_->SetAbsolutePosition(
  //       CVector3(current_position.GetX(), current_position.GetY(),
  //       ALTITUDE));
}

argos::CVector3 StateMachine::Seek(argos::CVector3 desired_velocity) {
  desired_velocity = desired_velocity.Normalize();
  desired_velocity *= MAX_SPEED;

  argos::CVector3 steering = desired_velocity - current_velocity;
  steering = Limit(steering, MAX_FORCE);
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
