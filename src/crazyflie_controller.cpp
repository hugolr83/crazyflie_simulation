#include "crazyflie_controller.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;

#define TICK_PULSE 10
#define MM_FACTOR 10.0

CrazyflieController::CrazyflieController()
    : battery_(NULL), distance_scanner_(NULL), position_sensor_(NULL), tick_(0),
      position_actuator_(NULL), target(0.0, 0.0, 0.0), rd() {}

void CrazyflieController::Init(TConfigurationNode &t_node) {
  socket_port_ = DroneRegistry::DroneIdToPort.find(GetId())->second;
  state_machine.SetState(State::NOT_READY);

  spdlog::info("Init Drone: {} with port {} ", GetId(), socket_port_);
  dist = std::uniform_real_distribution<double>(-2.0, 2.0);

  try {

    distance_scanner_ = GetSensor<CCI_CrazyflieDistanceScannerSensor>(
        "crazyflie_distance_scanner");
    battery_ = GetSensor<CCI_BatterySensor>("battery");
    position_sensor_ = GetSensor<CCI_PositioningSensor>("positioning");

    position_actuator_ =
        GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");

    state_machine.SetInitialPosition(position_sensor_->GetReading().Position);

    // Start socket server
    socket_link_ = new SocketLink(socket_port_);
    socket_link_->Start();

  } catch (CARGoSException &ex) {
    THROW_ARGOSEXCEPTION_NESTED(
        "Error initializing the crazyflie sensing controller for robot \""
            << GetId() << "\"",
        ex);
  }

  Reset();
}

void CrazyflieController::ControlStep() {

  current_position = position_sensor_->GetReading().Position;
  current_orientation = position_sensor_->GetReading().Orientation;
  auto distance_data = distance_scanner_->GetReadingsMap();
  battery_reading = battery_->GetReading();
  battery_percentage = static_cast<int>(battery_reading.AvailableCharge * 100);

  auto iter = distance_data.begin();
  double d1, d2, d3, d4;
  iter = distance_data.begin();
  RangeData range_data;
  if (distance_data.size() == 4) {
    range_data = {
        (iter++)->second,
        (iter++)->second,
        (iter++)->second,
        (iter++)->second,
    };
  } else {
    throw std::runtime_error("Sensor invalid");
  }

  if (tick_ % (TICK_PULSE * 5) == 0) {
    // Random position to seek
    target = CVector3(dist(rd), dist(rd), 0.0);
  }

  // 1Hz pulse, 10 ticks/sec
  if (tick_ % TICK_PULSE == 0) {
    Status current_status = EncodeStatus(range_data);
    socket_link_->SendStatus(current_status);
  }

  Command command = socket_link_->GetCommand();

  if (command) {
    switch (command) {
    case Command::TAKE_OFF:
      state_machine.SetState(State::TAKING_OFF);
      break;
    case Command::LAND:
      state_machine.SetState(State::LANDING);
      break;
    case Command::START_EXPLORATION:
      state_machine.SetState(State::EXPLORATION);
      break;
    case Command::RETURN_TO_BASE: {
      auto current_state = state_machine.GetState();
      if (current_state == State::HOVERING) {
        state_machine.SetState(State::LANDING);
      } else if (current_state == State::EXPLORATION) {
        state_machine.SetState(State::RETURNING_BASE);
      }
      break;
    }
    default:
      // noop
      break;
    }
  }

  Action action = state_machine.DoState(current_position, target, range_data,
                                        battery_percentage);

  if (state_machine.GetState() != State::READY) {
    if (action.is_absolute) {
      position_actuator_->SetAbsolutePosition(action.next_position);
    } else {
      position_actuator_->SetRelativePosition(action.next_position);
    }
  }

  tick_++;
}

Status CrazyflieController::EncodeStatus(RangeData range_data) {
  double kalman_state_x;
  double kalman_state_y;
  double kalman_state_z;
  double drone_battery_level;
  int range_front;
  int range_back;
  int range_left;
  int range_right;
  double yaw;

  CRadians x_angle, y_angle, z_angle;
  current_orientation.ToEulerAngles(x_angle, y_angle, z_angle);

  Status current_status = {current_position.GetX(), // kalman_x
                           current_position.GetY(), // kalman_y
                           current_position.GetZ(), // kalman_z
                           battery_percentage,
                           range_data.d4 * MM_FACTOR, // range front
                           range_data.d2 * MM_FACTOR, // range back
                           range_data.d1 * MM_FACTOR, // range left
                           range_data.d3 * MM_FACTOR, // range right
                           z_angle.GetValue(),
                           static_cast<int>(state_machine.GetState())};

  return current_status;
}

REGISTER_CONTROLLER(CrazyflieController, "crazyflie_controller")