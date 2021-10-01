#include "crazyflie_pdr.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;

CrazyflieController::CrazyflieController()
    : battery_(NULL), distance_scanner_(NULL), position_actuator_(NULL),
      position_sensor_(NULL), tick_(0) {}

void CrazyflieController::Init(TConfigurationNode &t_node) {
  socket_port_ = DroneRegistry::DroneIdToPort.find(GetId())->second;
  LOG << "Init Drone: " << GetId() << " with port : " << socket_port_ << "\n";

  try {

    distance_scanner_ = GetSensor<CCI_CrazyflieDistanceScannerSensor>(
        "crazyflie_distance_scanner");
    battery_ = GetSensor<CCI_BatterySensor>("battery");
    position_actuator_ =
        GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
    position_sensor_ = GetSensor<CCI_PositioningSensor>("positioning");

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
  auto bat_reading = battery_->GetReading();
  Command command = socket_link_->GetCommand();

  if (command) {
    switch (command) {
    case Command::START_EXPLORATION:
      LOG << "Drone " << GetId() << " taking off\n";
      state_ = State::NAVIGATING;
      break;
    case Command::LAND:
      LOG << "Drone " << GetId() << " landing\n";
      state_ = State::LANDING;
      Land();
      break;

    default:
      break;
    }
  }

  if (state_ == State::NAVIGATING) {
    TakeOff(tick_);
  }

  tick_++;
}

void CrazyflieController::TakeOff(unsigned int tick) {

  double x = 2.0 * sin(tick % 100);
  double y = 2.0 * cos(tick % 100);

  CVector3 curr = position_sensor_->GetReading().Position;
  curr.Set(x, y, 2.0);

  position_actuator_->SetAbsolutePosition(curr);
}

void CrazyflieController::Land() {
  CVector3 curr = position_sensor_->GetReading().Position;
  curr.Set(curr.GetX(), curr.GetY(), 0.0);

  LOG << "landing at" << curr << "\n";
  position_actuator_->SetAbsolutePosition(curr);
}

REGISTER_CONTROLLER(CrazyflieController, "crazyflie_pdr_controller")