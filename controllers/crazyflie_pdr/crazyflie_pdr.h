#ifndef CRAZYFLIE_CONTROLLER_H
#define CRAZYFLIE_CONTROLLER_H

#include "command.h"
#include "drone_registry.h"
#include "socket_link.h"
#include "state.h"
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/crazyflie/control_interface/ci_crazyflie_distance_scanner_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_speed_actuator.h>

using namespace argos;

class CrazyflieController : public CCI_Controller {

public:
  CrazyflieController();

  virtual ~CrazyflieController() {}

  virtual void Init(TConfigurationNode &t_node);
  virtual void ControlStep();

  virtual void Reset() {}

  virtual void Destroy() {}

  void TakeOff(unsigned int tick);

  void Land();

private:
  CCI_BatterySensor *battery_;
  CCI_CrazyflieDistanceScannerSensor *distance_scanner_;
  CCI_QuadRotorPositionActuator *position_actuator_;
  CCI_PositioningSensor *position_sensor_;
  SocketLink *socket_link_;
  unsigned long tick_;
  unsigned int socket_port_;
  State state_;

  // CCI_QuadRotorSpeedActuator* speed_actuator_;
};

#endif