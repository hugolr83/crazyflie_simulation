#include <argos3/core/utility/configuration/argos_configuration.h>
#include "crazyflie_pdr.h"
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;


CrazyflieController::CrazyflieController() : 
   battery_(NULL), 
   distance_scanner_(NULL),
   position_actuator_(NULL),
   position_sensor_(NULL),
   tick_(0) {}

void CrazyflieController::Init(TConfigurationNode& t_node) {

   socket_port_ = DroneRegistry::DroneIdToPort.find(GetId())->second;
   LOG << "Init Drone: " << GetId() << " with port : "<< socket_port_ << "\n";


   try {
      
      distance_scanner_   = GetSensor  <CCI_CrazyflieDistanceScannerSensor>("crazyflie_distance_scanner");
      battery_ = GetSensor<CCI_BatterySensor>("battery");
      position_actuator_ = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
      position_sensor_ = GetSensor<CCI_PositioningSensor>("positioning");

      // Start socket server
      socket_link_ = new  SocketLink(socket_port_);
      socket_link_->Start();
      
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the crazyflie sensing controller for robot \"" << GetId() << "\"", ex);
   }
   
   Reset();



}

void CrazyflieController::ControlStep() {
   auto bat_reading = battery_->GetReading();
   Command command = socket_link_->GetCommand();

   if (command) {
      switch (command)
      {
      case Command::START_EXPLORATION:
         LOG << "Drone " << GetId() << " taking off\n";
         TakeOff();
         break;
      case Command::RETURN_TO_BASE:
         LOG << "Drone " << GetId() << " landing\n";
         Land();
         break;
      
      default:
         break;
      }

   }

   tick_++;
   
}

void CrazyflieController::TakeOff() {
   // CVector3 curr = position_sensor_->GetReading().Position;
   // curr.SetZ(2.0);
   // position_actuator_->SetAbsolutePosition(curr);
   position_actuator_->SetAbsolutePosition(CVector3(0.25, 0.25, 2.0));

}

void CrazyflieController::Land() {
   // CVector3 curr = position_sensor_->GetReading().Position;
   // curr.SetZ(1.1);
   position_actuator_->SetAbsolutePosition(CVector3(1.25, 1.25, 2.0));

}

REGISTER_CONTROLLER(CrazyflieController, "crazyflie_pdr_controller")