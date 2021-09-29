#include <argos3/core/utility/configuration/argos_configuration.h>
#include "crazyflie_pdr.h"
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;


CrazyflieController::CrazyflieController() : 
   battery_(NULL), 
   distance_scanner_(NULL),
   // speed_actuator_(NULL),
   position_actuator_(NULL),
   position_sensor_(NULL),
   tick_(0) {}

void CrazyflieController::Init(TConfigurationNode& t_node) {
   LOG << " INIT \n";

   try {
      
      distance_scanner_   = GetSensor  <CCI_CrazyflieDistanceScannerSensor>("crazyflie_distance_scanner");
      battery_ = GetSensor<CCI_BatterySensor>("battery");
      position_actuator_ = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
      position_sensor_ = GetSensor<CCI_PositioningSensor>("positioning");
      // speed_actuator_ = GetSensor<CCI_QuadRotorSpeedActuator>("quadrotor_speed");


   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the crazyflie sensing controller for robot \"" << GetId() << "\"", ex);
   }
   
   Reset();



}

void CrazyflieController::ControlStep() {
   if (tick_ == 0) {
      TakeOff();
   }
   auto bat_reading = battery_->GetReading();
   LOG << "battery reading " << bat_reading.AvailableCharge << "\n";
   
   // LOG << "Current position " << position_sensor_->GetReading().Position << "\n";
   

   if (tick_ == 100) {
      Land();
   }

   tick_++;
   


}

void CrazyflieController::TakeOff() {
   CVector3 vec = CVector3(0.0, 0.0, 2.0);
   position_actuator_->SetAbsolutePosition(vec);
}

void CrazyflieController::Land() {
   CVector3 vec = CVector3(0.0, 0.0, 0.0);
   position_actuator_->SetAbsolutePosition(vec);
}

REGISTER_CONTROLLER(CrazyflieController, "crazyflie_pdr_controller")