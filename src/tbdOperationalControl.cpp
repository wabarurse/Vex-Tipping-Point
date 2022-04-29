#include "../include/main.h"
#include "tbdConfiguration.hpp"
using namespace pros;

//          VARIABLES          //

// motor readings //
float controllerAnalogLeftY;
float controllerAnalogRightY;

// drivetrain
bool upButtonState = false;
int drivetrainState = 0;

// intake
bool AButtonState = false;
bool XButtonState = false;
int intakeState = 0;

// front piston
bool R1ButtonState = false;
int frontPistonState = 0;

// back piston
bool R2ButtonState = false;
int backPistonState = 0;

// auton sheild
bool leftButtonState = false; 
int autonSheildState = 0;
void drive_control_fn(void* param){
  while (true) {
  
    if(master.get_digital(DIGITAL_UP)){
      if(upButtonState == false && drivetrainState == 0){
        upButtonState = true;
        drivetrainState = 1;
        left_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        left_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        right_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        right_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
      }
      else if(upButtonState == false && drivetrainState == 1){
        upButtonState = true;
        drivetrainState = 0;
        left_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
        left_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
        left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
        right_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
        right_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
        right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
      }
    }
    else{
      upButtonState = false;

    }

    if (fabs(master.get_analog(ANALOG_LEFT_Y)) > 10 || fabs(master.get_analog(ANALOG_RIGHT_Y)) > 10) {
      left_front_motor.move(master.get_analog(ANALOG_LEFT_Y));
      left_mid_motor.move(master.get_analog(ANALOG_LEFT_Y));
      left_back_motor.move(master.get_analog(ANALOG_LEFT_Y));
      right_front_motor.move(master.get_analog(ANALOG_RIGHT_Y));
      right_mid_motor.move(master.get_analog(ANALOG_RIGHT_Y));
      right_back_motor.move(master.get_analog(ANALOG_RIGHT_Y));
    
    }
    else{

      left_front_motor.move(0);
      left_mid_motor.move(0);
      left_back_motor.move(0);
      right_front_motor.move(0);
      right_mid_motor.move(0);
      right_back_motor.move(0);
    }

      delay(5);
    }
}

void arm_control_fn(void* param){

  while (true) {

    if (master.get_digital(DIGITAL_L1)){
      arm_motor.move(127);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      arm_motor.move(-127);
    }
    else if (!master.get_digital(DIGITAL_L1) && !master.get_digital(DIGITAL_L2)) {
      arm_motor.move(0);
    }
    delay(10);
  }
}

void intake_control_fn(void* param) {
  while (true) {
    // spin intake forwards //
    if(master.get_digital(DIGITAL_A)){
      if((intakeState == 0 || intakeState == 2) && AButtonState == false){
        AButtonState = true;
        intakeState = 1;
        intake_motor.move(127);
      }
    }
    if(master.get_digital(DIGITAL_A)){
      if(intakeState == 1 && AButtonState == false){
        AButtonState = true;
        intakeState = 0;
        intake_motor.move(0);
      }
    }
    // spin intake backwards //
    if(master.get_digital(DIGITAL_X)){
      if((intakeState == 0 || intakeState == 1) && XButtonState == false){
        XButtonState = true;
        intakeState = 2;
        intake_motor.move(-127);
      }
    }
    if(master.get_digital(DIGITAL_X)){
      if(intakeState == 2 && XButtonState == false){
        XButtonState = true;
        intakeState = 0;
        intake_motor.move(0);
      }
    }
    if (!master.get_digital(DIGITAL_A) && !master.get_digital(DIGITAL_X)){
      XButtonState = false;
      AButtonState = false;
    }
    delay(10);
  }
}

void front_piston_fn(void* param){
  while (true){
    if(master.get_digital(DIGITAL_R1) == 1){
      if(R1ButtonState == false && frontPistonState == 0){
        R1ButtonState = true;
        frontPistonState = 1;
        front_piston.set_value(true);
      }
      if(R1ButtonState == false && frontPistonState == 1){
        R1ButtonState = true;
        frontPistonState = 0;
        front_piston.set_value(false);
      }
    } else {
      R1ButtonState = false;
    }
    delay(10);
  }
}

void back_piston_fn(void* param){
  while (true){
    if(master.get_digital(DIGITAL_R2) == 1){
      if(R2ButtonState == false && backPistonState == 0){
        R2ButtonState = true;
        backPistonState = 1;
        back_piston.set_value(true);
      }
      if(R2ButtonState == false && backPistonState == 1){
        R2ButtonState = true;
        backPistonState = 0;
        back_piston.set_value(false);
      }
    } else {
      R2ButtonState = false;
    }
    delay(10);
  }
}
/*
void auton_sheild_fn(void* param){
  while (true){
       if(master.get_digital(DIGITAL_LEFT) == 1){
      if(leftButtonState == false && autonSheildState == 0){
        leftButtonState = true;
        autonSheildState = 1;
        autonPiston.set_value(true);
      }
      if(leftButtonState == false && autonSHeildState == 1){
        leftButtonState = true;
        autonSheildState = 0;
        autonPiston.set_value(false);
      }
    } else {
      leftButtonState = false;
    }
    delay(10);
  


  }
}
*/
void opcontrol() {


  Task drive(drive_control_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "drive control");
  Task arm(arm_control_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "arm control");
  Task intake(intake_control_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake control");
  Task front_piston(front_piston_fn, (void*) "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "front piston control");
  Task back_piston(back_piston_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "front piston control");

  while(true){
    lcd::print(2, "RF:%.1f-RM:%.1f-RB:%.1f", right_front_motor.get_temperature(), right_mid_motor.get_temperature(), right_back_motor.get_temperature());
    lcd::print(3, "LF:%.1f-LM:%.1f-LB:%.1f", left_front_motor.get_temperature(), left_mid_motor.get_temperature(), left_back_motor.get_temperature());
    delay(50);
  }
}
