#include "main.h"
#include "tbdConfiguration.hpp"

#define PRE 2
#define CATCH 1
#define RELEASE 0

#define ON 1
#define OFF -1

// go straight parameters

//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.35, 0, 26, 6000, 3, hardwareParameter);
//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.35, 0, 24, 6000, 2, hardwareParameter);
//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.3, 0, 28, 6000, 1, hardwareParameter);

// goStraightCmPID_lib(60.0, 90, 127, MOVE_BACKWARD, 5, 0, 3, 0.40, 0, 1, 2000, 2, hardwareParameter);



// left is positive, right is negative

using namespace pros;

Point curPos;
double distance;
double angle;
double currentInertia;


/***********************************************************************************************

AUTONOMOUS PROGRAMS

***********************************************************************************************/

/************************************************************************
// ENODEV; 19 - The port cannot be configured as a vision sensor
// EINVAL; 22 - sig_id is outside the range [1-7]
// EDOM;   33 - size_id is greater than the number of available objects.
// EAGAIN; 11 - Reading the vision sensor failed for an unknown reason.	
**************************************************************************/

detected_vision_goal_lib get_goal_object_front_vision(int goal_color_signature) {
	detected_vision_goal_lib final_goal = {goal_color_signature, 0, 0, 0, 0, 0, 0};
	int e_number1;
	int e_number2;
	vision_object_s_t left_goal;
	vision_object_s_t right_goal;

   errno = 6;
	vision_object_s_t largest_goal = front_vision.get_by_sig(0, goal_color_signature);
   std::cout << "   mid = " << largest_goal.x_middle_coord << "...";
	e_number1 = errno;
   errno = 6;
	vision_object_s_t second_goal = front_vision.get_by_sig(1, goal_color_signature);
	e_number2 = errno;

	if (e_number1 == 33) {
		final_goal.width = -1;
		final_goal.height = -1;
      std::cout << "   111   ";
		return final_goal;
	}

	final_goal.left_coord = largest_goal.left_coord;
	final_goal.top_coord = largest_goal.top_coord;
	final_goal.width = largest_goal.width;
	final_goal.height = largest_goal.height;
	final_goal.x_middle_coord = largest_goal.x_middle_coord;
	final_goal.y_middle_coord = largest_goal.y_middle_coord;
	if (e_number2 == 33)	{  //only one object detected
      std::cout << "   222   ";
		return final_goal;
	}	else	{ //two object detected
		if (largest_goal.width < 40)	{ //object too small to combine
         std::cout << "   333   ";
			return final_goal;
		} else { //object big/close enough to combine
			if((abs(largest_goal.top_coord - second_goal.top_coord) <= 5)  
							&& (abs(largest_goal.height - second_goal.height) <= 8)){  //two goals with similar height
				if(largest_goal.left_coord < second_goal.left_coord){
					left_goal = largest_goal;
					right_goal = second_goal;
				} else {
					left_goal = second_goal;
					right_goal = largest_goal;
				}
				if(left_goal.left_coord + left_goal.width < right_goal.left_coord) { // two goals not overlapped 
               final_goal.left_coord = left_goal.left_coord;
               final_goal.top_coord = left_goal.top_coord;
               final_goal.width = right_goal.left_coord - left_goal.left_coord + right_goal.width;
               final_goal.height = left_goal.height;
               final_goal.x_middle_coord = final_goal.left_coord + final_goal.width/2;
               final_goal.y_middle_coord = final_goal.top_coord + final_goal.height/2;
      std::cout << "   444   ";
					return final_goal;
				} else { //two goals overlapped
      std::cout << "   555   ";
					return final_goal;
				}
			} else { //two goals are in different height
				      std::cout << "   666   ";

				return final_goal;
			}	
		}
	}

      std::cout << "   777   ";
	return final_goal;
}



void auton_60s_skills_bridge_version(){
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;
	long hook_action_delay_time = 50;
	long claw_action_delay_time = 75;
	long start_time = pros::millis();
	int intake_speed = 120;
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

//	pros::Task arm_holding_task(arm_moving_holding_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "arm_holding_task");


// sys_display_info_terminal = DEBUG_DISPLAY_MASSAGE_ON;
// top_piston.set_value(false);
// sys_display_info_terminal = DEBUG_DISPLAY_MASSAGE_ON;
// goStraightCm_Front_Vision(55, 90, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
// 			              						0.5, 0, 1, 0.3, 0, 10, 0.8, 0, 0, 1000, 1, hardwareParameter);
// front_piston.set_value(true);
// pros::lcd::print(3, "bbbbbbbbbbbbbbbbb");
// waitForTouch();
// pros::lcd::print(3, "aaaaaaaaaaaaaaaaa");
// while(true){
// 	delay(50);
// }



// int num1 = 0;
// int num2 = 0;
// while(true){
// 	if(is_limit_switch_pressed() == true){
// 		num1++;
// 		pros::lcd::print(1, "pressed = %d", num1);
// 	} else {
// 	  num2++;
// 		pros::lcd::print(2, "not pressed%d", num2);
//   }
// 	delay(10);
// }

// //	bridge code
//    sys_initial_robot_heading = 135;
//
// clawAction_1 = {0, true, 1};
// hookAction_1 = {0, true, 1};
// delay(100);
// turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
// waitForTouch();
// 	clawAction_1 = {0, true, 1};
// 	delay(claw_action_delay_time);
// 	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
// 	delay(300);
// 	goStraightCmPID_lib(20, 185, 127, MOVE_FORWARD, 8, 0, 0, 0.2, 0, 5, 800, 1, hardwareParameter);
// 	delay(100);
// 	goStraightCmPID_lib(35, 183, 127, MOVE_FORWARD, 1.2, 0, 0, 0.5, 0, 5, 1200, 1, hardwareParameter);
// 	delay(100);
// //	turnDegreesPID_lib(185, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 900, 2, hardwareParameter);
// //	goStraightCmPID_lib(30, 185, 127, MOVE_FORWARD, 3, 0, 0, 1, 0, 5, 1000, 1, hardwareParameter);
// //waitForTouch();
// 	armAction_1 = {127, 0, 20, 1};
// 	delay(800);
// 	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
//
// 	goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 2000, 1, hardwareParameter);
// 	balance_bridge_PID_lib(127, -15, 15, 0, 0, 5000, 1, hardwareParameter);
// 	goStraightCmPID_lib(3.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);
//
// 	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
// 	waitForTouch();



// 	hookAction_1 = {0, true, 1};
// 	clawAction_1 = {0, true, 1};
// 	delay(claw_action_delay_time);
// 		left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 		left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 		left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 		right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 		right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD) ;
// 		right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

// 		goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 3000, 1, hardwareParameter);
// 		balance_bridge_PID_lib(127, -18, 15, 0, 0, 5000, 1, hardwareParameter);
// 		goStraightCmPID_lib(1.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 3, 0, 0, 800, 1, hardwareParameter);


// pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
// std::cout << "Time=%d" << pros::millis() - start_time << std::endl;
// waitForTouch();

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};
	w_piston.set_value(false);
	delay(300);

// ------------------------------------------------------------------------------------ //
//// part 1 take first blue goal
	goStraightCmPID_lib(10, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
//	goStraightCmPID_lib(15.5, 180, 80, MOVE_FORWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	delay(hook_action_delay_time);
//	intakeAction_1 = {100, 500, 0, 100, 1};
	goStraightCmPID_lib(65, 80, 127, MOVE_FORWARD, 1.8, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter); //100,80,127
   goStraightCm_Front_Vision(60, 80, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
 			              						0.5, 0, 1, 0.3, 0, 10, 0.8, 0, 0, 1500, 1, hardwareParameter);
	// new_goStraightCm_Front_Vision(25, 73, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
   //  										0.5, 0, 1, 0.5, 0, 15, 0.25, 0, 1, 600, 1, hardwareParameter);

  angle = get_robot_heading_lib(hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
//	goStraightCmPID_lib_limit_switch(20, angle, 60, MOVE_FORWARD, 1.5, 0, 1.5, 0.8, 0, 0, 600, 1, hardwareParameter);

///////catch first yellow goal
	clawAction_1 = {0, true, 1};
	delay(200);
	armAction_1 = {127, 5, 200, 1};
	intakeAction_1 = {intake_speed, 200, 0, intake_speed, 1};
delay(500);

// 	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	waitForTouch();



///////carry first yellow goal to bridge
angle = 64;
	goStraightCmPID_lib(158, angle, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.2, 0, 5, 1650, 1, hardwareParameter); //1650
	armAction_1 = {127, 0, PRESS_BRIDGE - 25, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay(100);

	goStraightCmPID_lib(15, angle, 127, MOVE_FORWARD, 2.5, 0, 1.5, 0.5, 0, 10, 400, 1, hardwareParameter); //100,80,127
///////place the first yellow goal to the bridge
  armAction_1 = {127, 0, PRESS_BRIDGE - 25, 1};
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time + 50);

	angle = 70;
	goStraightCmPID_lib(5, angle, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 10, 0, 5, 300, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(100);
	goStraightCmPID_lib(70, 50, 127, MOVE_BACKWARD, 5, 0, 2.5, 0.5, 0, 4, 1200, 1, hardwareParameter);

	////take first blue goal by front claw
		hookAction_1 = {0, false, 1};
		armAction_1 = {127, 0, 0, 1};
		delay(hook_action_delay_time);
		goStraightCmPID_lib(27, 50, 100, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 700, 1, hardwareParameter);
		turnDegreesPID_lib(230, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
//		goStraightCmPID_lib_limit_switch(27, 230, 60, MOVE_FORWARD, 3.5, 0, 1, 0.3, 0, 0, 700, 1, hardwareParameter);
		//delay(100);
		clawAction_1 = {0, true, 1};
		armAction_1 = {127, 0, 350, 1};
		delay(100);

		goStraightCmPID_lib(65, 5, 127, MOVE_BACKWARD, 1.0, 0, 1.2, 0.7, 0, 10, 1000, 1, hardwareParameter); //1500
		double distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 120);
		if(distance > 70 || distance < 10){
			distance = 20;
		}
		goStraightCm_Back_Vision(distance + 20, 350, 65, DETECT_BLUE_GOAL_SIG, back_vision,
														 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, 800, 1, hardwareParameter);

	////take first red goal by back claw
	  hookAction_1 = {0, true, 1};
	  armAction_1 = {127, 5, PRESS_BRIDGE + 300, 1};
		intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	////catch first five flower ring
		goStraightCmPID_lib(112, 340, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.3, 0, 5, 1500, 1, hardwareParameter);

	  goStraightCmPID_lib(20, 10, 70, MOVE_FORWARD, 2.5, 0, 0, 0.8, 0, 2, 800, 1, hardwareParameter); //1300
		goStraightCmPID_lib(25, 60, 80, MOVE_FORWARD, 2.5, 0, 0, 0.8, 0, 2, 800, 1, hardwareParameter); //1300
		goStraightCmPID_lib(43, 80, 80, MOVE_FORWARD, 2.5, 0, 2, 1, 0, 5, 1100, 1, hardwareParameter);
		armAction_1 = {127, 0, PRESS_BRIDGE - 35, 1};
		delay(100);
		goStraightCmPID_lib(15, 80, 100, MOVE_FORWARD, 2.5, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
		clawAction_1 = {0, false, 1};
		delay(claw_action_delay_time);
		goStraightCmPID_lib(5, 95, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 10, 0, 5, 500, 1, hardwareParameter);
		armAction_1 = {127, 0, PRESS_BRIDGE + 200, 1};
	  delay(100);
		goStraightCmPID_lib(12, 180, 100, MOVE_BACKWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);

		armAction_1 = {127, 200, 0, 1};
		turnDegreesPID_lib(260, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
		intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};

		////take middle high yellow goal
		  delay(100);  //
		  new_goStraightCm_Front_Vision(20, 230, 70, DETECT_YELLOW_GOAL_SIG, front_vision,
												0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 800, 1, hardwareParameter);
		//	intakeAction_1 = {-10, 0, 0, -10, 1};
		  angle = get_robot_heading_lib(hardwareParameter);
		  intakeAction_1 = {0, 0, 0, 0, 1};
//		  goStraightCmPID_lib(32, angle, 50, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 1000, 1, hardwareParameter);
//		  goStraightCmPID_lib_limit_switch(32, angle, 60, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 1000, 1, hardwareParameter);
			//delay(150);
			clawAction_1 = {0, true, 1};
			delay(claw_action_delay_time);
//			armAction_1 = {127, 0, PRESS_BRIDGE + 170, 1};
      armAction_1 = {127, 0, 70, 1};
			turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1600, 2, hardwareParameter);
		  intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
			armAction_1 = {127, 0, PRESS_BRIDGE + 190, 1};
			delay(350);
			angle = 95;
			goStraightCmPID_lib(90, angle, 85, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 1500, 1, hardwareParameter);
			//move high yellow goal to red bridge
			armAction_1 = {127, 0, PRESS_BRIDGE - 30, 1};
			delay(300);
			goStraightCmPID_lib(15, angle, 80, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 700, 1, hardwareParameter);
//			armAction_1 = {127, 0, PRESS_BRIDGE - 20, 1};
			armAction_1 = {127, 0, PRESS_BRIDGE - 30, 1};
			delay(500);
//			turnDegreesPID_lib(80, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 700, 2, hardwareParameter);
			////place high yellow mobile goal
			clawAction_1 = {0, false, 1};
			armAction_1 = {127, 0, PRESS_BRIDGE - 40, 1};
			delay(claw_action_delay_time + 100);
			goStraightCmPID_lib(5, angle, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 4, 0, 5, 400, 1, hardwareParameter);
			armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
			delay(200);

			intakeAction_1 = {0, 0, 0, 0, 1};
			goStraightCmPID_lib(60, 90, 127, MOVE_BACKWARD, 3, 0, 1, 0.5, 0, 2, 1200, 1, hardwareParameter);
			goStraightCmPID_lib(100, 135, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 2, 1300, 1, hardwareParameter);
			goStraightCmPID_lib(110, 145, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 2, 1400, 1, hardwareParameter);
			hookAction_1 = {0, false, 1};
			goStraightCmPID_lib(25, 145, 80, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 600, 1, hardwareParameter);

			intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
			goStraightCmPID_lib(68, 130, 127, MOVE_FORWARD, 1.5, 0, 1, 0.5, 0, 7, 1100, 1, hardwareParameter);
			turnDegreesPID_lib(180, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 700, 2, hardwareParameter);

			intakeAction_1 = {0, 0, 0, 0, 1};
			distance = get_distance_back_vision(back_vision, DETECT_RED_GOAL_SIG, 10, 10, 60, 120);
			if(distance > 70 || distance < 10){
				distance = 50;
			}
			goStraightCm_Back_Vision(distance + 20, 180, 70, DETECT_RED_GOAL_SIG, back_vision,
															 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 1, 1000, 1, hardwareParameter);
		//take corner blue goal
			hookAction_1 = {0, true, 1};
			delay(hook_action_delay_time);
			intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};

			armAction_1 = {127, 0, 0, 1};
      goStraightCmPID_lib(50, 180, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 5, 800, 1, hardwareParameter); //900
      turnDegreesPID_lib(90, ON_SPOT_TURN, 80, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);  //900

      goStraightCm_Front_Vision(30, 90, 90, DETECT_YELLOW_GOAL_SIG, front_vision,
			              						0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
      angle = get_robot_heading_lib(hardwareParameter);
//      goStraightCmPID_lib_limit_switch(30, angle, 70, MOVE_FORWARD, 1.5, 0, 1.5, 0.8, 0, 0, 800, 1, hardwareParameter);
//take last yellow goal
      clawAction_1 = {0, true, 1};
      delay(claw_action_delay_time);
      armAction_1 = {127, 0, PRESS_BRIDGE + 200, 1};
      goStraightCmPID_lib(36, 120, 80, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 800, 1, hardwareParameter); //2000
			goStraightCmPID_lib(115, 130, 100, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 1500, 1, hardwareParameter); //2000

			armAction_1 = {127, 0, PRESS_BRIDGE, 1};
			goStraightCmPID_lib(15, 110, 127, MOVE_FORWARD, 3.5, 0, 1, 0.4, 0, 0, 500, 1, hardwareParameter);
//drop last yellow goal
			clawAction_1 = {0, false, 1};
			delay(claw_action_delay_time);
			goStraightCmPID_lib(5, 125, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
			armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
			hookAction_1 = {200, false, 1};
			goStraightCmPID_lib(55, 120, 100, MOVE_BACKWARD, 3, 0, 1, 0.35, 0, 5, 1000, 1, hardwareParameter);

//hookAction_1 = {0, false, 1};
armAction_1 = {127, 0, 0, 1};
intakeAction_1 = {0, 0, 0, 0, 1};
//delay(hook_action_delay_time);
goStraightCmPID_lib(30, 120, 80, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 800, 2, hardwareParameter);
turnDegreesPID_lib(300, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
//goStraightCmPID_lib_limit_switch(45, 300, 60, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 1300, 1, hardwareParameter);
//get last red goal
//delay(100);
clawAction_1 = {0, true, 1};
delay(claw_action_delay_time);
armAction_1 = {127, 0, PRESS_BRIDGE + 190, 1};
turnDegreesPID_lib(110, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 1300, 2, hardwareParameter);
goStraightCmPID_lib(77, 110, 100, MOVE_FORWARD,	5, 0, 1, 0.3, 0, 2, 1400, 1, hardwareParameter);

//release last blue goal to the bridge
//armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
//delay(100);
clawAction_1 = {0, false, 1};
delay(claw_action_delay_time);

	armAction_1 = {127, 500, 0, 1};
	goStraightCmPID_lib(70, 180, 127, MOVE_BACKWARD,	3.5, 0, 1, 1, 0, 2, 1400, 1, hardwareParameter);
	turnDegreesPID_lib(120, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);

	distance = get_distance_front_vision(front_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 120);
	if(distance > 70 || distance < 10){
		distance = 50;
	}
	new_goStraightCm_Front_Vision(15, 120, 70, DETECT_BLUE_GOAL_SIG, front_vision,
										0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 1000, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
//	goStraightCmPID_lib_limit_switch(distance - 10, angle, 50, MOVE_FORWARD,	1, 0, 1, 0.3, 0, 1, 1200, 1, hardwareParameter);
//catch the goal beside the bridge
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, 60, 1};
	goStraightCmPID_lib(200, 83, 127, MOVE_BACKWARD,	2, 0, 1, 0.3, 0, 5, 2500, 1, hardwareParameter);
	goStraightCmPID_lib(40, 135, 127, MOVE_FORWARD,	1.5, 0, 1, 0.3, 0, 5, 1000, 1, hardwareParameter);
	goStraightCmPID_lib(90, 135, 127, MOVE_BACKWARD,	2, 0, 1, 0.3, 0, 5, 1500, 1, hardwareParameter);

	// turnDegreesPID_lib(30, ON_SPOT_TURN, 80, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);
	// goStraightCmPID_lib(40, 60, 127, MOVE_FORWARD,	1, 0, 1, 0.3, 0, 1, 900, 1, hardwareParameter);
	// turnDegreesPID_lib(135, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	// goStraightCmPID_lib(40, 135, 100, MOVE_BACKWARD,	1, 0, 1, 0.3, 0, 5, 1000, 1, hardwareParameter);
	delay(100);
	distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 110);
	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	goStraightCm_Back_Vision(distance + 8, 135, 100, DETECT_BLUE_GOAL_SIG, back_vision,
													 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, 1200, 1, hardwareParameter);
	delay(100);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);

	// //	bridge code

//		goStraightCmPID_lib(40, 195, 127, MOVE_FORWARD, 8, 0, 0, 0.3, 0, 5, 1200, 1, hardwareParameter);
		goStraightCmPID_lib(5, 135, 127, MOVE_FORWARD, 2, 0, 0, 1, 0, 0, 500, 1, hardwareParameter);
		turnDegreesPID_lib(215, ON_SPOT_TURN, 60, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
//		delay(100);
    goStraightCmPID_lib(45, 215, 127, MOVE_FORWARD, 1.2, 0, 0, 0.5, 0, 5, 1200, 1, hardwareParameter);
//		goStraightCmPID_lib(35, 180, 127, MOVE_FORWARD, 1.2, 0, 0, 0.5, 0, 5, 1200, 1, hardwareParameter);
//		delay(100);
    armAction_1 = {127, 200, 20, 1};
    turnDegreesPID_lib(180, ON_SPOT_TURN, 60, CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);

//		armAction_1 = {127, 0, 20, 1};
		delay(200);
		left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD) ;
		right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

		goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 3000, 1, hardwareParameter);
		balance_bridge_PID_lib(127, -18, 15, 0, 0, 5000, 1, hardwareParameter);
		goStraightCmPID_lib(1.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 3, 0, 0, 800, 1, hardwareParameter);

		//goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 3000, 1, hardwareParameter);
		//balance_bridge_PID_lib(127, -10, 15, 0, 0, 5000, 1, hardwareParameter);
		//goStraightCmPID_lib(3.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);


pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
std::cout << "Time=%d" << pros::millis() - start_time << std::endl;
waitForTouch();


	////////////////////////////////////////
	////////////////////////////////////////
	////////////////////////////////////////
	//	armAction_1 = {127, 0, 100, 1};
	///////////////////////////////////////?????
	//	goStraightCmPID_lib(15, 30, 100, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 0, 600, 1, hardwareParameter);

	////////////////////////////////////////
	////////////////////////////////////////
	////////////////////////////////////////

//bridge code
// clawAction_1 = {0, true, 1};
// delay(claw_action_delay_time);
// armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
// goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
// hookAction_1 = {0, true, 1};
// delay(hook_action_delay_time);
// delay(1000);
// goStraightCmPID_lib(35, 180, 127, MOVE_FORWARD, 3, 0, 0, 1, 0, 5, 1000, 1, hardwareParameter);
// armAction_1 = {127, 0, 20, 1};
// delay(1000);
// //waitForTouch();
// left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
//
// goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 2000, 1, hardwareParameter);
// balance_bridge_PID_lib(127, -15, 15, 0, 0, 5000, 1, hardwareParameter);
// goStraightCmPID_lib(3, 180, 127, MOVE_BACKWARD, 0, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);
//



pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
waitForTouch();
}


int get_frontYellow_width()
{
	vision_object_s_t front_yellow;
	int w = 0;
	int cnt = 10;

	for (int i = 0; i < 10; i++)
	{
		front_yellow = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
		w += front_yellow.width;
		std::cout << "time " << i + 1 << ": " << front_yellow.width << std::endl;
		delay(10);
	}

	w = w / cnt;
	std::cout << "W: " << w << std::endl;

	return w;
}



void auton_60s_skills_slow_version(){
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;   //was 180
	long hook_action_delay_time = 50;
	long claw_action_delay_time = 75;
	long start_time = pros::millis();

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};
	w_piston.set_value(false);
	delay(50);
	arm_motor.move(-60);
	delay(100);
	arm_motor.move(0);
	arm_motor.tare_position();

///// ------ TEST -----------
//armAction_1 = {127, 5, RELEASE_BRIDGE + 120, 1};
//waitForTouch();
//// ------ END TEST ------


// ------------------------------------------------------------------------------------ //
//// part 1 take first blue goal

	goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	//delay(200);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);

	intakeAction_1 = {-90, 400, 0, -90, 1};
	//goStraightCmPID_lib(60, 80, 127, MOVE_FORWARD, 2.0, 0, 2.5, 0.13, 0, 0, 1500, 1, hardwareParameter); //100,80,127
	goStraightCmPID_lib(30, 86, 127, MOVE_FORWARD, 2.0, 0, 2.5, 0.13, 0, 0, 1500, 1, hardwareParameter); //100,80,127
  	//arm_motor.move(0);
  	//arm_motor.tare_position();
	arm_motor.move(10);

    // Move to 1st yellow goal
	// goStraightCm_Front_Vision(25, 80, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
	//  	                  0.5, 0, 1, 0.5, 0, 7, 1, 0, 1, 700, 1, hardwareParameter); 										0.5, 0, 1, 0.5, 0, 15, 0.25, 0, 1, 600, 1, hardwareParameter);
	new_goStraightCm_Front_Vision(102, 999, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
    										0.5, 0, 1, 0.5, 0, 15, 0.25, 0, 1, 3000, 1, hardwareParameter);
    hookAction_1 = {1100, false, 1};
	angle = get_robot_heading_lib(hardwareParameter);

  // Take 1st yellow goal
	// new_goStraightCm_Front_Vision(40, 80, 70, DETECT_YELLOW_GOAL_SIG, front_vision,
  //   										0.5, 0, 1, 0.3, 0, 4, 0.65, 0, 4, 1000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};

	armAction_1 = {127, 5, RELEASE_BRIDGE + 120, 1};
	delay(100);


    
//	intakeAction_1 = {90, 400, 0, 90, 1};
	//delay(500);

 // Move to blue bridge with 1st yellow goal
	//goStraightCmPID_lib(150, 61, 95, MOVE_FORWARD, 3.5, 0, 2.5, 0.2, 0, 5, 1550, 1, hardwareParameter); //1650
	goStraightCmPID_lib(122, 85, 95, MOVE_FORWARD, 3.5, 0, 2.5, 0.2, 0, 5, 1550, 1, hardwareParameter); //1650
	armAction_1 = {127, 0, PRESS_BRIDGE - 100, 1};
	delay(500);

    //goStraightCmPID_lib(3, 62, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 500, 1, hardwareParameter);  //15
	goStraightCmPID_lib(3, 80, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 500, 1, hardwareParameter);  //15
	clawAction_1 = {0, false, 1};  //runtime 16?
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay (500);
////place first yellow goal
////take first blue goal by front claw
	goStraightCmPID_lib(9, 63, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 300, 1, hardwareParameter);  
	armAction_1 = {127, 5, PRESS_BRIDGE + 100, 1};
	delay(150);
	goStraightCmPID_lib(40, 55, 127, MOVE_BACKWARD, 5, 0, 2.5, 0.5, 0, 5, 1200, 1, hardwareParameter);

    pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
    //waitForTouch();


    // turn around to the blue goal and get the blue with front arm
	armAction_1 = {127, 0, 0, 1};
	//turnDegreesPID_lib(235, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	turnDegreesPID_lib(239, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	//turnDegreesPID_lib(235, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0, 1200, 3, hardwareParameter);
	goStraightCmPID_lib(28, 239, 60, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 700, 1, hardwareParameter);
//  goStraightCm_Front_Vision(30, 239, 100, DETECT_BLUE_GOAL_SIG, front_vision,
//	  									0.5, 0, 1, 0.3, 0, 7, 0.3, 0, 1, 1000, 1, hardwareParameter);
	delay(100);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 0, 250, 1};
	delay(100);

    waitForTouch();

	// Move back to get Red goal
	goStraightCmPID_lib(75, 360, 127, MOVE_BACKWARD, 1.0, 0, 1.2, 0.7, 0, 10, 1300, 1, hardwareParameter); //1500
	goStraightCm_Back_Vision(15, 360, 80, DETECT_RED_GOAL_SIG, back_vision,
		                       0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 500, 1, hardwareParameter);
  angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(25, angle, 80, MOVE_BACKWARD, 3.5, 0, 2.5, 1, 0, 0, 800, 1, hardwareParameter);
////take first red goal by back claw
//  delay(100);
  hookAction_1 = {0, true, 1};
	armAction_1 = {127, 5, PRESS_BRIDGE + 200, 1};
	intakeAction_1 = {-127, 0, 0, -127, 1};
	delay(100);

	goStraightCmPID_lib(102, 348, 90, MOVE_FORWARD, 3.5, 0, 2.5, 1, 0, 0, 2000, 1, hardwareParameter);
//	goStraightCmPID_lib(45, 0, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.3, 0, 0, 1000, 1, hardwareParameter); //1300
//	goStraightCmPID_lib(55, 0, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 1300, 2, hardwareParameter);  //1300
	 goStraightCmPID_lib(35, 0, 65, MOVE_FORWARD, 2.5, 0, 0, 1, 0, 0, 800, 1, hardwareParameter); //1300
	 goStraightCmPID_lib(35, 0, 65, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 0, 800, 1, hardwareParameter); //1300
	 goStraightCmPID_lib(30, 0, 65, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 1300, 2, hardwareParameter);  //1300
	goStraightCmPID_lib(150, 0, 75, MOVE_BACKWARD, 3.5, 0, 2.5, 0.5, 0, 5, 600, 1, hardwareParameter);  //1300



	//goStraightCmPID_lib(120, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 2500, 1, hardwareParameter);

//	goStraightCmPID_lib(60, 0, 100, MOVE_BACKWARD, 1.5, 0, 1, 0.5, 0, 5, 1000, 1, hardwareParameter);
  intakeAction_1 = {-20, 0, 200, 0, 1};
  angle = 105;
	turnDegreesPID_lib(angle, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	//turnDegreesPID_lib(105, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter);

	intakeAction_1 = {-127, 0, 0, -127, 1};
	goStraightCmPID_lib(35, angle, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
  //armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	armAction_1 = {127, 0, PRESS_BRIDGE-70, 1};
  delay(100);
	goStraightCmPID_lib(5, angle, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 300, 1, hardwareParameter);
    ////place blue mobile goal
	clawAction_1 = {0, false, 1};
	waitForTouch();
	goStraightCmPID_lib(5, angle, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 150, 1};
	delay(200);
	waitForTouch();

////take middle high yellow goal
	goStraightCmPID_lib(12, 180, 100, MOVE_BACKWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	intakeAction_1 = {-10, 0, 0, -10, 1};
	armAction_1 = {127, 200, -20, 1};
	turnDegreesPID_lib(230, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	intakeAction_1 = {-127, 0, 0, -127, 1};
//	armAction_1 = {127, 0, -20, 1};
  delay(100);  //
  new_goStraightCm_Front_Vision(40, 230, 70, DETECT_YELLOW_GOAL_SIG, front_vision,
										0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 1000, 1, hardwareParameter);
//	intakeAction_1 = {-10, 0, 0, -10, 1};
  angle = get_robot_heading_lib(hardwareParameter);
  goStraightCmPID_lib(30, angle, 45, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 800, 1, hardwareParameter);
	delay(100);

	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	delay(400);
	intakeAction_1 = {-127, 00, 0, -127, 1};

//+waitForTouch();
//move high yellow goal to red bridge
  angle = 274;
	goStraightCmPID_lib(120, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 2500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE - 10, 1};
	goStraightCmPID_lib(5.5, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 350, 1, hardwareParameter);
	delay(200);
////place high yellow mobile goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	//goStraightCmPID_lib(5, angle, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 350, 1, hardwareParameter);
	goStraightCmPID_lib(6, angle, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 350, 1, hardwareParameter);
  //	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	armAction_1 = {127, 5, PRESS_BRIDGE + 70, 1};   // delay from 0 to 5
	hookAction_1 = {0, false, 1};
	delay(200);

	goStraightCmPID_lib(45, 270, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(100);
	goStraightCmPID_lib(22, 270, 100, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 10, 800, 1, hardwareParameter);
	turnDegreesPID_lib(90, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	goStraightCmPID_lib(40, 90, 55, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 1200, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
  delay(claw_action_delay_time);

//  waitForTouch();

	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	delay(500);

	angle = 250;
	turnDegreesPID_lib(angle, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
  //turnDegreesPID_lib(angle, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 1.6, 0, -1, 1000, 2, hardwareParameter);
	goStraightCmPID_lib(105, angle, 100, MOVE_FORWARD, 3, 0, 1, 0.5, 0, 8, 1750, 1, hardwareParameter);
	armAction_1 = {127, 300, PRESS_BRIDGE, 1};
	turnDegreesPID_lib(275, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 800, 1, hardwareParameter);
	//turnDegreesPID_lib(275, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 1.6, 0, -1, 800, 1, hardwareParameter);
	delay(300);
////place red mobile goal in red bridge
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 270, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(200);
	goStraightCmPID_lib(12, angle + 10, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 500, 1, hardwareParameter);
//	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, 0, 1200, 3, hardwareParameter); 				//shit
  turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	//turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter); 				//shit
	armAction_1 = {127, 0, 0, 1};

// Move to Corner Blue
//goStraightCmPID_lib(130, 182, 100, MOVE_BACKWARD, 2, 0, 1, 1, 0, 5, 1800, 1, hardwareParameter);  //120

	goStraightCmPID_lib(130, 173, 100, MOVE_BACKWARD, 2, 0, 1, 1, 0, 5, 1800, 1, hardwareParameter);  //120

	goStraightCm_Back_Vision(40, 173, 60, DETECT_BLUE_GOAL_SIG, back_vision,
		                       0.5, 0, 1, 0.5, 0, 5, 1.25, 0, 1, 1200, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(20, angle, 60, MOVE_BACKWARD, 1.5, 0, 1.5, 1, 0, 0, 600, 1, hardwareParameter);


//take corner blue goal
  hookAction_1 = {0, true, 1};
  delay(hook_action_delay_time);

	intakeAction_1 = {-127, 0, 0, -127, 1};

// Move to last Yellow Goal
  goStraightCmPID_lib(50, 180, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 5, 800, 1, hardwareParameter); //900
	turnDegreesPID_lib(90, ON_SPOT_TURN, 80, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);  //900
	goStraightCm_Front_Vision(30, 90, 90, DETECT_YELLOW_GOAL_SIG, front_vision,
										0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
  angle = get_robot_heading_lib(hardwareParameter);
  goStraightCmPID_lib(30, angle, 50, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 800, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};


//	intakeAction_1 = {127, 300, 0, 127, 1};
	goStraightCmPID_lib(136, 120, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 2100, 1, hardwareParameter); //2000
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
  delay(200);
	goStraightCmPID_lib(15, 140, 127, MOVE_FORWARD, 3.5, 0, 1, 0.4, 0, 0, 500, 1, hardwareParameter);

  //waitForTouch();

////////////////push left
//release third yellow goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 125, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(200);
	goStraightCmPID_lib(55, 120, 100, MOVE_BACKWARD, 3, 0, 1, 0.35, 0, 5, 1000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay(hook_action_delay_time);
	goStraightCmPID_lib(30, 120, 80, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 900, 2, hardwareParameter);
	turnDegreesPID_lib(300, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	goStraightCmPID_lib(40, 300, 60, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 1000, 1, hardwareParameter);

	delay(100);
	clawAction_1 = {0, true, 1};
  delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 250, 1};
	delay(400);
	turnDegreesPID_lib(120, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	//turnDegreesPID_lib(115, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, -1, 1200, 2, hardwareParameter);
	goStraightCmPID_lib(80, 115, 100, MOVE_FORWARD,	5, 0, 1, 0.3, 0, 0, 1200, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 150, 1};
	goStraightCmPID_lib(10, 120, 127, MOVE_FORWARD,	4, 0, 1, 0.5, 0, 0, 400, 1, hardwareParameter);

//release last blue goal to the bridge
  delay(100);
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time + 150);

	armAction_1 = {127, 300, 400, 1};
	goStraightCmPID_lib(30, 115, 100, MOVE_BACKWARD,	2, 0, 1, 0.3, 0, 0, 1000, 1, hardwareParameter);
	turnDegreesPID_lib(40, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
//	goStraightCmPID_lib(15, 30, 100, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 0, 600, 1, hardwareParameter);
//  delay(400);
	armAction_1 = {127, 300, 200, 1};
	goStraightCmPID_lib(90, 40, 90, MOVE_FORWARD,	5, 0, 1, 0.3, 0, 8, 1600, 1, hardwareParameter);
//	goStraightCmPID_lib(15, 90, 90, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 8, 500, 1, hardwareParameter);
  armAction_1 = {127, 0, 500, 1};
	turnDegreesPID_lib(345, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);
	//turnDegreesPID_lib(315, ON_SPOT_TURN, 100, CLOCKWISE, 1.2, 0, -1, 800, 2, hardwareParameter);

  // Moving to 2nd Red Goal
	double distance = get_distance_back_vision(back_vision, DETECT_RED_GOAL_SIG, 10, 10, 60, 200);
	if(distance > 50 || distance < 20){
		distance = 40;
	}
	goStraightCm_Back_Vision(distance + 15, 345, 50, DETECT_RED_GOAL_SIG, back_vision,
		                       0.5, 0, 1, 0.5, 0, 5, 5, 0, 1, 1000, 1, hardwareParameter); //40
  delay(100);
  hookAction_1 = {0, true, 1};
  delay(hook_action_delay_time);
//	turnDegreesPID_lib(290, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);
	intakeAction_1 = {127, 200, 0, 127, 1};

	//goStraightCmPID_lib(100, 285, 75, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 2000, 1, hardwareParameter);
	goStraightCmPID_lib(200, 270, 127, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 2000, 1, hardwareParameter);
  long time_left = 60000 - (pros::millis() - start_time);
	hookAction_1 = {time_left - 200, false, 1};
//	goStraightCmPID_lib(160, 280, 65, MOVE_FORWARD,	1.5, 0, 0, 1, 0, 0, 3500, 1, hardwareParameter);
  goStraightCmPID_lib(40, 270, 65, MOVE_FORWARD,	3, 0, 0, 2, 0, 0, 800, 1, hardwareParameter);
  goStraightCmPID_lib(40, 265, 65, MOVE_FORWARD,	3, 0, 0, 2, 0, 0, 800, 1, hardwareParameter);
	goStraightCmPID_lib(40, 275, 65, MOVE_FORWARD,	3, 0, 0, 2, 0, 0, 800, 1, hardwareParameter);
	goStraightCmPID_lib(30, 285, 65, MOVE_FORWARD,	3, 0, 0, 2, 0, 0, 700, 1, hardwareParameter);
	hookBreak = true;
	hookAction_1 = {0, false, 1};
	goStraightCmPID_lib(20, 315, 45, MOVE_FORWARD,	1.5, 0, 0, 1, 0, 0, 500, 1, hardwareParameter);
	// if(pros::millis() - start_time < 58800){
	// 	delay(300);
	// }
//	goStraightCmPID_lib(80, 280, 75, MOVE_FORWARD,	1, 0, 1, 0.5, 0, 0, 2000, 1, hardwareParameter);
/* Comment out here
Commemnt out ended here */

pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
waitForTouch();
}

//FINALS AUTONS

void red_left_FINALS () {
	sys_initial_robot_heading = 90;
	armAction_1 = {127, 0, 20, 1};
	clawAction_1 = {1050, true, 1};
	goStraightCmPID_lib(116, 90, 127, MOVE_FORWARD, 5, 0, 0, 0.75, 0, 5, 1200, 1, hardwareParameter);
	delay(300);
	goStraightCmPID_lib(120, 75, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 1, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(155, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 1.6, 0, -1, 800, 3, hardwareParameter);
	goStraightCm_Back_Vision(40, 155, 80, DETECT_RED_GOAL_SIG, back_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-25, 0, 5000, -25, 1};
	goStraightCmPID_lib(52, 155, 127, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter);

}

void blue_left_FINALS () {
	sys_initial_robot_heading = 90;
	armAction_1 = {127, 0, 20, 1};
	clawAction_1 = {1050, true, 1};
	goStraightCmPID_lib(116, 90, 127, MOVE_FORWARD, 5, 0, 0, 0.75, 0, 5, 1000, 1, hardwareParameter);
	delay(300);
	goStraightCmPID_lib(120, 75, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(155, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 1.6, 0, -1, 800, 3, hardwareParameter);
	goStraightCm_Back_Vision(40, 155, 80, DETECT_RED_GOAL_SIG, back_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-25, 0, 0, -25, 1};
	goStraightCmPID_lib(52, 155, 127, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter);

}

void red_middle_FINALS () {
	vision_object_s_t closest_goal;

	sys_initial_robot_heading = 30;
	armAction_1 = {127, 0, 20, 1};
	goStraightCmPID_lib(140, 30, 127, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(300);
	goStraightCmPID_lib(80, 30, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 1, hardwareParameter);
	goStraightCm_Back_Vision(70, 90, 127, DETECT_RED_GOAL_SIG, back_vision,
													 0.5, 0, 1, 0.3, 0, 5, 0.4, 0, 0, 1500, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(10, angle, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-75, 0, 2000, -75, 1};
	goStraightCmPID_lib(60, 180, 127, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	clawAction_1 = {0, false, 1};
}

//WINPOINT AUTONS

void blue_win_point()
{
	long start_time;
	sys_initial_robot_heading = 90;
	goStraightCmPID_lib(20, 90, 127, MOVE_BACKWARD, 4, 0, 2, 0.5, 0, 5, 500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-50, 0, 2000, -50, 1};
	goStraightCmPID_lib(40, 90, 127, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 500, 1, hardwareParameter);
	turnDegreesPID_lib(0, ON_SPOT_TURN, 80, CLOCKWISE, 1.6, 0, -1, 1000, 2, hardwareParameter);  //KP 1
	goStraightCmPID_lib(80, 0, 127, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 1500, 1, hardwareParameter);
	intakeAction_1 ={-127, 0, 1000, -127, 1};
	turnDegreesPID_lib(270, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, -1, 1500, 2, hardwareParameter);
	goStraightCmPID_lib(190, 270, 127, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 2200, 1 ,hardwareParameter);
	hookAction_1 = {0, false, 1};
	goStraightCmPID_lib(15, 270, 127, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 700, 1, hardwareParameter);
	turnDegreesPID_lib(90, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 1.6, 0, -1, 1000, 2, hardwareParameter);
	goStraightCm_Back_Vision(50, 90, 127, DETECT_BLUE_GOAL_SIG, back_vision,
                               0.5, 0, 1, 0.3, 0, 5, 0.4, 0, 0, 800, 1, hardwareParameter);
  angle = get_robot_heading_lib(hardwareParameter);
  goStraightCmPID_lib(10, angle, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
  hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(38, 90, 100, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);
	turnDegreesPID_lib(0, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, -1, 700, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	intakeAction_1 = {-90, 0, 2000, -90, 1};
	goStraightCmPID_lib(90, 0, 75, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 1000, 2, hardwareParameter);
	delay(50);
		intakeAction_1 = {0, 0, 0, 0, 1};
	goStraightCmPID_lib(110, 0, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);
	hookAction_1 = {0, false, 1};

}

void red_win_point()
{
  long start_time;
	sys_initial_robot_heading = 90;
	goStraightCmPID_lib(20, 90, 127, MOVE_BACKWARD, 4, 0, 2, 0.5, 0, 5, 500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-50, 0, 2000, -50, 1};
	goStraightCmPID_lib(40, 90, 127, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 500, 1, hardwareParameter);
	turnDegreesPID_lib(0, ON_SPOT_TURN, 80, CLOCKWISE, 1.6, 0, -1, 1000, 2, hardwareParameter);  //KP 1
	goStraightCmPID_lib(80, 0, 127, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 1500, 1, hardwareParameter);
	intakeAction_1 ={-127, 0, 1000, -127, 1};
	turnDegreesPID_lib(270, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, -1, 1500, 2, hardwareParameter);
	goStraightCmPID_lib(190, 270, 127, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 2200, 1 ,hardwareParameter);
	hookAction_1 = {0, false, 1};
	goStraightCmPID_lib(15, 270, 127, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 700, 1, hardwareParameter);
	turnDegreesPID_lib(90, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 1.6, 0, -1, 1000, 2, hardwareParameter);
	goStraightCm_Back_Vision(50, 90, 127, DETECT_RED_GOAL_SIG, back_vision,
                               0.5, 0, 1, 0.3, 0, 5, 0.4, 0, 0, 800, 1, hardwareParameter);
  angle = get_robot_heading_lib(hardwareParameter);
  goStraightCmPID_lib(10, angle, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
  hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(38, 90, 100, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);
	turnDegreesPID_lib(0, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, -1, 700, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	intakeAction_1 = {-90, 0, 2000, -90, 1};
	goStraightCmPID_lib(90, 0, 75, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 1000, 2, hardwareParameter);
	delay(50);
		intakeAction_1 = {0, 0, 0, 0, 1};
	goStraightCmPID_lib(110, 0, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);
	hookAction_1 = {0, false, 1};
}

//LEFT AUTONS

void red_left_default()
{
	sys_initial_robot_heading = 90;
	armAction_1 = {127, 0, 20, 1};
	clawAction_1 = {1050, true, 1};
	w_piston.set_value(true);
	goStraightCmPID_lib(116, 90, 127, MOVE_FORWARD, 5, 0, 0, 0.75, 0, 5, 2000, 1, hardwareParameter);
	delay (80);

//////////
    ///////////////
	// Move back to get Red Goal
	//////////////
	goStraightCmPID_lib(70, 70, 127, MOVE_BACKWARD, 5, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	goStraightCmPID_lib(26, 135, 127, MOVE_BACKWARD, 5.3, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	goStraightCm_Back_Vision(47, 133, 65, DETECT_RED_GOAL_SIG, back_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	intakeAction_1 = {127, 0, 1000, 127, 1};
	hookAction_1 = {0, true, 1};

    // Relase low Yellow goal
    w_piston.set_value(false);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(60, angle, 55, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter);
    clawAction_1={0, false, 1};
	goStraightCmPID_lib(10, angle, 55, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);  
	
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(52, ON_SPOT_TURN, 100, CLOCKWISE, 1.6, 0, -1, 1500, 3, hardwareParameter);	
	goStraightCm_Front_Vision(163, 52, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.5, 0, 0, 1500, 1, hardwareParameter);   //100
	angle = get_robot_heading_lib(hardwareParameter);
	//w_piston.set_value(true);
	goStraightCmPID_lib(20, angle, 55, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(100);
	armAction_1 = {127, 0, -200, 1};
	goStraightCmPID_lib(120, 15, 127, MOVE_BACKWARD, 4, 0, 2, 0.3, 0, 5, 1500, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};

}

void blue_left_default()
{
	sys_initial_robot_heading = 90;
	armAction_1 = {127, 0, 20, 1};
	clawAction_1 = {1050, true, 1};
	w_piston.set_value(true);
	goStraightCmPID_lib(116, 90, 127, MOVE_FORWARD, 5, 0, 0, 0.75, 0, 5, 2000, 1, hardwareParameter);
	delay (80);

//////////
    ///////////////
	// Move back to get Red Goal
	//////////////
	goStraightCmPID_lib(70, 70, 127, MOVE_BACKWARD, 5, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	goStraightCmPID_lib(26, 135, 127, MOVE_BACKWARD, 5.3, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	//waitForTouch();
	//hookAction_1 = {150, true, 1};
	goStraightCm_Back_Vision(47, 133, 65, DETECT_BLUE_GOAL_SIG, back_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	//delay (50);
	intakeAction_1 = {127, 0, 15000, 127, 1};
	hookAction_1 = {0, true, 1};

    // Relase low Yellow goal
    w_piston.set_value(false);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(60, angle, 55, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter);
    clawAction_1={0, false, 1};
	goStraightCmPID_lib(10, angle, 55, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);  
	
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(52, ON_SPOT_TURN, 100, CLOCKWISE, 1.6, 0, -1, 1500, 3, hardwareParameter);	
	goStraightCm_Front_Vision(168, 52, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.5, 0, 0, 1500, 1, hardwareParameter);   //100
	angle = get_robot_heading_lib(hardwareParameter);
	//w_piston.set_value(true);
	goStraightCmPID_lib(20, angle, 55, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(100);
	armAction_1 = {127, 0, -200, 1};
	goStraightCmPID_lib(120, 70, 127, MOVE_BACKWARD, 4, 0, 2, 0.5, 0, 5, 1500, 1, hardwareParameter);

}

void red_left_rings()

{
	sys_initial_robot_heading = 90;
	armAction_1 = {127, 0, 20, 1};
	clawAction_1 = {1050, true, 1};
	w_piston.set_value(true);
	goStraightCmPID_lib(116, 90, 127, MOVE_FORWARD, 5, 0, 0, 0.75, 0, 5, 2000, 1, hardwareParameter);
	delay (80);

//////////
    ///////////////
	// Move back to get Red Goal
	//////////////
	goStraightCmPID_lib(70, 70, 127, MOVE_BACKWARD, 5, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	goStraightCmPID_lib(26, 135, 127, MOVE_BACKWARD, 5.3, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	//waitForTouch();
	//hookAction_1 = {150, true, 1};
	goStraightCm_Back_Vision(47, 133, 65, DETECT_RED_GOAL_SIG, back_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	//delay (50);
	intakeAction_1 = {127, 0, 1000, 127, 1};
	hookAction_1 = {0, true, 1};

	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(20, angle, 55, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter); 
	turnDegreesPID_lib(90, ON_SPOT_TURN, 100, CLOCKWISE, 1.6, 0, -1, 1500, 3, hardwareParameter);  
	goStraightCmPID_lib(72, 90, 100, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter);
	armAction_1 = {127, 0, 250, 1};
	turnDegreesPID_lib(0, ON_SPOT_TURN, 100, CLOCKWISE, 1.6, 0, -1, 1500, 3, hardwareParameter);
	goStraightCmPID_lib(115, 0, 100, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 15000, 2, hardwareParameter);
	
	
}

//MIDDLE AUTONS

void blue_middle_default()

{
	vision_object_s_t closest_goal;

	sys_initial_robot_heading = 30;
	armAction_1 = {127, 0, 20, 1};
	goStraightCmPID_lib(140, 30, 127, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(300);
	goStraightCmPID_lib(80, 30, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 1, hardwareParameter);
	goStraightCm_Back_Vision(70, 90, 127, DETECT_BLUE_GOAL_SIG, back_vision,
													 0.5, 0, 1, 0.3, 0, 5, 0.4, 0, 0, 1500, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(10, angle, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-75, 0, 2000, -75, 1};
	goStraightCmPID_lib(60, 180, 127, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(20, 180, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 1500, 2, hardwareParameter);
	armAction_1 = {127, 0, 20, 1};
	delay(50);
	turnDegreesPID_lib(0, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 2000, 2, hardwareParameter);

	closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
  std::cout << "Yellow width: " << closest_goal.width << std::endl;

  if (true) {
		goStraightCm_Front_Vision(50, 0, 127, DETECT_YELLOW_GOAL_SIG, front_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
		angle = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, angle, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		clawAction_1 = {0, true, 1};
		armAction_1 = {127, 0, -150, 1};
		goStraightCmPID_lib(120, 0, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 5, 2000, 2, hardwareParameter);

	} else {
		goStraightCmPID_lib(25, 0, 127, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
		turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1500, 2, hardwareParameter);
		intakeAction_1 = {-75, 0, 3000, -75, 1};
		armAction_1 = {127, 0, -150, 1};
		goStraightCmPID_lib(150, 90, 75, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
		intakeAction_1 = {0, 0, 0, 0, 1};

	}

}

void red_middle_default()
{
	vision_object_s_t closest_goal;

	sys_initial_robot_heading = 30;
	armAction_1 = {127, 0, 20, 1};
	goStraightCmPID_lib(140, 30, 127, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(300);
	goStraightCmPID_lib(80, 30, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 1, hardwareParameter);
	goStraightCm_Back_Vision(70, 90, 127, DETECT_RED_GOAL_SIG, back_vision,
													 0.5, 0, 1, 0.3, 0, 5, 0.4, 0, 0, 1500, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(10, angle, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-75, 0, 2000, -75, 1};
	goStraightCmPID_lib(60, 180, 127, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(20, 180, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 1500, 2, hardwareParameter);
	armAction_1 = {127, 0, 20, 1};
	delay(50);
	turnDegreesPID_lib(0, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 2000, 2, hardwareParameter);

	closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
  std::cout << "Yellow width: " << closest_goal.width << std::endl;

  if (true) {
		goStraightCm_Front_Vision(50, 0, 127, DETECT_YELLOW_GOAL_SIG, front_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
		angle = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, angle, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		clawAction_1 = {0, true, 1};
		armAction_1 = {127, 0, -150, 1};
		goStraightCmPID_lib(120, 0, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 5, 2000, 2, hardwareParameter);

	} else {
		goStraightCmPID_lib(25, 0, 127, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
		turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1500, 2, hardwareParameter);
		intakeAction_1 = {-75, 0, 3000, -75, 1};
		armAction_1 = {127, 0, -150, 1};
		goStraightCmPID_lib(150, 90, 75, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
		intakeAction_1 = {0, 0, 0, 0, 1};

	}

}

//RIGHT AUTONS

void red_right_default()
{

	vision_object_s_t closest_goal;
	sys_initial_robot_heading = 90;
	long start_time = pros::millis();

	clawAction_1 = {975, true, 1};
	goStraightCmPID_lib(105, 90, 127, MOVE_FORWARD, 2, 0, 2, 0.75, 0, 5, 1000, 1, hardwareParameter);
	goStraightCmPID_lib(55, 90, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	//armAction_1 = {127, 0, 150, 1};
	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, 1, 1000, 3, hardwareParameter);
	goStraightCm_Back_Vision(50, 180, 127, DETECT_RED_GOAL_SIG, back_vision,
													 0.5, 0, 1, 0.3, 0, 5, 0.4, 0, 0, 1000, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	turnDegreesPID_lib(250, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, 1, 1000, 3, hardwareParameter);
	goStraightCmPID_lib(20, 250, 127, MOVE_FORWARD, 2, 0, 2, 0.75, 0, 5, 500, 1, hardwareParameter);
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(20, 250, 127, MOVE_BACKWARD, 2, 0, 2, 0.75, 0, 5, 500, 1, hardwareParameter);
	turnDegreesPID_lib(145, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, 1, 750, 3, hardwareParameter);
	goStraightCmPID_lib(50, 145, 127, MOVE_FORWARD, 2, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);

 	if (get_frontYellow_width() >= 55) {
		intakeAction_1 = {100, 0, 0, 100, 1};
		armAction_1 = {127, 0, -20, 1};
		goStraightCm_Front_Vision(50, 145, 127, DETECT_YELLOW_GOAL_SIG, front_vision,
								  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
		currentInertia = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(40, currentInertia - 20, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		delay(250);
		clawAction_1 = {0, true, 1};
		goStraightCmPID_lib(120, 120, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 2000, 1, hardwareParameter);

		
	} else {
		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, 1, 1000, 3, hardwareParameter);
		intakeAction_1 = {100, 0, 0, 100, 1};
		goStraightCmPID_lib(200, 180, 75, MOVE_FORWARD, 2, 0, 2, 0.75, 0, 5, 4000, 1, hardwareParameter);
		goStraightCmPID_lib(200, 180, 127, MOVE_BACKWARD, 2, 0, 2, 0.75, 0, 5, 2000, 1, hardwareParameter);
		intakeAction_1 = {0, 0, 0, 0, 1};
		turnDegreesPID_lib(90, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, 1, 1000, 3, hardwareParameter);




	}
    pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
    waitForTouch();



}

void red_two_goals()
{
	vision_object_s_t closest_goal;
	long startingTime = pros::millis();
	sys_initial_robot_heading = 90;

	/*clawAction_1 = {0, false, 1};
    delay(1000);
	clawAction_1 = {0, true, 1};
    front_piston.set_value(false);
    delay(1000);
    front_piston.set_value(true);
	waitForTouch();*/
    
	armAction_1 = {127, 0, 50, 1};
	w_piston.set_value(true);
	goStraightCmPID_lib(105, 90, 127, MOVE_FORWARD, 4, 0, 3, 0.75, 0, 5, 1300, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(40);
	//waitForTouch();
	goStraightCmPID_lib(78, 90, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	clawAction_1 = {0, false, 1};
	delay(40);
	goStraightCmPID_lib(25, 90, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 1000, 2, hardwareParameter);
	turnDegreesPID_lib(131, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1000, 3, hardwareParameter);
	
	waitForTouch();
	goStraightCmPID_lib(70, 131, 127, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);
	armAction_1 = {127, 0, 20, 1};
	goStraightCm_Front_Vision(45, 131, 100, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	clawAction_1 = {0, true, 1};
	goStraightCmPID_lib(52, angle, 127, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);
	delay(50);
	
	waitForTouch();
	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 3, hardwareParameter);
	//distance = get_distance_back_vision(back_vision, DETECT_RED_GOAL_SIG, 10, 10, 60, 110);
	/*goStraightCm_Back_Vision(distance + 8, 135, 100, DETECT_RED_GOAL_SIG, back_vision,
													 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, 1200, 1, hardwareParameter);
	*/delay(100); 
	hookAction_1 = {0, true, 1};
	//goStraightCmPID_lib(120, 180, 127, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);
	intakeAction_1 = {-25, 0, 5000, -25, 1};
}

void encoder()
{
	double enc = 0;
	//sys_initial_robot_heading = 90;

	//goStraightCmPID_lib(20, 90, 127, MOVE_FORWARD, 2, 0, 2, 0.3, 0, 5, 1000, 1, hardwareParameter);


	while(true)
	{
		enc = Y_encoder.get_value();
		pros::lcd::print(1, "encoder = %.1f", enc);
		//std::cout << "encoder: " << enc << std::endl;

		delay(50);

	}
}

/**************************
AUTON
**************************/
void autonomous()
{
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	intake_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);

	// left_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// left_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// right_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// right_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	left_back_motor.set_zero_position(0.0);
	left_front_motor.set_zero_position(0.0);
	left_mid_motor.set_zero_position(0.0);
	right_back_motor.set_zero_position(0.0);
	right_front_motor.set_zero_position(0.0);
	right_mid_motor.set_zero_position(0.0);
	intake_motor.set_zero_position(0.0);
	arm_motor.set_zero_position(0.0);

	sys_initial_to_auton_drifting = hardwareParameter.inertialSensorLib.get_rotation();


//   //this line records the inertial sensor drifting between initialization and start of auton.
//  	sys_initial_robot_heading = 270;
// 	turnDegreesPID_Parameter = {0, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1500, 2};
//   pros::Task background(background_execution_turnDegreesPID_lib, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "turn");

//   double distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 50, 100, 60, 10000);

//waitForTouch();



// waitForTouch();
// //	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);



/***********************************************

CHOOSE RUN HERE

***********************************************/


//finals code -------------------------------------------------------------------------------------------------------------
	//red_left_FINALS ();
    //blue_left_FINALS ();
	//red_middle_FINALS ();
//winpoint code ------------------------------------------------------------------------------------------------------------------
	//blue_win_point ();
	//red_win_point ();
//left side code --------------------------------------------------------------------------------------------------------------
	//red_left_default ();
	//blue_left_default ();
	//red_left_rings ();
	//blue_left_rings ();
//middle goal code --------------------------------------------------------------------------------------------------------------
	//blue_middle_default ();
	//red_middle_default ();
//right side code ---------------------------------------------------------------------------------------------------------------
	red_right_default();
	//red_two_goals();

	//encoder();
	
//skills code ------------------------------------------------------------------------------------------------------------------
	//auton_60s_skills();

	waitForTouch();
}
