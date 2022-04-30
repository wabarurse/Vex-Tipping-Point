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

void goStraightCmPID_lib_backVision(int distance, int angle, int speed, int timeout)
{
	vision_object_s_t closest_goal_1;
	vision_object_s_t closest_goal_2;
	double w1 = 0, w2 = 0;
	int cnt = 20;

	for (int i = 0; i < 20; i++)
	{
		closest_goal_1 = back_vision.get_by_sig(0, DETECT_RED_GOAL_SIG);
		w1 += closest_goal_1.width;
		closest_goal_2 = back_vision.get_by_sig(0, DETECT_BLUE_GOAL_SIG);
		w2 += closest_goal_2.width;
		delay(10);
	}

	w1 = w1 / cnt;
	w2 = w2 / cnt;

	if (w1 > w2)
	{
		goStraightCm_Back_Vision(distance, angle, speed, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, timeout, 1, hardwareParameter);
		return;
	}

	if (w2 > w1)
	{
		goStraightCm_Back_Vision(distance, angle, speed, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, timeout, 1, hardwareParameter);
		return;
	}

	goStraightCmPID_lib(distance, angle, speed, MOVE_BACKWARD, 0.9, 0, 0.5, 1, 0, 0, timeout, 1, hardwareParameter);
	return;
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

void red_right_default_NEW()
{

	vision_object_s_t closest_goal;
	sys_initial_robot_heading = 90;

	clawAction_1 = {975, true, 1};
	goStraightCmPID_lib(105, 90, 127, MOVE_FORWARD, 2, 0, 2, 0.75, 0, 5, 1000, 1, hardwareParameter);
	goStraightCmPID_lib(55, 90, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, 1, 1000, 3, hardwareParameter);
	goStraightCmPID_lib_backVision(50, 180, 127, 1000);
	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(25, 180, 100, MOVE_FORWARD, 2, 0, 2, 0.75, 0, 5, 700, 2, hardwareParameter);
	clawAction_1 = {0, false, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, 1, 1000, 3, hardwareParameter);
	intakeAction_1 = {127, 0, 0, 127, 1};
	goStraightCmPID_lib(90, 90, 50, MOVE_FORWARD, 2, 0, 0, 1, 0, 0, 2000, 1, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 1.6, 0, 1, 1000, 3, hardwareParameter);
	goStraightCmPID_lib(40, 180, 127, MOVE_FORWARD, 2, 0, 2, 0.75, 0, 5, 650, 1, hardwareParameter);

	if (get_frontYellow_width() >= 65) {
		
		goStraightCm_Front_Vision(40, 180, 127, DETECT_YELLOW_GOAL_SIG, front_vision,
								  0.5, 0, 1, 0.3, 0, 10, 0.3, 0, 5, 1500, 1, hardwareParameter);
		currentInertia = get_robot_heading_lib(hardwareParameter);
		armAction_1 = {127, 0, -50, 1};
		goStraightCmPID_lib(20, currentInertia, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		clawAction_1 = {100, true, 1};
		delay(250);
		armAction_1 = {127, 0, 200, 1};
		turnDegreesPID_lib(110, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, 1, 1000, 1, hardwareParameter);
		goStraightCmPID_lib(150, 110, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 1500, 1, hardwareParameter);
		
	} else {

		goStraightCmPID_lib(40, 180, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 800, 2, hardwareParameter);
		delay(100);
		goStraightCmPID_lib(40, 180, 127, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 800, 2, hardwareParameter);
		turnDegreesPID_lib(270, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, 1, 1000, 3, hardwareParameter);
		goStraightCm_Front_Vision(40, 270, 127, DETECT_YELLOW_GOAL_SIG, front_vision,
								  0.5, 0, 1, 0.3, 0, 10, 0.3, 0, 5, 1500, 1, hardwareParameter);
		currentInertia = get_robot_heading_lib(hardwareParameter);
		armAction_1 = {127, 0, -50, 1};
		goStraightCmPID_lib(50, currentInertia, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		clawAction_1 = {100, true, 1};
		//armAction_1 = {127, 0, 200, 1};
		goStraightCmPID_lib(50, 270, 127, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);


	}

}

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

//SKILLS AUTONS

void auton_60s_skills_slow_version()
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;   //was 180
	long hook_action_delay_time = 50;
	long claw_action_delay_time = 75;
	long start_time = pros::millis();

    int _PRESS_BRIDGE = 360;   // 340;
    int _RELEASE_BRIDGE = 790;
    int _MAX_TOP = 1000;
    int _CARRY_GOAL = 110;

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};
	delay(50);
	arm_motor.move(-60);
	delay(100);
	arm_motor.move(0);
	arm_motor.tare_position();

/// ------------------------------------------------------------------------------------ 
//// 10) take first blue goal
///-------------------------------------------------------------------------------------
	goStraightCmPID_lib(10, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);
	
/// ------------------------------------------------------------------------------------ 
//// 20) Move to 1st Low Yellow
///-------------------------------------------------------------------------------------
	goStraightCmPID_lib(73, 85, 127, MOVE_FORWARD, 1.8, 0, 2.5, 1, 0, 0, 1600, 1, hardwareParameter); //100,80,127 //70

	clawAction_1 = {1100, true, 1};
    goStraightCm_Front_Vision(60, 85, 90, DETECT_YELLOW_GOAL_SIG, front_vision,
 			              						0.5, 0, 1, 0.3, 0, 10, 0.5, 0, 0, 1600, 1, hardwareParameter);  

 	intakeAction_1 = {-90, 200, 0, -90, 1};
	
	///////catch first yellow goal

	delay(200);
	armAction_1 = {127, 5, 200, 1};
    delay(500);
	armAction_1 = {127, 0, 700, 1};   // +120
	hookAction_1 = {700, false, 1};
    
 	

   //-------------------
   // Move to blue bridge with 1st yellow goal
   //-------------------
	
	goStraightCmPID_lib(160, 63, 90, MOVE_FORWARD, 3.5, 0, 2.5, 0.2, 0, 5, 1650, 1, hardwareParameter); //1650 ,speed was 95, now give time for arm to raise
	armAction_1 = {127, 0, _PRESS_BRIDGE, 1};
	delay(500);

 	goStraightCmPID_lib(3, 63, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 500, 1, hardwareParameter);  //15  80
	clawAction_1 = {0, false, 1};  //runtime 16?
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay (500);
////place first yellow goal
////take first blue goal by front claw
	goStraightCmPID_lib(9, 63, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 300, 1, hardwareParameter);  
	armAction_1 = {127, 5, _PRESS_BRIDGE + 100, 1};
	delay(150);
	goStraightCmPID_lib(30, 55, 127, MOVE_BACKWARD, 5, 0, 2.5, 0.5, 0, 5, 1200, 1, hardwareParameter);  //40

    pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
    //waitForTouch();

    ////////////////////////////////////////////
	// 30) Get Blue goal and Red Goal 
	///////////////////////////////////////////
    // turn around to the blue goal and get the blue with front arm
	armAction_1 = {127, 0, 0, 1};
	//turnDegreesPID_lib(235, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	turnDegreesPID_lib(230, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
    goStraightCm_Front_Vision(34, 239, 80, DETECT_BLUE_GOAL_SIG, front_vision,
	  									0.5, 0, 1, 0.3, 0, 7, 0.3, 0, 1, 1200, 1, hardwareParameter);   //33
	delay(100);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 0, 250, 1};
	delay(100);

    //waitForTouch();

	//////////////////////////
	// Move back to get Red goal
	goStraightCmPID_lib(75, 360, 127, MOVE_BACKWARD, 1.0, 0, 1.2, 0.7, 0, 10, 1300, 1, hardwareParameter); //1500
	//goStraightCm_Back_Vision(15, 360, 80, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 500, 1, hardwareParameter);
 
	goStraightCm_Back_Vision(20, 360, 80, DETECT_RED_GOAL_SIG, back_vision,
		                       0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 500, 1, hardwareParameter);
  angle = get_robot_heading_lib(hardwareParameter);
	//goStraightCmPID_lib(25, angle, 80, MOVE_BACKWARD, 3.5, 0, 2.5, 1, 0, 0, 800, 1, hardwareParameter);
	goStraightCmPID_lib(28, angle, 80, MOVE_BACKWARD, 3.5, 0, 2.5, 1, 0, 0, 800, 1, hardwareParameter);
	////take first red goal by back claw
	//  delay(100);
	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 5, _PRESS_BRIDGE + 103, 1};  //+ 100
	intakeAction_1 = {-127, 0, 0, -127, 1};
	delay(100);

	goStraightCmPID_lib(100, 348, 90, MOVE_FORWARD, 3.5, 0, 2.5, 1, 0, 0, 2000, 1, hardwareParameter);   //distance: 102
//	goStraightCmPID_lib(45, 0, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.3, 0, 0, 1000, 1, hardwareParameter); //1300
//	goStraightCmPID_lib(55, 0, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 1300, 2, hardwareParameter);  //1300
	 goStraightCmPID_lib(35, 0, 65, MOVE_FORWARD, 2.5, 0, 0, 1, 0, 0, 800, 1, hardwareParameter); //1300
	 goStraightCmPID_lib(35, 0, 65, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 0, 800, 1, hardwareParameter); //1300
	 goStraightCmPID_lib(30, 0, 65, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 1300, 2, hardwareParameter);  //1300
	goStraightCmPID_lib(150, 0, 75, MOVE_BACKWARD, 3.5, 0, 2.5, 0.5, 0, 5, 600, 1, hardwareParameter);  //1300


	////////////////////////////////////////////
	//  40) Place Blue goal on blue bridge
	///////////////////////////////////////////
	//goStraightCmPID_lib(120, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 2500, 1, hardwareParameter);

	//	goStraightCmPID_lib(60, 0, 100, MOVE_BACKWARD, 1.5, 0, 1, 0.5, 0, 5, 1000, 1, hardwareParameter);
 	 intakeAction_1 = {-20, 0, 200, 0, 1};
 	 angle = 95; //110;
	turnDegreesPID_lib(angle, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	//turnDegreesPID_lib(105, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter);

    //waitForTouch();

	intakeAction_1 = {-127, 0, 0, -127, 1};

	goStraightCmPID_lib(50, angle, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);  //65  60  54
	armAction_1 = {127, 0, _PRESS_BRIDGE, 1};
    delay(150);
	goStraightCmPID_lib(5, angle+6, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 300, 1, hardwareParameter);
    ////place blue mobile goal
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(5, angle+6, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, _PRESS_BRIDGE + 150, 1};

	 pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
    //waitForTouch();
	delay(200);
	//waitForTouch();

	//////////////////////////////////////////
	////  50) take middle high yellow goal
	//////////////////////////////////////////
    //goStraightCmPID_lib(12, 180, 100, MOVE_BACKWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	goStraightCmPID_lib(18, 180, 100, MOVE_BACKWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	intakeAction_1 = {-10, 0, 0, -10, 1};
	armAction_1 = {127, 200, -20, 1};
	//turnDegreesPID_lib(230, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	
	turnDegreesPID_lib(270, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	intakeAction_1 = {-127, 0, 0, -127, 1};
	//	armAction_1 = {127, 0, -20, 1};
	  delay(100);  //
 	 //new_goStraightCm_Front_Vision(40, 230, 70, DETECT_YELLOW_GOAL_SIG, front_vision,	0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 1000, 1, hardwareParameter);
 	 new_goStraightCm_Front_Vision(40, 270, 70, DETECT_YELLOW_GOAL_SIG, front_vision,
										0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 1000, 1, hardwareParameter);
	//	intakeAction_1 = {-10, 0, 0, -10, 1};
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 45, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 800, 1, hardwareParameter);
	delay(100);

	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, _PRESS_BRIDGE +130, 1};
	delay(400);
	intakeAction_1 = {-127, 00, 0, -127, 1};

    //waitForTouch();
	//move high yellow goal to red bridge
	angle = 270; //274;
	goStraightCmPID_lib(120, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 2500, 1, hardwareParameter);
	armAction_1 = {127, 0, _PRESS_BRIDGE - 100, 1};
	goStraightCmPID_lib(5.5, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 350, 1, hardwareParameter);
	delay(200);
////place high yellow mobile goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time+50);
	//goStraightCmPID_lib(5, angle, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 350, 1, hardwareParameter);
	goStraightCmPID_lib(12, angle, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 350, 1, hardwareParameter);  // dis:6
  //	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	armAction_1 = {127, 5, _PRESS_BRIDGE + 50, 1};   // delay from 0 to 5 ; was +70
	hookAction_1 = {0, false, 1};
	delay(200);

    ////////////////////////////////////////////////////
	// Place Rear Red goal on floorturn and get,  then place on red bridge
	////////////////////////////////////////////////////

	goStraightCmPID_lib(60, 270, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);   //50
	intakeAction_1 = {0, 0, 0, 0, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(100);

	goStraightCmPID_lib(42, 270, 100, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 10, 800, 1, hardwareParameter);  //32
	turnDegreesPID_lib(105, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0, 1200, 2, hardwareParameter);
	//turnDegreesPID_lib(90, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	
	//waitForTouch();
	//------------------
	// "See" and get the red goal
	//-----------------------
	goStraightCm_Front_Vision(50, 95, 55, DETECT_RED_GOAL_SIG, front_vision,
	  									0.5, 0, 1, 0.3, 0, 7, 0.3, 0, 1, 1000, 1, hardwareParameter);   //0.3   //50
	//goStraightCmPID_lib(40, 90, 55, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 1200, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
  	delay(claw_action_delay_time);



    //waitForTouch();

	armAction_1 = {127, 0, _PRESS_BRIDGE + 120, 1};
	delay(500);


	///////////////////////
	////place red mobile goal in red bridge
	///////////////////////
	angle = 255;  //245
	turnDegreesPID_lib(angle, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
  //turnDegreesPID_lib(angle, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 1.6, 0, -1, 1000, 2, hardwareParameter);
	goStraightCmPID_lib(120, angle, 100, MOVE_FORWARD, 3, 0, 1, 0.5, 0, 8, 1750, 1, hardwareParameter);  //angle  dis:105  140
	armAction_1 = {127, 300, _PRESS_BRIDGE, 1};
	//angle = get_robot_heading_lib(hardwareParameter);
	//turnDegreesPID_lib(260, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 800, 1, hardwareParameter);  //275  265
	//turnDegreesPID_lib(275, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 1.6, 0, -1, 800, 1, hardwareParameter);
	delay(300);

	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 270, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, _PRESS_BRIDGE + 100, 1};
	delay(200);
	goStraightCmPID_lib(12, 270, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 500, 1, hardwareParameter);

	 //////////////////
	// Go for the 2nd Blue Goal
	///////////////////
	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	//turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter); 				//shit
	armAction_1 = {127, 0, 0, 1};

	 //////////////////
	//waitForTouch();
	///////////////////

	// Move to Corner Blue
	//goStraightCmPID_lib(130, 182, 100, MOVE_BACKWARD, 2, 0, 1, 1, 0, 5, 1800, 1, hardwareParameter);  //120

	goStraightCmPID_lib(130, 173, 100, MOVE_BACKWARD, 2, 0, 1, 1, 0, 5, 1800, 1, hardwareParameter);  //120

	goStraightCm_Back_Vision(40, 173, 60, DETECT_BLUE_GOAL_SIG, back_vision,
		                       0.5, 0, 1, 0.5, 0, 5, 1.25, 0, 1, 1200, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	hookAction_1 = {300, true, 1};
	goStraightCmPID_lib(20, angle, 60, MOVE_BACKWARD, 1.5, 0, 1.5, 1, 0, 0, 320, 1, hardwareParameter);  //600


	//take corner blue goal
  	//hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);

	intakeAction_1 = {-127, 0, 0, -127, 1};

	//////////////////
	//waitForTouch();
	///////////////////

    ////////////////////////
	// Move to last Yellow Goal
	///////////////////
 	goStraightCmPID_lib(50, 180, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 5, 800, 1, hardwareParameter); //900
	turnDegreesPID_lib(90, ON_SPOT_TURN, 80, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);  //900
	goStraightCm_Front_Vision(30, 90, 90, DETECT_YELLOW_GOAL_SIG, front_vision,
										0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
  	angle = get_robot_heading_lib(hardwareParameter);
  goStraightCmPID_lib(30, angle, 50, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 800, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, _PRESS_BRIDGE + 200, 1};   //300


//	intakeAction_1 = {127, 300, 0, 127, 1};
	goStraightCmPID_lib(136, 120, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 2100, 1, hardwareParameter); //2000
	armAction_1 = {127, 0, _PRESS_BRIDGE, 1};
    delay(200);
	goStraightCmPID_lib(15, 140, 127, MOVE_FORWARD, 3.5, 0, 1, 0.4, 0, 0, 500, 1, hardwareParameter);

  //waitForTouch();

	////////////////push left
	//release third yellow goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 125, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, _PRESS_BRIDGE + 100, 1};
	delay(200);
	goStraightCmPID_lib(55, 120, 100, MOVE_BACKWARD, 3, 0, 1, 0.35, 0, 5, 1000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay(hook_action_delay_time);

	//////////////////
	// Get 2nd blue goal with front arm
	goStraightCmPID_lib(30, 120, 80, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 900, 2, hardwareParameter);
	turnDegreesPID_lib(300, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	goStraightCmPID_lib(40, 300, 60, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 1000, 1, hardwareParameter);

	delay(100);
	clawAction_1 = {0, true, 1};
  	delay(claw_action_delay_time);
	armAction_1 = {127, 0, _PRESS_BRIDGE + 250, 1};
	delay(400);
	turnDegreesPID_lib(120, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	//turnDegreesPID_lib(115, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, -1, 1200, 2, hardwareParameter); angle 115
	goStraightCmPID_lib(130, 115, 100, MOVE_FORWARD,	2, 0, 1, 0.3, 0, 0, 1500, 1, hardwareParameter);  // newly added
	armAction_1 = {127, 0, _PRESS_BRIDGE + 150, 1};
	goStraightCmPID_lib(10, 120, 127, MOVE_FORWARD,	4, 0, 1, 0.5, 0, 0, 400, 1, hardwareParameter);

//release last blue goal to the bridge
    delay(100);
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time + 150);

	//armAction_1 = {127, 300, 400, 1};
	///////////////////////
	// Move to last red goal
	armAction_1 = {127, 0, 00, 1};
	goStraightCmPID_lib(170, 170, 100, MOVE_BACKWARD,	2, 0, 1, 1.1, 0, 0, 2000, 1, hardwareParameter);   //30
	////////////////
	//waitForTouch();
	/////////////////
	goStraightCm_Front_Vision(100, 135, 90, DETECT_RED_GOAL_SIG, front_vision,
										0.5, 0, 1, 0.3, 0, 10, 0.7, 0, 0, 1300, 1, hardwareParameter);  //kp vision 0.4
	clawAction_1 = {0, true, 1};
	waitForTouch();
	goStraightCmPID_lib(1100, 90, 127, MOVE_FORWARD,	2, 0, 1, 1.1, 0, 0, 1000, 1, hardwareParameter); 
	//goStraightCmPID_lib(50, 115, 100, MOVE_BACKWARD,	2, 0, 1, 0.3, 0, 0, 1000, 1, hardwareParameter);   //30

	////////////////
	waitForTouch();
	/////////////////

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


/**************************
AUTON
**************************/
void autonomous()
{
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	intake_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

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
	red_right_default_NEW();
	//red_right_default();
	//red_two_goals();

	//encoder();
	
//skills code ------------------------------------------------------------------------------------------------------------------
	//auton_60s_skills();

	waitForTouch();
}
