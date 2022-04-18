#include "../include/main.h"
#include "PROVogyConfiguration.hpp"

using namespace pros;

#define PRE 2
#define CATCH 1
#define RELEASE 0
#define ON 1
#define OFF -1

bool seeYellow = false;
int currentInertia;

// go straight parameters

//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.35, 0, 26, 6000, 3, hardwareParameter);
//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.35, 0, 24, 6000, 2, hardwareParameter);
//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.3, 0, 28, 6000, 1, hardwareParameter);

// goStraightCmPID_lib(60.0, 90, 127, MOVE_BACKWARD, 5, 0, 3, 0.40, 0, 1, 2000, 2, hardwareParameter);

// left is positive, right is negative

//GRAB MIDDLE YELLOW LEFT PERFECT
//goStraightCmPID_lib(105, 0, 127, MOVE_FORWARD, 2, 0, 0, 2, 0, 10, 1500, 1, hardwareParameter);
//clawAction_1 = {0, true, 1};

Point curPos;
double distance;
double angle;

/***********************************************************************************************

AUTONOMOUS PROGRAMS

***********************************************************************************************/

//////////////////
//RED SIDE AUTONS/
//////////////////

void red_right_high_score()
{
	vision_object_s_t closest_goal;

	armAction_1 = {127, 0, 20, 1};
	goStraightCmPID_lib(105, 0, 127, MOVE_FORWARD, 4, 0, 3, 0.75, 0, 5, 2000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(300);
	goStraightCmPID_lib(95, 0, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(-135, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, -1, 1000, 3, hardwareParameter);
	goStraightCmPID_lib(20, -135, 127, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);
	clawAction_1 = {0, false, 1};
	delay(50);
	goStraightCmPID_lib(40, -135, 127, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);
	turnDegreesPID_lib(30, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 3, hardwareParameter);
	armAction_1 = {127, 0, 20, 1};
	goStraightCmPID_lib(35, 30, 100, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);
	//checking for yellow width

	/*for (int i = 0; i < 10; i++)
	{
		closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
		delay(10);

		if (closest_goal.width >= 60)
			break;
	}*/

  if (true) { //you see middle yellow
		goStraightCm_Front_Vision(55, 40, 127, DETECT_YELLOW_GOAL_SIG, front_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.5, 0, 0, 1500, 1, hardwareParameter);
		currentInertia = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, currentInertia, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		clawAction_1 = {0, true, 1};
		armAction_1 = {127, 0, -150, 1};
		goStraightCmPID_lib(80, 40, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
		turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 1, hardwareParameter);
		goStraightCm_Back_Vision(65, 90, 127, DETECT_RED_GOAL_SIG, back_vision,
	                           0.5, 0, 1, 0.3, 0, 5, 0.5, 0, 0, 1500, 1, hardwareParameter);
		currentInertia = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, currentInertia, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		hookAction_1 = {0, true, 1};
		intakeAction_1 = {-75, 0, 2000, -75, 1};
		goStraightCmPID_lib(60, 90, 100, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);

	} else { //you dont see middle yellow
		turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 1, hardwareParameter);
		goStraightCm_Back_Vision(90, 90, 127, DETECT_RED_GOAL_SIG, back_vision,
	                           0.5, 0, 1, 0.3, 0, 5, 0.4, 0, 0, 1500, 1, hardwareParameter);
		currentInertia = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, currentInertia, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		hookAction_1 = {0, true, 1};
		goStraightCmPID_lib(30, 90, 100, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);
		turnDegreesPID_lib(0, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, -1, 700, 2, hardwareParameter);
		armAction_1 = {127, 0, -150, 1};
		intakeAction_1 = {-75, 0, 2000, -75, 1};
		goStraightCmPID_lib(90, 0, 75, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 2000, 2, hardwareParameter);
		intakeAction_1 = {0, 0, 0, 0, 1};
		delay(50);
		goStraightCmPID_lib(110, 0, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);


	}

}

void red_middle_high_score()
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
	currentInertia = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(10, currentInertia, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
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
		currentInertia = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, currentInertia, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
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

void red_left_high_score()
{
	sys_initial_robot_heading = 90;
	armAction_1 = {127, 0, 15, 1};	
	clawAction_1 = {1050, true, 1};
	goStraightCmPID_lib(115, 90, 127, MOVE_FORWARD, 5, 0, 0, 0.75, 0, 5, 2000, 1, hardwareParameter);
	waitForTouch();
	delay(300);
	goStraightCmPID_lib(120, 75, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(155, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 1.6, 0, -1, 1500, 3, hardwareParameter);
	waitForTouch();
	goStraightCm_Back_Vision(45, 155, 100, DETECT_RED_GOAL_SIG, back_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);

	hookAction_1 = {0, true, 1};
	intakeAction_1 = {90, 0, 1000, 90, 1};
	goStraightCmPID_lib(55, 155, 127, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter);
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(15, 160, 127, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 400, 2,hardwareParameter);
	armAction_1 = {127, 0, -20, 1};
	turnDegreesPID_lib(52, ON_SPOT_TURN, 100, CLOCKWISE, 1.6, 0, -1, 1500, 2, hardwareParameter);
	goStraightCm_Front_Vision(170, 52, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.5, 0, 0, 1500, 1, hardwareParameter);   //100
	currentInertia = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(10, currentInertia, 55, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(100);
	intakeAction_1 = {0, 0, 0, 0, 1};
	armAction_1 = {127, 0, 0, 1};
	goStraightCmPID_lib(120, 52, 127, MOVE_BACKWARD, 4, 0, 2, 0.5, 0, 5, 1500, 1, hardwareParameter);
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
  currentInertia = get_robot_heading_lib(hardwareParameter);
  goStraightCmPID_lib(10, currentInertia, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
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

///////////////////
//BLUE SIDE AUTONS/
///////////////////

void blue_right_high_score()
{
	vision_object_s_t closest_goal;

	armAction_1 = {127, 0, 20, 1};
	goStraightCmPID_lib(105, 0, 127, MOVE_FORWARD, 4, 0, 3, 0.75, 0, 5, 2000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(300);
	goStraightCmPID_lib(95, 0, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(-135, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, -1, 1000, 3, hardwareParameter);
	goStraightCmPID_lib(20, -135, 127, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);
	clawAction_1 = {0, false, 1};
	delay(50);
	goStraightCmPID_lib(40, -135, 127, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);
	turnDegreesPID_lib(35, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 3, hardwareParameter);
	armAction_1 = {127, 0, 20, 1};
	goStraightCmPID_lib(35, 40, 100, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1000, 2, hardwareParameter);

	//checking for yellow width
	closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
  std::cout << "Yellow width: " << closest_goal.width << std::endl;

  if (true) { //you see middle yellow
		goStraightCm_Front_Vision(55, 40, 127, DETECT_YELLOW_GOAL_SIG, front_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
		currentInertia = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, currentInertia, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		clawAction_1 = {0, true, 1};
		armAction_1 = {127, 0, -150, 1};
		goStraightCmPID_lib(80, 40, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
		turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 1, hardwareParameter);
		goStraightCm_Back_Vision(65, 90, 127, DETECT_BLUE_GOAL_SIG, back_vision,
	                           0.5, 0, 1, 0.3, 0, 5, 0.5, 0, 0, 1500, 1, hardwareParameter);
		currentInertia = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, currentInertia, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		hookAction_1 = {0, true, 1};
		intakeAction_1 = {-75, 0, 2000, -75, 1};
		goStraightCmPID_lib(60, 90, 100, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);

	} else { //you dont see middle yellow
		turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 700, 1, hardwareParameter);
		goStraightCm_Back_Vision(90, 90, 127, DETECT_BLUE_GOAL_SIG, back_vision,
	                           0.5, 0, 1, 0.3, 0, 5, 0.4, 0, 0, 1500, 1, hardwareParameter);
		currentInertia = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, currentInertia, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
		hookAction_1 = {0, true, 1};
		goStraightCmPID_lib(30, 90, 100, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);
		turnDegreesPID_lib(0, ON_SPOT_TURN, 127, CLOCKWISE, 1.6, 0, -1, 700, 2, hardwareParameter);
		armAction_1 = {127, 0, -150, 1};
		intakeAction_1 = {-75, 0, 2000, -75, 1};
		goStraightCmPID_lib(90, 0, 75, MOVE_FORWARD, 4, 0, 2, 0.5, 0, 5, 2000, 2, hardwareParameter);
		intakeAction_1 = {0, 0, 0, 0, 1};
		delay(50);
		goStraightCmPID_lib(110, 0, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 5, 1000, 2, hardwareParameter);


	}

}

void blue_middle_high_score()
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
	currentInertia = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(10, currentInertia, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
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
		currentInertia = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(10, currentInertia, 50, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
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

void blue_left_high_score()
{
	sys_initial_robot_heading = 90;
	armAction_1 = {127, 0, 20, 1};
	clawAction_1 = {1050, true, 1};
	goStraightCmPID_lib(116, 90, 127, MOVE_FORWARD, 5, 0, 0, 0.75, 0, 5, 2000, 1, hardwareParameter);
	delay(300);
	goStraightCmPID_lib(120, 75, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(155, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 1.6, 0, -1, 1500, 3, hardwareParameter);

	goStraightCm_Back_Vision(45, 155, 100, DETECT_BLUE_GOAL_SIG, back_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);

	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-100, 0, 1000, -100, 1};
	goStraightCmPID_lib(52, 155, 127, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter);
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(15, 160, 127, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 400, 2,hardwareParameter);
	turnDegreesPID_lib(52, ON_SPOT_TURN, 100, CLOCKWISE, 1.6, 0, -1, 1500, 2, hardwareParameter);
	goStraightCmPID_lib(140, 52, 80, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter);
	armAction_1 = {127, 0, -200, 1};
	goStraightCmPID_lib(120, 52, 127, MOVE_BACKWARD, 4, 0, 2, 0.5, 0, 5, 1500, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	hookAction_1 = {0, false, 1};
}

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
  currentInertia = get_robot_heading_lib(hardwareParameter);
  goStraightCmPID_lib(10, currentInertia, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
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

////////////////
//FINALS AUTONS/
////////////////

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
	currentInertia = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(10, currentInertia, 50, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-75, 0, 2000, -75, 1};
	goStraightCmPID_lib(60, 180, 127, MOVE_FORWARD, 4, 0, 2, 1, 0, 5, 3000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	clawAction_1 = {0, false, 1};
}

void auton_60s_skills_slow_version(){
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;   //was 180
	long hook_action_delay_time = 50;
	long claw_action_delay_time = 75;
	long start_time = pros::millis();

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};
	delay(50);
	arm_motor.move(-60);
	delay(100);
	arm_motor.move(0);
	//Test
	/*
 intakeAction_1 = {-90, 0, 0, -90, 1};
	waitForTouch();
intakeAction_1 = {10, 0, 0, 10, 1};
arm_motor.move(60);
delay(100);
arm_motor.move(0);
	waitForTouch();
	intakeAction_1 = {0, 0, 0,0, 1};
	pros::lcd::print(2, "Hello Here");
delay(99999);  */

// ------------------------------------------------------------------------------------ //
//// part 1 take first blue goal
	goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
//	delay(200);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);

	intakeAction_1 = {-90, 400, 0, -90, 1};
	goStraightCmPID_lib(90, 80, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter); //100,80,127
  arm_motor.move(0);
  arm_motor.tare_position();


  // Move to 1st yellow goal
	// goStraightCm_Front_Vision(25, 80, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
	//  	                  0.5, 0, 1, 0.5, 0, 7, 1, 0, 1, 700, 1, hardwareParameter); 										0.5, 0, 1, 0.5, 0, 15, 0.25, 0, 1, 600, 1, hardwareParameter);
	new_goStraightCm_Front_Vision(30, 80, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
    										0.5, 0, 1, 0.5, 0, 15, 0.25, 0, 1, 600, 1, hardwareParameter);
  angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(20, angle, 60, MOVE_FORWARD, 1.5, 0, 1.5, 1, 0, 0, 600, 1, hardwareParameter);

  // Take 1st yellow goal
	// new_goStraightCm_Front_Vision(40, 80, 70, DETECT_YELLOW_GOAL_SIG, front_vision,
  //   										0.5, 0, 1, 0.3, 0, 4, 0.65, 0, 4, 1000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 5, RELEASE_BRIDGE + 100, 1};
	delay(100);
//	intakeAction_1 = {90, 400, 0, 90, 1};
	//delay(500);

 // Move to blue bridge with 1st yellow goal
	goStraightCmPID_lib(150, 61, 95, MOVE_FORWARD, 3.5, 0, 2.5, 0.2, 0, 5, 1550, 1, hardwareParameter); //1650
	armAction_1 = {127, 0, PRESS_BRIDGE - 25, 1};
	delay(250);

	goStraightCmPID_lib(1, 62, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 500, 1, hardwareParameter);  //15
	clawAction_1 = {0, false, 1};  //runtime 16?
	intakeAction_1 = {0, 0, 0, 0, 1};


////place first yellow goal
//	delay(300);
////take first blue goal by front claw
	goStraightCmPID_lib(6, 63, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 300, 1, hardwareParameter);  //5
	armAction_1 = {127, 5, PRESS_BRIDGE + 100, 1};
	delay(150);
	goStraightCmPID_lib(70, 55, 127, MOVE_BACKWARD, 5, 0, 2.5, 0.5, 0, 5, 1200, 1, hardwareParameter);


//	intakeAction_1 = {0, 0, 0, 0, 1};
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(hook_action_delay_time);
	goStraightCmPID_lib(28, 55, 100, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 700, 1, hardwareParameter);
	turnDegreesPID_lib(235, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	//turnDegreesPID_lib(235, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0, 1200, 3, hardwareParameter);
	goStraightCmPID_lib(28, 235, 60, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 700, 1, hardwareParameter);

//  goStraightCm_Front_Vision(30, 239, 100, DETECT_BLUE_GOAL_SIG, front_vision,
//	  									0.5, 0, 1, 0.3, 0, 7, 0.3, 0, 1, 1000, 1, hardwareParameter);

	delay(100);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 0, 250, 1};
	delay(100);
	goStraightCmPID_lib(75, 360, 127, MOVE_BACKWARD, 1.0, 0, 1.2, 0.7, 0, 10, 1300, 1, hardwareParameter); //1500
	goStraightCm_Back_Vision(10, 360, 80, DETECT_RED_GOAL_SIG, back_vision,
		                       0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 500, 1, hardwareParameter);
  angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(25, angle, 80, MOVE_BACKWARD, 3.5, 0, 2.5, 1, 0, 0, 800, 1, hardwareParameter);
////take first red goal by back claw
//  delay(100);
  hookAction_1 = {0, true, 1};
	armAction_1 = {127, 5, PRESS_BRIDGE + 300, 1};
	intakeAction_1 = {-127, 0, 0, -127, 1};
	delay(100);
//waitForTouch();
	goStraightCmPID_lib(102, 348, 90, MOVE_FORWARD, 3.5, 0, 2.5, 1, 0, 0, 2000, 1, hardwareParameter);
//	goStraightCmPID_lib(45, 0, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.3, 0, 0, 1000, 1, hardwareParameter); //1300
//	goStraightCmPID_lib(55, 0, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 1300, 2, hardwareParameter);  //1300
	 goStraightCmPID_lib(35, 0, 65, MOVE_FORWARD, 2.5, 0, 0, 1, 0, 0, 800, 1, hardwareParameter); //1300
	 goStraightCmPID_lib(35, 0, 65, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 0, 800, 1, hardwareParameter); //1300
	 goStraightCmPID_lib(30, 0, 65, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 1300, 2, hardwareParameter);  //1300
	goStraightCmPID_lib(20, 0, 75, MOVE_BACKWARD, 3.5, 0, 2.5, 0.5, 0, 5, 600, 1, hardwareParameter);  //1300



	//goStraightCmPID_lib(120, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 2500, 1, hardwareParameter);

//	goStraightCmPID_lib(60, 0, 100, MOVE_BACKWARD, 1.5, 0, 1, 0.5, 0, 5, 1000, 1, hardwareParameter);
  intakeAction_1 = {-20, 0, 200, 0, 1};
  angle = 113;
	turnDegreesPID_lib(angle, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	//turnDegreesPID_lib(105, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter);

	intakeAction_1 = {-127, 0, 0, -127, 1};
	goStraightCmPID_lib(55, angle, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
  //armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	armAction_1 = {127, 0, PRESS_BRIDGE-70, 1};
  delay(100);
	goStraightCmPID_lib(5, angle, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 300, 1, hardwareParameter);
////place blue mobile goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, angle, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 150, 1};
	delay(200);
//	delay(99999);

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
	goStraightCm_Back_Vision(distance + 5, 345, 50, DETECT_RED_GOAL_SIG, back_vision,
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
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	intake_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	left_back_motor.set_zero_position(0.0);
	left_front_motor.set_zero_position(0.0);
	left_mid_motor.set_zero_position(0.0);
	right_back_motor.set_zero_position(0.0);
	right_front_motor.set_zero_position(0.0);
	right_mid_motor.set_zero_position(0.0);
	intake_motor.set_zero_position(0.0);
	arm_motor.set_zero_position(0.0);

	// this line records the inertial sensor drifting between initialization and start of auton.
	sys_initial_to_auton_drifting = hardwareParameter.inertialSensorLib.get_rotation();

	/***********************************************
	CHOOSE RUN HERE
	***********************************************/


	//red_right_high_score(
	//red_middle_high_score();
	red_left_high_score();
	//red_win_point();
	//blue_right_high_score();
	//blue_middle_high_score();
	//blue_left_high_score();
	//blue_win_point();

	//red_left_FINALS();
	//blue_left_FINALS ();
	//red_middle_FINALS ();

}
