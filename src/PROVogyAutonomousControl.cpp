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
	armAction_1 = {127, 0, 20, 1};
	clawAction_1 = {1050, true, 1};
	goStraightCmPID_lib(116, 90, 127, MOVE_FORWARD, 5, 0, 0, 0.75, 0, 5, 2000, 1, hardwareParameter);
	delay(300);
	goStraightCmPID_lib(120, 75, 127, MOVE_BACKWARD, 4, 0, 2, 1, 0, 0, 15000, 2, hardwareParameter);
	armAction_1 = {127, 0, -150, 1};
	turnDegreesPID_lib(155, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 1.6, 0, -1, 1500, 3, hardwareParameter);

	goStraightCm_Back_Vision(45, 155, 100, DETECT_RED_GOAL_SIG, back_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);

	hookAction_1 = {0, true, 1};
	intakeAction_1 = {-90, 0, 1000, -90, 1};
	goStraightCmPID_lib(55, 155, 127, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 1500, 2, hardwareParameter);
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(15, 160, 127, MOVE_BACKWARD, 4, 0, 2, 0.75, 0, 5, 400, 2,hardwareParameter);
	turnDegreesPID_lib(52, ON_SPOT_TURN, 100, CLOCKWISE, 1.6, 0, -1, 1500, 2, hardwareParameter);
	armAction_1 = {127, 1000, -150, 1};
	goStraightCm_Front_Vision(140, 52, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
															0.5, 0, 1, 0.3, 0, 10, 0.5, 0, 0, 1500, 1, hardwareParameter);   //100
	currentInertia = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(10, currentInertia, 55, MOVE_FORWARD, 4, 0, 2, 0.75, 0, 5, 500, 2, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(100);
	intakeAction_1 = {0, 0, 0, 0, 1};
	armAction_1 = {127, 0, -200, 1};
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
	//red_left_high_score();
	//red_win_point();
	//blue_right_high_score();
	//blue_middle_high_score();
	//blue_left_high_score();
	//blue_win_point();

	red_left_FINALS();
	//blue_left_FINALS ();
	//red_middle_FINALS ();

}
