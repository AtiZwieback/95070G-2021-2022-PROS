#include "../include/main.h"
#include  "../include/autonomous.h"

#include <vector>
#include <algorithm>

void leftBtn(){

}
void centerBtn(){

}
void rightBtn(){

}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(leftBtn);
	pros::lcd::register_btn1_cb(centerBtn);
	pros::lcd::register_btn2_cb(rightBtn);
	FBarL.set_brake_mode(MOTOR_BRAKE_HOLD);
	FBarR.set_brake_mode(MOTOR_BRAKE_HOLD);
	BRLift.set_brake_mode(MOTOR_BRAKE_HOLD);
	BLLift.set_brake_mode(MOTOR_BRAKE_HOLD);
	piston.set_value(true);
	//set to false if default position is the same

  autonSelector();
	//autonSelector();
}


void disabled() {}

void competition_initialize() {}

void autonomous() {
  switch(selected){
    case 0:
		disabledAuton();
		break;
    case 1:
	    Robot_inspection();
		break;
    case 2:
		Drive();
		break;
    case 3:
		AWP1();
		break;
    case 4:
		AWP2();
		break;
    case 5:
		AutoGoMid();
		break;
    case 6:
		AutoGoFront();
		break;
    case 7:
		AutoTurnLeft();
		break;
	case 8:
		AutoGoFront1In();
		break;
	default:
		return;
   }
}

const int NUM_HEIGHTS = 3;
const int height1 = 0;
const int height2 = 700;
const int height3 = 1500;

const int heights[NUM_HEIGHTS] = {height1, height2,height3};
const int heights2[NUM_HEIGHTS] = {0, 700,1800};

constexpr int BACK_LIFT_VAL = 120;

int x = 0;

double GetMaxTemperature() {
	std::vector<double> temps;
	temps.push_back(FrontLeft.get_temperature());
	temps.push_back(BackLeft.get_temperature());
	temps.push_back(BackRight.get_temperature());
	temps.push_back(BackLeft.get_temperature());
	temps.push_back(FBarL.get_temperature());
	temps.push_back(FBarR.get_temperature());
	temps.push_back(BLLift.get_temperature());
	temps.push_back(BRLift.get_temperature());
	double max_temp = *max_element(temps.begin(), temps.end());
	std::string s = "MaxTemp: " + std::to_string(max_temp);
	control.print(0, 1, "MaxTemp: %.3f", max_temp);

	 pros::lcd::print(0, "MaxTemp: %.3f", max_temp);
//   std::string bllift_pos = std::to_string(BLLift.get_position());
//   std::string brlift_pos = std::to_string(BRLift.get_position());
//   std::string left = "BLift L: " + bllift_pos;
//   std::string right = "BLift R: "+ brlift_pos;
	pros::lcd::print(1, "BLift L: %.3f", BLLift.get_position());
	pros::lcd::print(2, "BLift R: %.3f", BRLift.get_position());
  control.print(1, 1, "BLift L: %.3f", BLLift.get_position());
  control.print(2, 1, "BLift R: %.3f", BRLift.get_position());
  return max_temp;
}

void GetBLiftPosition() {

}

void opcontrol() {
	master.clear();
	control.clear();
	BRLift.set_brake_mode(MOTOR_BRAKE_HOLD);
	BLLift.set_brake_mode(MOTOR_BRAKE_HOLD);
	FBarL.set_brake_mode(MOTOR_BRAKE_HOLD);
	FBarR.set_brake_mode(MOTOR_BRAKE_HOLD);
  //piston.set_value(true);
  	int goalHeight = 0;
	int bGoalHeight = 0;
	double prevr = 0;
	double prevl = 0;
	int back_lift_offset = 0;
  while (true){
	GetMaxTemperature();
	double power = -control.get_analog(ANALOG_LEFT_Y);
	double turn = -control.get_analog(ANALOG_LEFT_X);
	driverControl(2*power+turn, 2*power - turn);
	if (control.get_digital(E_CONTROLLER_DIGITAL_X)){
		piston.set_value(false);
	}
	if (control.get_digital(E_CONTROLLER_DIGITAL_B)){
		piston.set_value(true);
	}


    if (RUp.changedToPressed() && goalHeight < NUM_HEIGHTS - 1) {
      goalHeight++;
      liftControl->setTarget(heights[goalHeight]);
    } else if (RDown.changedToPressed() && goalHeight > 0) {
      goalHeight--;
      liftControl->setTarget(heights[goalHeight]);
    }
	if (control.get_digital(E_CONTROLLER_DIGITAL_L1)) {
      fourbarmove(120);
    } else if (control.get_digital(E_CONTROLLER_DIGITAL_L2)) {
      fourbarmove(-90);
    } else {
		fourbarmove(0);
	}

	if(LUp.changedToPressed() && bGoalHeight < NUM_HEIGHTS-1){
		bGoalHeight++;
		liftControl->setTarget(heights[bGoalHeight]);
	} else if (LDown.changedToPressed() && bGoalHeight > 0){
		bGoalHeight--;
		liftControl->setTarget(heights[bGoalHeight]);
	}
	if (control.get_digital(E_CONTROLLER_DIGITAL_R1)){
		bliftmove(-BACK_LIFT_VAL);
	} else if (control.get_digital(E_CONTROLLER_DIGITAL_R2)) {
		bliftmove(BACK_LIFT_VAL);
	} else {
		bliftmove(0);
	}
	pros::delay(20);
  }
}
