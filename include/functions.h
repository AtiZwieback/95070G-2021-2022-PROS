#include "../include/main.h"
#include "../include/motorSetup.h"

okapi::Controller master;
//Useful Constants
const double wheelCircumfrence = 2.75 * M_PI;


std::vector<std::string> autons{"Disabled", "Robot_inspection", "SentretBot", "AWP_L", "AWP_R", 
                          "Auto_go_mid", "Auto_go_front", "Auto_turn_left", "Auto_go_front_2Inx"};
int size = autons.size();//*(&autons + 1) - autons;
int selected = 7 < autons.size()? 7 : autons.size()-1;
void autonSelector(){
  master.clear();
  pros::delay(200);
  while(true){
    master.clear();
    pros::delay(100);
    control.print(2, 1, autons[selected].c_str());
    pros::delay(100);
     if(control.get_digital(DIGITAL_RIGHT)){
       selected = (selected + 1 + size) % size;
     }else if(control.get_digital(DIGITAL_LEFT)){
       selected = (selected - 1 + size) % size;
     }else if(control.get_digital(DIGITAL_A)){
       pros::delay(200);
       if(control.get_digital(DIGITAL_A)){
         goto slctEnd;
       }
     }
   }
   slctEnd:
   master.rumble("..");
}

void driverControl(double l, double r){
  //Calculates speed of wheels for driver control

	FrontLeft.move_velocity(l);
	FrontRight.move_velocity(r);
	BackLeft.move_velocity(l);
	BackRight.move_velocity(r);
}

void bliftmove(double speed){
  BLLift.move(speed);
  BRLift.move(speed);
}

void bliftmove_relative(double encoderTicks, double speed){
  BLLift.move_relative(encoderTicks,speed);
  BRLift.move_relative(encoderTicks,speed);
}

void bliftmove_absolute(double abs_pos, double speed){
  BLLift.move_absolute(abs_pos, speed);
  BRLift.move_absolute(abs_pos, speed);
}

void fourbarmove(double speed){
  FBarR.move_velocity(speed);
  FBarL.move_velocity(speed);
}

void fourbarmoverelative(double encoderTicks, double speed){
  FBarR.move_relative(encoderTicks, speed);
  FBarL.move_relative(encoderTicks, speed);
}

void fourbarmoveabsolute(double position, double speed){
  FBarR.move_absolute(position, speed);
  FBarL.move_absolute(position, speed);
}

void BotMoveAboslute(double pos, int velocity){
	FrontLeft.move_absolute(pos, velocity);
	FrontRight.move_absolute(pos, velocity);
	BackLeft.move_absolute(pos, velocity);
	BackRight.move_absolute(pos, velocity);
}

void BotMoveRelative(double pos, int velocity){
	FrontLeft.move_relative(pos, velocity);
	FrontRight.move_relative(pos, velocity);
	BackLeft.move_relative(pos, velocity);
	BackRight.move_relative(pos, velocity);
}

// + pos -> Turn left
void BotTurnAboslute(double pos, int velocity){
	FrontLeft.move_absolute(pos, velocity);
	BackLeft.move_absolute(pos, velocity);
	FrontRight.move_absolute(-pos, velocity);
	BackRight.move_absolute(-pos, velocity);
}

// + pos -> Turn left
void BotTurnRelative(double pos, int velocity){
	FrontLeft.move_relative(pos, velocity);
	BackLeft.move_relative(pos, velocity);
	FrontRight.move_relative(-pos, velocity);
	BackRight.move_relative(-pos, velocity);
}

/*
//For debugging things
void printOnScreen(){
	//lcd::print(1, "Velocity FL: %f", FrontLeft.get_actual_velocity());
	//lcd::print(2, "Target Velocity FL: %f", drive.wheelTL);
  pros::lcd::print(0, "Inertial Reading: %f", inertial.get_rotation());
  pros::lcd::print(1, "Y Wheel Reading: %f", ((double) yWheel.get_value()));
  pros::lcd::print(2, "X Wheel Reading: %f", ((double) xWheel.get_value()));
}

void driverControl(double l, double r){
  //Calculates speed of wheels for driver control

	FrontLeft.move_velocity(l);
	FrontRight.move_velocity(r);
	BackLeft.move_velocity(l);
	BackRight.move_velocity(r);
}


void stopDrive(bool hold = false){
  //Shortcut to stop the drive quickly
  if(hold){
    FrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    FrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    BackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    BackRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  FrontLeft.move_velocity(0);
	FrontRight.move_velocity(0);
	BackLeft.move_velocity(0);
	BackRight.move_velocity(0);
  delay(100);
  FrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  FrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  BackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  BackRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
*/
