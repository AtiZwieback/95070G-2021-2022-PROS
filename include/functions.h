#include "../include/main.h"
#include "../include/motorSetup.h"

okapi::Controller master;
//Useful Constants
const double wheelCircumfrence = 2.75 * M_PI;

int selected = 0;
std::string autons[9] = {"Disabled", "Robot_inspection", "SentretBot", "AWP_L", "AWP_R", "TEST_1goal", "TEST_2right_go", "TEST_2right_backup", "rev12"};
int size = 9;//*(&autons + 1) - autons;

void pistonextend(){
  piston.set_value(true);
}

void pistonretract(){
  piston.set_value(false);
}

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

void driverControl1(double l1, double r1){
  FrontLeft.move_velocity(l1);
  FrontRight.move_velocity(r1);
  BackLeft.move_velocity(l1);
  BackRight.move_velocity(r1);
}

void driverControl2(double l2, double r2){
  FrontLeft.move_velocity(l2);
  FrontRight.move_velocity(r2);
  BackLeft.move_velocity(l2);
  BackRight.move_velocity(r2);
}


void bliftmove(double speed){
  BLLift.move(speed);
  BRLift.move(speed);
}

void bliftmoverelative(double encoderTicks, double speed){
  BLLift.move_relative(encoderTicks,speed);
  BRLift.move_relative(encoderTicks,speed);
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
