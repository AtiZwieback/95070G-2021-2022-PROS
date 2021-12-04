#include "main.h"

using namespace pros;

const double ticksPerDeg = 900/360;
//200*4.125*pi/60 * sqrt(2), = 60
const double robotSpeed = 43.196 * sqrt(2);
const double rotationSpeed = 200;

extern pros::Motor FrontLeft;
extern pros::Motor FrontRight;
extern pros::Motor BackLeft;
extern pros::Motor BackRight;


//Ports
const int FLPort = 13;
const int FRPort = 14;
const int BLPort = 15;
const int BRPort = 16;
const int FBLPort = 3;
const int FBRPort = 4;
const int BRLPort = 8;
const int BLLPort = 20;


pros::Controller control (pros::E_CONTROLLER_MASTER);

ControllerButton RUp(ControllerDigital::R1);
ControllerButton RDown(ControllerDigital::R2);
ControllerButton LUp(ControllerDigital::L1);
ControllerButton LDown(ControllerDigital::L2);
ControllerButton A(ControllerDigital::A);
ControllerButton B(ControllerDigital::B);
ControllerButton X(ControllerDigital::X);
ControllerButton Y(ControllerDigital::Y);
ControllerButton left(ControllerDigital::left);
ControllerButton right(ControllerDigital::right);
ControllerButton up(ControllerDigital::up);
ControllerButton down(ControllerDigital::down);
/*
pros::Controller control (E_CONTROLLER_MASTER);*/

pros::Motor FrontLeft(FLPort, false);
pros::Motor BackLeft(BLPort, false);
pros::Motor FrontRight(FRPort, true);
pros::Motor BackRight(BRPort, true);
pros::Motor FBarR(FBRPort, false);
pros::Motor FBarL(FBLPort, true);
pros::Motor BLLift(BLLPort, true);
pros::Motor BRLift(BRLPort, false);
pros::ADIDigitalOut piston ('A');
/*
pros::Motor LeftIntake(LIPort, false);
pros::Motor RightIntake(RIPort, true);
pros::Motor BackRoller(LRPort, true);
pros::Motor MainRoller(RRPort, true);
pros::Imu inertial(IMUPort);

ADIEncoder yWheel('C', 'D', true);
ADIEncoder xWheel('A', 'B', false);


//Calibrate the sensors
void calibrateSensors(){
  xWheel.reset();
  yWheel.reset();
  inertial.reset();

  int timeInit = pros::millis();

  inertial.reset();
  while(inertial.is_calibrating()){
    lcd::print(1, "Calibrating");
    delay(10);
  }
  delay(2000);
  lcd::print(1, "Calibration took %f", millis() - timeInit);
}*/
