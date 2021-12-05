#include "../include/main.h"
#include "../include/functions.h"

////////////////////////////////////////////
//DO THE GEAR RATIO THINGY
////////////////////////////////////////////////
std::shared_ptr<AsyncPositionController<double, double>> liftControl =
    AsyncPosControllerBuilder().withMotor({BRPort,BLPort}).build();
std::shared_ptr<AsyncPositionController<double, double>> fourbar =
    AsyncPosControllerBuilder().withMotor({FBRPort,FBLPort}).build();

//This file has all of the autonomous
void disabledAuton(){

}

void Robot_inspection(){
  std::shared_ptr<ChassisController> driveauton =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withGains(
  {0.002, 0, 0.0001}, // Distance controller gains
  {0.001, 0, 0.0001} // Turn controller gains
  )
  .withMaxVelocity(200)

  .withDerivativeFilters(
        std::make_unique<AverageFilter<3>>()
    )
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .withOdometry() // use the same scales as the chassis (above)
  .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)

  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController2 =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();
    driveauton->moveDistance(25_in);


}
void skills(){
  std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();

  // Target location path
  profileController->generatePath({
      {64_in, 0_in, 0_deg},
      {0_ft, 0_ft, 0_deg}},
      "first_move"
  );
  profileController->generatePath({
      {0_in, 0_in, 0_deg},
      {30_in, 0_in, 0_deg}},
      "retreat"
  );

  profileController->setTarget("first_move",true);
  delay(1500);
  bliftmove(100);
  delay(500);
  profileController->setTarget("retreat",true);
  delay(1000);
}


void Drive(){

  std::shared_ptr<ChassisController> driveauton =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withGains(
  {0.002, 0, 0.0001}, // Distance controller gains
  {0.001, 0, 0.0001} // Turn controller gains
  )
  .withMaxVelocity(200)

  .withDerivativeFilters(
        std::make_unique<AverageFilter<3>>()
    )
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .withOdometry() // use the same scales as the chassis (above)
  .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)

  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();
profileController->generatePath({
      {0_ft, 0_ft, 0_deg},
      {40_in, 0_ft, 0_deg}},
      "Gotonumogo"
    );
    profileController->generatePath({
          {0_ft, 0_ft, 0_deg},
          {40_in, 0_ft, 0_deg}},
          "return"
        );
//driveauton->moveDistance(40_in);
profileController->setTarget("Gotonumogo");
delay(1600);
bliftmove(100);
delay(100);
profileController->generatePath({
      {0_ft, 0_ft, 0_deg},
      {35_in, 0_ft, 0_deg}},
      "Gotoamogo"
    );
profileController->setTarget("Gotoamogo",true);
delay(800);
bliftmove(100);
delay(100);
delay(15000);
}

void AWP1(){

    std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)

    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

    std::shared_ptr<ChassisController> driveautonnotpid =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withMaxVelocity(200)

    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
    .build(); // build an odometry chassis

    std::shared_ptr<AsyncMotionProfileController> profileController2 =
    AsyncMotionProfileControllerBuilder()
      .withLimits({
        1.0, // Maximum linear velocity of the Chassis in m/s
        2.0, // Maximum linear acceleration of the Chassis in m/s/s
        10.0 // Maximum linear jerk of the Chassis in m/s/s/s
      })
      .withOutput(driveauton)
      .buildMotionProfileController();
      profileController2->generatePath({
          {0_in, 0_in, 0_deg},
          {10_in, 0_ft, 0_deg}},
          "BOOOOOOM"
      );
profileController2->setTarget("BOOOOOOM", true);
delay(2000);
profileController2->setTarget("BOOOOOOM");
delay(15000);
}
void AWP2(){

  std::shared_ptr<ChassisController> driveauton =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withGains(
  {0.002, 0, 0.0001}, // Distance controller gains
  {0.001, 0, 0.0001} // Turn controller gains
  )
  .withMaxVelocity(200)

  .withDerivativeFilters(
        std::make_unique<AverageFilter<3>>()
    )
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .withOdometry() // use the same scales as the chassis (above)
  .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)

  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController2 =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })

    .withOutput(driveauton)
    .buildMotionProfileController();

    profileController2->generatePath({
        {0_in, 0_in, 0_deg},
        {28_in, 0_ft, 0_deg}},
        "scuffed_V12"
    );
    profileController2->generatePath({
        {0_in, 0_in, 0_deg},
        {5_in, 0_ft, 0_deg}},
        "go_back"
    );
    profileController2->generatePath({
        {0_in, 0_in, 0_deg},
        {8_in, 0_ft, 0_deg}},
        "yes"
    );
    piston.set_value(true);
    profileController2->setTarget("scuffed_V12", true);
    delay(1200);
    profileController2->setTarget("go_back");
    delay(500);
    profileController2->setTarget("yes", true);
    piston.set_value(false);
    delay(2000);
    profileController2->setTarget("scuffed_V12");
    delay(15000);

}

void TEST_GO_1() {
  std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();

  // Target location path
  profileController->generatePath({
      {0_in, 0_in, 0_deg},
      {64_ft, 0_ft, 0_deg}},
      "first_move"
  );
  profileController->generatePath({
      {0_in, 0_in, 0_deg},
      {60_in, 0_in, 0_deg}},
      "retreat"
  );
piston.set_value(true);
profileController->setTarget("first_move");
//driveauton->moveDistance(-64_in);
delay(100);
piston.set_value(false);
delay(500);
profileController->setTarget("retreat",true);
//bliftmove(100);
delay(500);
}

void TEST_GO_2() {
  std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();

  // Target location path
  profileController->generatePath({
      {0_in, 0_in, 0_deg},
      {52_ft, 0_ft, 0_deg}},
    "first_move"
  );
  profileController->generatePath({
    {0_ft, 0_ft, 0_deg},
    {40_in, 0_in, 0_deg}},
    "2ndnumogo"
  );
  profileController->generatePath({
    {0_ft, 0_ft, 0_deg},
    {50_in, 0_in, 0_deg}},
    "engage"
  );
  profileController->generatePath({
    {0_ft, 0_ft, 0_deg},
    {45_in, 0_in, 0_deg}},
    "eeeeee"
  );

//profileController->setTarget("first_move",true);
//bliftmove.move_relative(7500, 10);
  piston.set_value(true);
  profileController->setTarget("first_move", true);
  //driveauton->moveDistance(-52_in);
  piston.set_value(false);
  delay(300);
  //driveauton->moveDistance(40_in);
  profileController->setTarget("2ndnumogo");
  driveauton->turnAngle(-35_deg);
  piston.set_value(true);
  driveauton->turnAngle(72_deg);
  //could possibly also be 75 degrees
  profileController->setTarget("engage", true);
  piston.set_value(false);
  delay(300);
  profileController->setTarget("eeeeee");
  //driveauton->moveDistance(45_in);
  piston.set_value(true);
  delay(1000);
  driveauton->turnAngle(180_deg);
  delay(15000);
}

void TEST_GO_3() {
  std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(100)
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();

  // Target location path
  profileController->generatePath({
      {38_in, 0_in, 0_deg},
      {0_ft, 0_ft, 0_deg}},
      "first_mogo"
  );
  profileController->generatePath({
      {0_ft, 0_ft, 0_deg},
      {10_in, 0_in, 0_deg}},
      "mogonom2"
  );
  profileController->generatePath({
      {0_ft, 0_ft, 0_deg},
      {10_in, 0_in, 0_deg}},
      "eee"
  );
  piston.set_value(true);
  profileController->setTarget("first_mogo", true);
  piston.set_value(false);
  delay(300);
  driveauton->turnAngle(225_deg);
  delay(300);
  profileController->setTarget("mogonom2");
  delay(1000);
  bliftmove(500);
  profileController->setTarget("eee", true);
  delay(15000);
}

void AUTO_BACK() {
  const double MOVE_INCH = -109; // move forward
  auto drive = [MOVE_INCH](double dist, double vel){BotMoveRelative(MOVE_INCH*dist, vel);};

  bliftmove_relative(3520, 120); // move down
  delay(1000);
  drive(-42, 180); // move X inch
  delay(3000);
  bliftmove_relative(-2000, 120); // move up
  delay(2000);
  drive(35, 180); // move X inch

}

void AutoGoMid() {
  const double MOVE_INCH = -109; // move forward
  auto drive = [MOVE_INCH](double dist, double vel){BotMoveRelative(MOVE_INCH*dist, vel);};
  piston.set_value(false);
  delay(100);
  drive(64, 200); // move X inch
  delay(4000);
  drive(1, 100); // move X inch
  delay(500);
  piston.set_value(true);
  delay(100);
  fourbarmoverelative(150,200);
  delay(500);
  drive(-50, 200); // move X inch
  delay(3000);
  fourbarmoverelative(-150,200);
  piston.set_value(false);
}

void AutoTurnLeft() {
  const double MOVE_INCH = -109; // move forward
  auto drive = [MOVE_INCH](double dist, double vel){BotMoveRelative(MOVE_INCH*dist, vel);};
  BotTurnRelative(450, 200);
  piston.set_value(false);
  delay(500);
  drive(58, 200); // move X inch
  delay(3000);
  drive(1, 100); // move X inch
  delay(500);
  piston.set_value(true);
  delay(100);
  fourbarmoverelative(150,200);
  delay(500);
  drive(-50, 200); // move X inch
  delay(3000);
  fourbarmoverelative(-150,200);
  piston.set_value(false);
}

void AutoGoFront() {
  const double MOVE_INCH = -109; // move forward
  auto drive = [MOVE_INCH](double dist, double vel){BotMoveRelative(MOVE_INCH*dist, vel);};
  piston.set_value(false);
  drive(40, 200); // move X inch
  delay(2000);
  drive(1, 100); // move X inch
  delay(500);
  piston.set_value(true);
  delay(100);
  fourbarmoverelative(150,200);
  delay(500);
  drive(-35, 200); // move X inch
  delay(3000);
  fourbarmoverelative(-150,200);
  piston.set_value(false);
}

void AutoGoFront1In() {
  const double MOVE_INCH = -109; // move forward
  auto drive = [MOVE_INCH](double dist, double vel){BotMoveRelative(MOVE_INCH*dist, vel);};
  piston.set_value(false);
  drive(41, 200); // move X inch
  delay(2000);
  drive(1, 100); // move X inch
  delay(500);
  piston.set_value(true);
  delay(100);
  fourbarmoverelative(150,200);
  delay(500);
  drive(-35, 200); // move X inch
  delay(3000);
  fourbarmoverelative(-150,200);
  piston.set_value(false);
}