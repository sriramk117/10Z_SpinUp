// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// frontRight           motor         16              
// frontLeft            motor         15              
// backRight            motor         19              
// backLeft             motor         12              
// Gyro                 inertial      11              
// dr4b                 motor         3               
// Controller1          controller                    
// LE                   encoder       E, F            
// convey               motor         7               
// fourBar2             motor         1               
// twoBar               motor         10              
// tilter               motor         4               
// DistanceBack         distance      2               
// RotationTilter       rotation      13              
// DistanceFront        distance      21              
// vision_sensor        vision        20              
// middleLeft           motor         5               
// middleRight          motor         6               
// flywheel             motor         17              
// indexer              digital_out   B               
// DigitalOutH          digital_out   C               
// compression          digital_out   A               
// Optical              optical       9               
// Distance             distance      8               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include "odom.h"
#include "driver.h"
#include "iostream"


using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

extern odom* pOdom;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


//variables
double fourBarPos = 0;
double twoBarPos = 0;


void pre_auton(void) {    
  // Initializing Robot Configuration. DO NOT REMOVE!
  
  vexcodeInit();
  Gyro.calibrate();
  while(Gyro.isCalibrating()) {
    wait(1, msec);
  }
  
  Optical.brightness(75);
  Optical.setLight(ledState::on);
  wait(2, sec);


  Brain.Screen.print("Gyro + Optical calibrated");
  Brain.Screen.newLine();
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

///////////////////////////////////////////////////AUTON FUNCTIONS
void moveForwardConveyer(double d, double c, double t, double cap) {
  frontLeft.setTimeout(t, sec);
  frontRight.setTimeout(t, sec);
  backLeft.setTimeout(t, sec);
  backRight.setTimeout(t, sec);
  convey.setVelocity(100, percent);
  convey.startRotateFor(fwd, c, deg);
  odom::moveForwardPID(d, cap);
  convey.stop();
}

void moveToConveyer(double c, double waittime, double target_x, double target_y, double dir, double turnScale, double tolD, double capD, double capA, double settleTime) {
  convey.setStopping(coast);
  convey.setVelocity(200, rpm);
  convey.startRotateFor(fwd, c, deg);
  odom::moveTo(target_x, target_y, dir, turnScale, tolD, capD, capA, settleTime);
  wait(waittime, msec);
  convey.stop();
}

void align(int c) {
  /*KEY:
    RED = 0
    BLUE = 1
  */
  int centerFOV = 195;
  int offsetX = 25;
  if (c == 0) {
    vision_sensor.takeSnapshot(vision_sensor__GOAL_RED);
  } else if (c == 1) {
    vision_sensor.takeSnapshot(vision_sensor__GOAL_BLUE);
  }
  
  while(vision_sensor.largestObject.centerX > centerFOV + offsetX || vision_sensor.largestObject.centerX < centerFOV - offsetX){  
    if (c == 0) {
      vision_sensor.takeSnapshot(vision_sensor__GOAL_RED);
    } else if (c == 1) {
      vision_sensor.takeSnapshot(vision_sensor__GOAL_BLUE);
    }

    if(vision_sensor.largestObject.centerX > centerFOV + offsetX){
      // eventually change to turnTo PID movement
      frontRight.spin(vex::forward, -20, vex::rpm);
      frontLeft.spin(vex::forward, 20, vex::rpm);
      middleRight.spin(vex::forward, -20, vex::rpm);
      middleLeft.spin(vex::forward, 20, vex::rpm);
      backRight.spin(vex::forward, -20, vex::rpm);
      backLeft.spin(vex::forward, 20, vex::rpm);
    } else if(vision_sensor.largestObject.centerX < centerFOV - offsetX){
      // eventually change to turnTo PID movement
      frontRight.spin(vex::forward, 20, vex::rpm);
      frontLeft.spin(vex::forward, -20, vex::rpm);
      middleRight.spin(vex::forward, 20, vex::rpm);
      middleLeft.spin(vex::forward, -20, vex::rpm);
      backRight.spin(vex::forward, 20, vex::rpm);
      backLeft.spin(vex::forward, -20, vex::rpm);
    } else {
      break;
    }
  }
  frontRight.stop();
  frontLeft.stop(); 
  backRight.stop(); 
  backLeft.stop();
}

void moveSpinner(double deg) {
  convey.setVelocity(50, percent);
  convey.rotateFor(vex::reverse, deg, degrees);
}

// CHANGED SHOOTER CODE
void moveShooter(double deg) {
  convey.setVelocity(100, percent);
  convey.rotateFor(vex::reverse, deg, degrees);
}

void startShooter(double vel) {
  flywheel.spin(vex::fwd, vel, rpm);
}

int getColor(){
  Optical.setLightPower(100,pct);
  if (Optical.isNearObject()) {
    int objHue = Optical.hue();
    if (objHue < 300 && objHue > 180) {return 1;}
    else if (objHue < 20) {return 0;} 
    else {return 2;}
  }  
  else {return 2;}
}

void rollerSpin(bool color, double speed, double timeout, double extraspin){
  indexer.set(true);
  if (color == 0) {
    double starttime = Brain.timer(sec);
    while(!getColor() == color && Brain.timer(sec)-starttime < timeout){
      convey.spin(fwd, speed, pct);
    }
    while(getColor() == color && Brain.timer(sec)-starttime < timeout){
      convey.spin(fwd, speed/2, pct);
    }
    convey.stop(hold);
    convey.rotateFor(fwd, extraspin, deg, speed, velocityUnits::pct, false);
  }
  if (color == 1) {
    double starttime = Brain.timer(sec);
    while(getColor() == 0 && Brain.timer(sec)-starttime < timeout){
      convey.spin(fwd, speed, pct);
    }
    while( ((!getColor()) == 0) && Brain.timer(sec)-starttime < timeout){
      convey.spin(fwd, speed/2, pct);
    }
    convey.stop(hold);
    convey.rotateFor(fwd, extraspin, deg, speed, velocityUnits::pct, false);
  }
  indexer.set(false);
}


void brakeHold() {
  frontLeft.setStopping(hold);
  frontRight.setStopping(hold);
  middleLeft.setStopping(hold);
  middleRight.setStopping(hold);
  backLeft.setStopping(hold);
  backRight.setStopping(hold);
}

void brakeCoast() {
  frontLeft.setStopping(coast);
  frontRight.setStopping(coast);
  middleLeft.setStopping(coast);
  middleRight.setStopping(coast);
  backLeft.setStopping(coast);
  backRight.setStopping(coast);
}

void expand() {
  DigitalOutH.set(true);
}

void autonomous(void) {

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  pOdom = new odom();
  vex::thread positionTrackingThread(odom::positionTracking); // Define thread for Position Tracking Method

  frontRight.setPosition(0, degrees);
  frontLeft.setPosition(0, degrees);
  middleRight.setPosition(0, degrees);
  middleLeft.setPosition(0, degrees);
  backLeft.setPosition(0, degrees);
  backRight.setPosition(0, degrees);
  

  // RED = 0
  // BLUE = 1

  // AUTON TEMPLATE

  //odom::moveTo(double target_x, double target_y, double dir, double turnScale, double tolD, double tolA, double capD, double capA, double settleTime);
  //odom::turnToPoint(double target_x, double target_y, double cap, double settleTime);
  //rollerSpin(bool color, double speed, double timeout, double extraspin)

  startShooter(200);
  indexer.set(true);
  wait(750, msec);
  odom::moveTo(36, 10, -1, 1, 1, 600, 600, 10);
  moveSpinner(-250);
  wait(5, msec);
  odom::moveTo(36, 17, 1, 1, 1, 600, 600, 10);
  wait(5, msec);
  odom::turnTo(82.5, 600);
 
  //SHOOT
  moveShooter(150);
  wait(100, msec); 
  moveShooter(150);
  wait(5, msec);

  // COLLECT
  startShooter(170);
  odom::turnTo(143, 600);
  wait(50, msec);
  moveToConveyer(100000, 0, 56, 31, 1, 1, 2, 600, 600, 5);
  moveToConveyer(100000, 500, 75, 47.5, 1, 1, 2, 200, 600, 5);
  wait(5, msec);
  odom::turnTo(58, 550); 
  
  // SHOOT
  moveShooter(150);
  wait(100, msec); 
  moveShooter(150);
  wait(100, msec); 
  moveShooter(150);
  wait(5, msec);
  
  // COLLECT 
  startShooter(190);
  odom::turnTo(138, 600);
  wait(5, msec);
  moveToConveyer(100000,500, 128.5, 113, 1, 1.2, 1, 450, 1000000, 5);
  wait(5, msec);

  // ROLLER
  odom::turnTo(0, 600);
  wait(5, msec);
  odom::moveTo(137, 113, -1, 1, 2, 600, 600, 5);
  moveSpinner(-250);
  wait(5, msec);

  // SHOOT
  odom::moveTo(128.5, 113, 1, 1, 1, 600, 600, 5); 
  wait(5, msec);
  odom::turnTo(8.5, 550);
  moveShooter(150);
  wait(100, msec); 
  moveShooter(150);
  wait(100, msec);
  moveShooter(150);

  while (1){
    //Insert move to functions and other functions over here
    this_thread::sleep_for(10); //Wait for 10 Seconds, Updates x,y Every 10 Seconds  
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  driver::robotDrive();
  
  //wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
}



//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

   

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
