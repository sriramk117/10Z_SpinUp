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
// RotationTilter       rotation      20              
// DistanceFront        distance      21              
// vision_sensor        vision        13              
// middleLeft           motor         5               
// middleRight          motor         6               
// flywheel             motor         17              
// DigitalOutH          digital_out   C               
// Optical              optical       9               
// Distance             distance      8               
// LineTrackerG         line          G               
// indexer              digital_out   B               
// compression          digital_out   A               
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
      frontRight.spin(vex::forward, -20, vex::rpm);
      frontLeft.spin(vex::forward, 20, vex::rpm);
      middleRight.spin(vex::forward, -20, vex::rpm);
      middleLeft.spin(vex::forward, 20, vex::rpm);
      backRight.spin(vex::forward, -20, vex::rpm);
      backLeft.spin(vex::forward, 20, vex::rpm);
    } else if(vision_sensor.largestObject.centerX < centerFOV - offsetX){
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
  convey.setVelocity(95, percent);
  convey.rotateFor(vex::reverse, deg, degrees);
void startShooter(double vel) {
  flywheel.spin(vex::fwd, vel, percent);
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

  //odom::moveTo(double target_x, double target_y, double dir, double turnScale, double tolD, double tolA, double capD, double capA, doluble settleTime);
  //odom::setPoint(double target_x, double target_y, double dir, double turnScale, double tolD, double tolA, double capD, double capA, double settleTime);
  //odom::turnToPoint(double target_x, double target_y, double cap, double settleTime);
  //rollerSpin(bool color, double speed, double timeout, double extraspin)
  
  // ROLLER 1
  startShooter(70);
  odom::moveTo(36, 8.5, -1, 0.1, 2.52, 600, 600, 5);
  wait(5, msec);
  // roller sub (REMOVE)
  wait(500, msec);
  // roller sub (REMOVE)

  // COLLECT DISCS
  odom::moveTo(36, 14, 1, 0.1, 1, 600, 600, 5);
  wait(5, msec);
  odom::turnToPoint(21, 27, 550, 2);
  wait(5, msec);
  moveToConveyer(10000, 0, 21,27, 1, 1, 2, 550, 600, 2);
  wait(5, msec);

  // ROLLER 2
  odom::turnTo(180, 550);
  wait(5, msec);
  odom::moveTo(11, 27, -1, 0.1, 1, 550, 600, 2);
  // roller sub (REMOVE)
  wait(500, msec);
  // roller sub (REMOVE)
 
  // SHOOT DISCS
  moveToConveyer(10000, 0, 17.5, 87.5, 1, 1.8, 2, 550, 600, 5); //15.5, 76.5
  wait(5, msec);
  odom::turnToPoint(29, 128, 550, 2);
  wait(5, msec);
  moveShooter(1000);
  
  // COLLECT DISCS
  odom::turnToPoint(25, 87.5, 550, 2);
  wait(5, msec);
  moveToConveyer(10000, 0, 25, 88, 1, 2, 1, 200, 600, 5);
  moveToConveyer(10000, 300, 53, 93, 1, 2.2, 2, 150, 600, 5);
  wait(5, msec);

  // SHOOT DISCS
  moveToConveyer(10000, 50, 19, 88, -1, 1, 1, 350, 600, 5);
  wait(5, msec);
  odom::turnToPoint(30, 128, 550, 2);
  wait(5, msec);
  moveShooter(1000);

  // COLLECT DISCS
  startShooter(65);
  odom::turnTo(90, 600);
  odom::moveTo(26, 48, -1, 2.9, 1.5, 550, 600, 5);
  wait(5, msec);
  odom::turnToPoint(70, 90, 550, 2);
  wait(5, msec);
  moveToConveyer(10000, 500, 70, 90, 1, 0.2, 2, 250, 600, 5);
  wait(5, msec);
  
  // SHOOT DISCS
  odom::turnToPoint(32, 132, 550, 2);
  indexer.set(true);
  moveShooter(250);
  wait(75, msec);
  moveShooter(250);
  wait(75, msec);
  moveShooter(250);
  wait(75, msec);
  indexer.set(false);

  // COLLECT 3 DISCS
  startShooter(60);
  odom::turnToPoint(115, 136, 550, 2);
  wait(5, msec);
  moveToConveyer(10000, 250, 115, 136, 1, 1, 2, 300, 600, 5);
  wait(5, msec);

  // ROLLER 3 
  odom::turnTo(270, 600);
  wait(5, msec);
  odom::moveTo(115, 140, -1, 0.1, 1, 600, 600, 5); //112, 141.5
  wait(5, msec);
  // roller sub (REMOVE)
  wait(500, msec);
  // roller sub (REMOVE)

  // SHOOT 3 DISCS
  moveToConveyer(10000, 0, 64, 136, 1, 2.2, 2, 500, 600, 5);
  wait(5, msec);
  odom::turnToPoint(29, 130, 550, 2);
  moveShooter(1000);

  // COLLECT 3 DISCS
  odom::turnToPoint(64, 118, 500, 2);
  wait(5, msec);
  moveToConveyer(10000, 0, 62.5, 118, 1, 1, 2, 200, 600, 5);
  moveToConveyer(10000, 250, 62.5, 83, 1, 1, 2, 150, 600, 5);
  wait(5, msec);

  // SHOOT DISCS
  odom::turnToPoint(29, 128, 550, 2);
  indexer.set(true);
  moveShooter(250);
  wait(50, msec);
  moveShooter(250);
  wait(50, msec);
  moveShooter(250);
  wait(50, msec);
  indexer.set(false);
  
  // COLLECT 3 DISCS
  odom::turnToPoint(99, 117, 550, 2);
  wait(5, msec);
  moveToConveyer(100000, 0, 99, 117, 1, 1.5, 2, 550, 600, 5);
  moveToConveyer(100000, 250, 125, 117, 1, 1.5, 2, 200, 600, 5);
  

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
  frontRight.setPosition(0, degrees);
  frontLeft.setPosition(0, degrees);
  middleRight.setPosition(0, degrees);
  middleLeft.setPosition(0, degrees);
  backLeft.setPosition(0, degrees);
  backRight.setPosition(0, degrees);
  
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
