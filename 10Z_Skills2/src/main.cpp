// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// frontRight           motor         16              
// frontLeft            motor         13              
// backRight            motor         19              
// backLeft             motor         12              
// Gyro                 inertial      11              
// dr4b                 motor         3               
// Controller1          controller                    
// LE                   encoder       C, D            
// convey               motor         7               
// fourBar2             motor         1               
// twoBar               motor         10              
// tilter               motor         4               
// DistanceBack         distance      2               
// RotationTilter       rotation      20              
// DistanceFront        distance      21              
// vision_sensor        vision        17              
// middleLeft           motor         5               
// middleRight          motor         6               
// flywheel             motor         18              
// DigitalOutH          digital_out   H               
// Optical              optical       9               
// Distance             distance      8               
// LineTrackerG         line          G               
// indexer              digital_out   B               
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
  /*
  Optical.brightness(75);
  Optical.setLight(ledState::on);
  wait(2, sec);*/
  //wait(2, sec);
  Brain.Screen.print("Gyro calibrated");
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

void pickUpMogoD(bool tBar, bool fBar,  double d, double pos, double cap, int t1, int t2) {
  /*
  KEY: 
  ======> sys = true ---> fourBar
  ======> sys = false ---> twoBar
  */
  double dist = 0;
  if(DistanceFront.isObjectDetected()) {
    dist = DistanceFront.objectDistance(inches) + 3;
  } else {
    dist = d;
  }
    frontLeft.setTimeout(t1, sec);
    frontRight.setTimeout(t1, sec);
    backLeft.setTimeout(t1, sec);
    backRight.setTimeout(t1, sec);
    if (tBar) {
      twoBar.setTimeout(t2, sec);
      twoBar.startRotateFor(fwd, pos, degrees);
    } else if (fBar) {
      dr4b.setTimeout(t2, sec);
      twoBar.startRotateFor(fwd, pos, degrees);
    }
    odom::moveForwardPID(dist, cap);
    if (tBar) {
      twoBar.stop();
    } else if (fBar) {
      dr4b.stop();
    }
  
  
}

void pickUpMogoT(bool tBar, bool fBar, double d, double pos, double cap, int t1, int t2) {
  /*
  KEY: 
  ======> sys = true ---> fourBar
  ======> sys = false ---> twoBar
  */
  double dist = 0;
  if(DistanceBack.isObjectDetected()) {
    dist = -DistanceBack.objectDistance(inches);
    Brain.Screen.print("**********************TRUE");
    Brain.Screen.newLine();
  } else {
    dist = -d;
    Brain.Screen.print("*********************FALSE");
    Brain.Screen.newLine();
  }
    frontLeft.setTimeout(t1, sec);
    frontRight.setTimeout(t1, sec);
    backLeft.setTimeout(t1, sec);
    backRight.setTimeout(t1, sec);
    if (tBar) {
      twoBar.setTimeout(t2, sec);
      twoBar.startRotateFor(fwd, d, degrees);
    } else if (fBar) {
      dr4b.setTimeout(t2, sec);
      twoBar.startRotateFor(fwd, d, degrees);
    }
    odom::moveForwardPID(dist, cap);
    if (tBar) {
      twoBar.stop();
    } else if (fBar) {
      dr4b.stop();
    }
  
}

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
  //convey.setVelocity(100, percent);
  
  convey.setPosition(0, degrees);
  double error = deg - convey.position(degrees); 
  while (error > 2) {
    error = deg - convey.position(degrees);
    convey.spin(vex::reverse, error, rpm);
  }
  //convey.rotateFor(vex::reverse, deg, degrees);

}

void startShooter(double deg) {
  flywheel.spin(vex::reverse, deg, vex::percent);
}

void rotateSpinnerOptical(double col, double t) {
  // if col = 1 --> color is blue
  // if col = 0 --> color is red
  // t sets the timeout
  convey.setPosition(0, degrees);
  frontLeft.setStopping(hold);
  frontRight.setStopping(hold);
  middleLeft.setStopping(hold);
  middleRight.setStopping(hold);
  backLeft.setStopping(hold);
  backRight.setStopping(hold);
  convey.setTimeout(t, seconds);
  //Optical.setLightPower(75, percent);
  bool spinning = true; 
  if (col == 0) {
    while (Optical.color() != red && spinning) {
      convey.spin(vex::forward, 20, percent);
      spinning = convey.isSpinning();
      wait(1, msec);
      if (convey.position(degrees) > 1000) {
        break;
      }
    }
    convey.stop(); 
  } else if (col == 1) {
    while (Optical.color() != blue && spinning) {
      convey.spin(vex::forward, 20, percent);
      spinning = convey.isSpinning();
      wait(1, msec);
      if (convey.position(degrees) > 1000) {
        break;
      }
    }
    
    convey.stop();
  }

  frontLeft.setStopping(coast);
  frontRight.setStopping(coast);
  middleLeft.setStopping(coast);
  middleRight.setStopping(coast);
  backLeft.setStopping(coast);
  backRight.setStopping(coast);
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
  

  //odom::moveTo(double target_x, double target_y, double target_a, double agg, double tolD, double tolA, double capD, double capA, double settleTime);
  //odom::turnToPoint(double target_x, double target_y, double cap, double settleTime);
  //cout << atan2(45, -45) * 180/3.14159265; 
  //odom::turnTo(45, 100000000);
  odom::turnToPoint(0, 24, 400, 10);
  //odom::moveTo(0, 24, 1, 2, 2, 400, 400, 10);
  /*
  // .............................................
  // FIRST HALF
  // .............................................
  
  // SPINNER
  startShooter(54);
  odom::moveForwardPIDDistance(550);
  //wait(5, msec);
  moveSpinner(-750);
  //wait(5, msec);
  //odom::moveForwardPID(10, 400);
  wait(5, msec);

  // SPINNER
  odom::turnTo(45, 450);
  wait(5, msec);
  moveForwardConveyer(28, 57000, 5, 450); //21
  wait(5, msec);
  odom::turnTo(180, 300);
  wait(5, msec);
  //odom::moveForwardPID(-11.45, 600); // -12
  odom::moveForwardPIDDistance(450);
  //wait(5, msec);
  //rotateSpinnerOptical(0, 3);
  moveSpinner(-450);
  wait(5, msec);

  // SHOOT 3 DISCS
  odom::moveForwardPID(12, 500);
  wait(5, msec);
  odom::turnTo(80, 300);
  wait(5, msec);
  moveForwardConveyer(59, 57000, 5, 450); //58
  wait(5, msec);
  odom::turnTo(102, 450);
  //wait(5, msec);
  moveShooter(3500); //4000
  //wait(5, msec);

  // COLLECT 3 DISCS
  odom::turnTo(178, 500);
  wait(5, msec);
  moveForwardConveyer(40.25, 9000000, 5, 200);
  wait(5, msec);
  moveForwardConveyer(-40.25, 9000000, 5, 200);
  wait(5, msec);


  //SHOOT 3 DISCS
  odom::turnTo(100, 500);
  //wait(5,msec);
  moveShooter(3500);
  //wait(5,msec);

  // COLLECT 3 DISCS
  startShooter(53);
  odom::turnTo(90, 600);
  wait(5, msec);
  odom::moveForwardPID(-52, 450);
  wait(5, msec);
  odom::turnTo(135, 400);
  wait(5, msec);
  moveForwardConveyer(72, 50000, 10, 400);
  convey.setVelocity(100, percent);
  convey.rotateFor(fwd, 3000, deg);
  wait(5, msec);
  
  // SHOOT 3 DISCS
  odom::turnTo(45, 500);
  wait(5, msec);
  moveShooter(500);
  wait(50, msec);
  moveShooter(500);
  wait(50, msec);
  moveShooter(500);
  //wait(5, msec);
  
  // COLLECT 3 DISCS
  startShooter(54);
  odom::moveForwardPID(9, 300);
  wait(5, msec);  
  odom::turnTo(90, 450);
  wait(5, msec);
  moveForwardConveyer(38, 50000, 10, 200);
  convey.setVelocity(100, percent);
  convey.rotateFor(fwd, 2500, deg);
  wait(5, msec);

  // SHOOT 3 DISCS
  odom::reverseTurn(358, 400);
  //wait(5, msec);
  moveShooter(3500);
  //wait(5, msec);

  // SPINNER 
  odom::reverseTurn(359, 450); 
  wait(5, msec);
  odom::moveForwardPID(-55, 450);
  wait(5, msec);
  odom::reverseTurn(270, 400); //300 
  wait(5, msec);
  //odom::moveForwardPID(-10, 550);
  odom::moveForwardPIDDistance(450);
  //wait(5, msec);
  //rotateSpinnerOptical(0, 3);
  moveSpinner(-450);
  wait(5, msec);
  odom::moveForwardPID(4, 550);
  //odom::moveForwardPID(10, 450);
  //wait(5, msec);

  // SPINNER
  odom::turnTo(245, 550); //500
  wait(5, msec);
  moveForwardConveyer(23, 57000, 5, 450);//21
  wait(5, msec);
  odom::turnTo(357, 400);
  wait(5, msec);
  odom::moveForwardPIDDistance(450);
  //wait(5, msec);
  moveSpinner(-500);
  wait(5, msec);

  // SHOOT 3 DISCS
  //startShooter(58);
  odom::moveForwardPID(10, 500);
  wait(5, msec);
  odom::turnTo(265, 450);
  wait(5, msec);
  moveForwardConveyer(58, 57000, 5, 450);//58
  wait(5, msec);
  odom::turnTo(282, 450);
  //wait(5, msec);
  moveShooter(3500);
  //wait(5, msec);
  //odom::turnTo(268, 600);
  //wait(5, msec);



  // COLLECT 3 DISCS
  odom::turnTo(354, 500); //357
  wait(5, msec);
  moveForwardConveyer(39, 9000000, 5, 200);
  wait(5, msec);
  moveForwardConveyer(-39, 9000000, 5, 300);
  wait(5, msec);

  //SHOOT 3 DISCS
  odom::turnTo(280, 500);
  //wait(5,msec);
  moveShooter(3500);
  //wait(5,msec); // --> comment out

  // COLLECT 3 DISCS
  odom::turnTo(270, 500);
  wait(5, msec);
  odom::moveForwardPID(-50, 500); //450
  wait(5, msec);
  odom::turnTo(315, 500);
  wait(5, msec);
  moveForwardConveyer(72, 50000, 10, 400);
  convey.setVelocity(100, percent);
  convey.rotateFor(fwd, 2500, deg);
  wait(5, msec);
  
  // SHOOT 3 DISCS
  startShooter(53);
  odom::turnTo(225, 400);
  wait(5, msec);
  moveShooter(500);
  wait(50, msec);
  moveShooter(500);
  wait(50, msec);
  moveShooter(500);
  
  // EARLY EXPANSION
  odom::turnTo(145, 600); //145
  odom::moveForwardPID(-30, 600); //40
  odom::turnTo(132, 600);
  expand();*/

  // odom::turnTo(135, 600);
  /*
  // COLLECT 3 DISCS
  odom::moveForwardPID(9, 400);
  wait(5, msec);
  odom::turnTo(270, 500);
  wait(5, msec);
  moveForwardConveyer(36, 50000, 10, 200);
  convey.setVelocity(100, percent);
  convey.rotateFor(fwd, 2500, deg);
  wait(5, msec);

  // SHOOT 3 DISCS
  odom::turnTo(172, 450);
  //wait(5, msec);
  moveShooter(4000);
  wait(5, msec);

  // SPINNER 
  odom::turnTo(179, 600);
  //wait(5, msec);
  odom::moveForwardPID(-60, 600);
  odom::turnTo(135, 600);
  //expand();*/
  

  while (1){
    //Insert move to functions and other functions over here
    
    //odom::moveForwardPID(24, 80);
    this_thread::sleep_for(10); //Wait for 10 Seconds, Updates x,y Every 10 Seconds
    //odom::turnTo(90);
  
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


  

/*
void fourBarFunction(){
  if (Controller1.ButtonR1.pressing()) {
    dr4b.spin(vex::forward, 100, vex::percent);
  } else if (Controller1.ButtonR2.pressing()){
    dr4b.spin(vex::reverse, 100, vex::percent);
  } else {
    dr4b.setStopping(brakeType::hold);
    dr4b.stop();
  }
}

void conveyorFunction() {
  if (Controller1.ButtonRight.pressing()) {
    conveyor.spin(vex::forward, 50, vex::percent); //75
  } else if (Controller1.ButtonLeft.pressing()) {
    conveyor.spin(vex::reverse, 50, vex::percent);
  } else {
    conveyor.setStopping(coast);
    conveyor.stop();
  }
}

void latchFunction() {
  if (Controller1.ButtonL1.pressing()) twoBar.spin(vex::forward, 100, vex::percent);
  else if (Controller1.ButtonL2.pressing()) twoBar.spin(vex::reverse, 100, vex::percent);
  else {
   twoBar.setStopping(hold);
   twoBar.stop(); 
  }


}

void latchActivation() {
  latch.spinFor(fwd, 100, degrees);
}

void latchRelease() {
  latch.spinFor(fwd, -100, degrees);
}


void clamp(){
  if (Controller1.ButtonX.pressing()) latch.spin(vex::reverse, 100, vex::percent);
  else if (Controller1.ButtonB.pressing()) latch.spin(vex::forward, 100, vex::percent);
  else {
    latch.setStopping(hold);
    latch.stop();
  }
}

////ONE STICK CONTROL
void oneStick()
{
  // Assigns  joystick axis values to variables 
  int axis3val = Controller1.Axis3.value();
  int axis4val = Controller1.Axis4.value();
  int  LV = 0;
  int  RV = 0;
  // Turning movement velocity declaration 
  int turnvel = 0;
  // Forward/backwards movement velocity declaration 
  int latvel = 0;
  // Threshold to provide for driver joystick error (sensitivity)
  int threshold = 10;

  //States that if the value of the joystick is greater than the threshold, 
  //set the velocity equal to the latvel variable
  if(abs(axis4val) > threshold)

    latvel = axis4val;
  // If the joystick val is less than 10, then don't take the value 
  else

    latvel = 0;
  // Same logic as above 
  if(abs(axis3val) > threshold)

    turnvel = axis3val;

  else

    turnvel = 0;
  // Allows for movement using axis vals 

  // If right side movement is needed, LV is greater and vice versa

  // If straight movement is needed the turnvel will be 0 so both velocities are the 
  // same 
  LV = turnvel + latvel;
  RV = turnvel - latvel;
  // Ignore the next 2 lines. Used for testing of Odometry 
  //O.update();

  //printf("%f,%f\n",O.posX,O.posY);
  //printf("%f,%f\n",LE.position(turns),RE.position(turns));

  // Gives move command to motors based on velocities from the axis vals on joystick

  backLeft.spin(vex::forward,LV,percent);
  backRight.spin(vex::forward,RV,percent);
  frontLeft.spin(vex::forward,LV,percent);
  frontRight.spin(vex::forward,RV,percent);

  
}

void holdRobot() {
  double l = Controller1.Axis3.position(vex::percent);  //Axis 3 controls forward + backward
    //int lateral = Controller1.Axis4.position(vex::percent);       //Axis 4 controls strafe left + strafe right
  double r = Controller1.Axis1.position(vex::percent);    //Axis 1 controls turning

  frontRight.setStopping(hold);
  frontLeft.setStopping(hold);
  backRight.setStopping(hold);
  backLeft.setStopping(hold);
  frontRight.spin(vex::forward, l - r, vex::percent); 
  frontLeft.spin(vex::forward, l + r, vex::percent);
  backRight.spin(vex::forward, l - r, vex::percent);
  backLeft.spin(vex::forward, l + r, vex::percent);
  
}
*/

/*
int Xpressed = 0;

bool alatch = false;
void flagA() {
  alatch = true;
}

int Upressed = 0;
bool ahold = false;

void flagB() {
  ahold = true;
}*/


void usercontrol(void) {
//Replace Cap and Floor Values with Actual Measured Values {Placer Holders Kept for Now}
  //pOdom = new odom();
  //vex::thread positionTrackingThread(odom::positionTracking); // Define thread for Position Tracking Method
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