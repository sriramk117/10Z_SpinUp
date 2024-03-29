#include "vex.h"
#include "driver.h"
#include "odom.h"
#include <iostream>

using namespace std;

extern odom* pOdom;
double driver::fourBarPos = 0;
double driver::twoBarPos = 0;

bool shoot = false;
bool droll = false;
bool active = false;

int Xpressed = 0;

bool alatch = false;
void flagA() {
  alatch = true;
}

int Upressed = 0;
bool ahold = false;

void flagB() {
  ahold = true;
}

void driver::robotDrive(){

  pOdom = new odom();

  vex::thread positionTrackingThread(odom::positionTracking); // Define thread for Position Tracking Method
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    frontRight.setStopping(brake);
    frontLeft.setStopping(brake);
    middleRight.setStopping(brake);
    middleLeft.setStopping(brake);
    backRight.setStopping(brake);
    backLeft.setStopping(brake);

    //Controller Position Values
    double longitudinal = Controller1.Axis3.position(vex::percent); //Axis 3 controls forward + backward
    double rotational = Controller1.Axis1.position(vex::percent);    //Axis 1 controls turning

    //Acceleration Curve
    double k1 = 10;
    double k2 = 10;

    if (longitudinal < 0) {
      k1 = -10;
    } else if (longitudinal >= 0) {
      k1 = 10;
    } 

    if (rotational < 0) {
      k2 = -9;
    } else if (rotational >= 0) {
      k2 = 9;
    } 

    double e = 2.718281828459;
    double t = 13;

    double rotCurve =  0.75 * (pow(e, -t/10.0) + pow(e, (abs(rotational) - 100.0)/10.0) * (1 - pow(e, -t/10.0))) * rotational;//pow(rotational,2) * 0.01;//(pow(e, -t/10.0) + pow(e, (abs(rotational) - 127.0)/10.0) * (1 - pow(e, -t/10.0))) * rotational;
    if (rotational > 5.0 && rotational < 30.0) {
      rotCurve = 10.0;
    }
    if (rotational < -5.0 && rotational > -30.0) {
      rotCurve = -10.0;
    }
   
    double lonCurve = longitudinal;
    
    // Drive Code
    frontRight.spin(vex::forward, lonCurve - rotCurve, vex::percent);
    frontLeft.spin(vex::forward, lonCurve + rotCurve, vex::percent);
    middleRight.spin(vex::forward, lonCurve - rotCurve, vex::percent);
    middleLeft.spin(vex::forward, lonCurve + rotCurve, vex::percent);
    backRight.spin(vex::forward, lonCurve - rotCurve, vex::percent);
    backLeft.spin(vex::forward, lonCurve + rotCurve, vex::percent);

    // Driver Functions
    shooter();
    intake();
    expand();
    angleChanger();         
  } 

  
    this_thread::sleep_for(10); //Wait for 10 Seconds, Updates x,y Every 10 Seconds
    Brain.Screen.print("/n Coordinates: (%s, %s)", odom::x, odom::y);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  
}

void driver::angleChanger() {
  if (active) {
    if (Controller1.ButtonY.pressing() || Controller1.ButtonRight.pressing()){
      indexer.set(false);
      waitUntil(!(Controller1.ButtonY.pressing() || Controller1.ButtonRight.pressing()));
      active = false;
   }
  } else {
    if (Controller1.ButtonY.pressing()){
      indexer.set(true);
      waitUntil(!Controller1.ButtonY.pressing());
      active = true;
    } else if (Controller1.ButtonRight.pressing()) {
      indexer.set(false);
      waitUntil(!Controller1.ButtonRight.pressing());
      active = true;
    } 
  }
}

void driver::shooter() {
    flywheel.spin(vex::fwd, 60, percent);

    if (Controller1.ButtonX.pressing()) {
      flywheel.spin(vex::fwd, 60, percent); 
    }
}

void driver::intake() {
  if (Controller1.ButtonL2.pressing()) {
    droll = true;
    convey.spin(vex::forward, 100, vex::percent);
  } else if (Controller1.ButtonR2.pressing()){
    droll = true;
    convey.spin(vex::reverse, 100, percent);
  } else {
    compression.set(false);
    droll = false;
    convey.stop();
  }
}

void driver::expand() {
  if (Controller1.ButtonDown.pressing()) {
    DigitalOutH.set(true);
  }
}



void driver::conveyorSpin(int conveyorSwitch){
  if (conveyorSwitch==1) {
    if (Controller1.ButtonUp.pressing()) convey.spin(vex::forward, 85, vex::percent);
  } else {
    if (Controller1.ButtonUp.pressing())  convey.stop();
  }
  if (Controller1.ButtonUp.pressing()) conveyorSwitch = 3-conveyorSwitch;
}

void driver::fourBar(){
  if (Controller1.ButtonR1.pressing()) {
    //dr4b.setBrake(brakeType::undefined);
    dr4b.spin(vex::forward, 100, vex::percent);
    fourBar2.spin(vex::forward, 100, vex::percent);
  } else if (Controller1.ButtonR2.pressing()){
    //dr4b.setBrake(brakeType::undefined);
    dr4b.spin(vex::reverse, 100, vex::percent);
    fourBar2.spin(vex::reverse, 100, vex::percent);
  } else {
    dr4b.setStopping(brakeType::hold);
    dr4b.stop();
    fourBar2.stop();
  }
}

void driver::twoBarMove(){
  if (Controller1.ButtonL1.pressing()) tilter.spin(vex::forward, 100, vex::percent);
  else if (Controller1.ButtonL2.pressing()) tilter.spin(vex::reverse, 100, vex::percent);
  else {
    tilter.setStopping(hold);
    tilter.stop();
  }
}


void driver::motorTemp() {
  Brain.Screen.print("Temps: %s, %s, %s, %s", frontRight.temperature(percent),frontLeft.temperature(percent), backRight.temperature(percent), backLeft.temperature(percent) );
  Brain.Screen.newLine();
}
