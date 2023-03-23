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

    frontRight.setPosition(0, degrees);
    frontLeft.setPosition(0, degrees);
    middleRight.setPosition(0, degrees);
    middleLeft.setPosition(0, degrees);
    backLeft.setPosition(0, degrees);
    backRight.setPosition(0, degrees);

    //Controller Position Values
    double longitudinal = Controller1.Axis3.position(vex::percent); //Axis 3 controls forward + backward
    //double lateral = Controller1.Axis4.position(vex::percent);       //Axis 4 controls strafe left + strafe right
    double rotational = Controller1.Axis1.position(vex::percent);    //Axis 1 controls turning

    //Acceleration Curve
    double k1 = 10;//0.85;
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
    double t = 13;//13;//15;//6;//19;//15;//10;//12;//6.7;
    double rotCurve = 0;

    if (rotational >= 0) {
      rotCurve = 1.2 * pow(1.043, rotational) -1.2 + 0.2 * rotational;//0.75 * (pow(e, -t/10.0) + pow(e, (abs(rotational) - 100.0)/10.0) * (1 - pow(e, -t/10.0))) * rotational;
    } else {
      rotational = -rotational;
      rotCurve = 1.2 * pow(1.043, rotational) -1.2 + 0.2 * rotational;//0.75 * (pow(e, -t/10.0) + pow(e, (abs(rotational) - 100.0)/10.0) * (1 - pow(e, -t/10.0))) * rotational;
      rotCurve = -rotCurve;
    }
    rotCurve *= .8;
    
    //if (rotational > 5.0 && rotational < 30.0) {
    //  rotCurve = 10.0;
    //}
    //if (rotational < -5.0 && rotational > -30.0) {
    //  rotCurve = -10.0;
    //}
    //if (rotational < 0) {
    //  rotCurve *= -1;
    //}
    double lonCurve = longitudinal;//(pow(e, ((abs(longitudinal) - 100.0)* t)/1000.0)) * longitudinal;//pow(longitudinal,2) * 0.01;//(pow(e, ((abs(longitudinal) - 127)* t)/1000.0)) * longitudinal;    
    //if (longitudinal < 0) {
    //  lonCurve *= -1;
    //}
    // Drive Code
    //frontRight.spin(vex::forward, k1*sqrt(fabs(longitudinal)) - k2*sqrt(fabs(rotational)), vex::percent); 
    //frontLeft.spin(vex::forward, k1*sqrt(fabs(longitudinal)) + k2*sqrt(fabs(rotational)), vex::percent);
    //middleRight.spin(vex::forward, k1*sqrt(fabs(longitudinal)) - k2*sqrt(fabs(rotational)), vex::percent);
    //middleLeft.spin(vex::forward, k1*sqrt(fabs(longitudinal)) + k2*sqrt(fabs(rotational)), vex::percent);
    //backRight.spin(vex::forward, k1*sqrt(fabs(longitudinal)) - k2*sqrt(fabs(rotational)), vex::percent);
    //backLeft.spin(vex::forward, k1*sqrt(fabs(longitudinal)) + k2*sqrt(fabs(rotational)), vex::percent);
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
    
    

    
  // COAST --> HOLD MACRO
  /*
    double longitudinal = Controller1.Axis3.position(vex::percent);  //Axis 3 controls forward + backward
    double rotational = Controller1.Axis1.position(vex::percent);    //Axis 1 controls turning

    if (ahold && (Upressed % 2)==0) {
      // if button X was pressed even times do this
      frontRight.setStopping(coast);
      frontLeft.setStopping(coast);
      backRight.setStopping(coast);
      backLeft.setStopping(coast);
      Upressed++;
      ahold = false;   
    } else if (ahold && (Upressed % 2)!=0) {
      // if button X was pressed odd times do this
      frontRight.setStopping(hold);
      frontLeft.setStopping(hold);
      backRight.setStopping(hold);
      backLeft.setStopping(hold);
      Upressed++;
      ahold = false;
    }
    frontRight.spin(vex::forward, longitudinal - rotational, vex::percent); 
    frontLeft.spin(vex::forward, longitudinal + rotational, vex::percent);
    backRight.spin(vex::forward, longitudinal - rotational, vex::percent);
    backLeft.spin(vex::forward, longitudinal + rotational, vex::percent);
    
    // OLD FUNCTIONS
    driver::fourBar();
    driver::twoBarMove();
    driver::airClamp();
    driver::cata();
    driver::motorTemp();*/
         
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
  
    flywheel.spin(vex::fwd, 170, rpm); //60
    //flywheel.spin(vex::fwd, 11, voltageUnits::volt);

    if (Controller1.ButtonX.pressing()) {
      flywheel.spin(vex::fwd, 190, rpm); //87 //60
    }
  
  
}

void driver::intake() {
  if (Controller1.ButtonL2.pressing()) {
    droll = true;
    convey.spin(vex::forward, 100, vex::percent);
  } else if (Controller1.ButtonR2.pressing()){
    //convey.setPosition(0, degrees);
    //compression.set(true);
    droll = true;
    //convey.spin(vex::reverse, 100, vex::percent);
    /*
    double error = (100 - convey.position(degrees)) * 0.8; 
    while (error > 2 && Controller1.ButtonR2.pressing()) {
      error = (100 + convey.position(degrees)) * 0.8;
      convey.spin(vex::reverse, error, percent);
    }*/
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