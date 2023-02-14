#include "vex.h"
#include "driver.h"
#include "odom.h"

using namespace std;

extern odom* pOdom;
double driver::fourBarPos = 0;
double driver::twoBarPos = 0;

bool shoot = false;
bool droll = false;

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
    frontRight.setStopping(coast);
    frontLeft.setStopping(coast);
    middleRight.setStopping(coast);
    middleLeft.setStopping(coast);
    backRight.setStopping(coast);
    backLeft.setStopping(coast);

    //Controller Position Values
    double longitudinal = Controller1.Axis3.position(vex::percent);  //Axis 3 controls forward + backward
    //int lateral = Controller1.Axis4.position(vex::percent);       //Axis 4 controls strafe left + strafe right
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
      k2 = -10;
    } else if (rotational >= 0) {
      k2 = 10;
    } 

    

    // Drive Code
    frontRight.spin(vex::forward, k1*fabs(longitudinal)*fabs(longitudinal) - k2*fabs(rotational)*fabs(rotational), vex::percent); 
    frontLeft.spin(vex::forward, k1*fabs(longitudinal)*fabs(longitudinal) + k2* fabs(rotational)*fabs(rotational), vex::percent);
    middleRight.spin(vex::forward, k1*fabs(longitudinal)*fabs(longitudinal) - k2*fabs(rotational)*fabs(rotational), vex::percent);
    middleLeft.spin(vex::forward, k1*fabs(longitudinal)*fabs(longitudinal) + k2*fabs(rotational)*fabs(rotational), vex::percent);
    backRight.spin(vex::forward, k1*fabs(longitudinal)*fabs(longitudinal) - k2*fabs(rotational)*fabs(rotational), vex::percent);
    backLeft.spin(vex::forward, k1*fabs(longitudinal)*fabs(longitudinal) + k2*fabs(rotational)*fabs(rotational), vex::percent);
    
    // Driver Functions
    shooter();
    intake();
    expand();
    
    

    
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


void driver::shooter() {
  
  if (Controller1.ButtonR1.pressing()) {
    //indexer.set(false);
    flywheel.spin(vex::fwd, 12, voltageUnits::volt);
  } else {
    //indexer.set(true);
    flywheel.spin(vex::fwd, 12, voltageUnits::volt);
  }
  
}

void driver::intake() {
  if (Controller1.ButtonL2.pressing()) {
    droll = true;
    convey.spin(vex::forward, 100, vex::percent);
  } else if (Controller1.ButtonR2.pressing()){
    droll = true;
    convey.spin(vex::reverse, 100, vex::percent);
  } else {
    droll = false;
    convey.stop();
  }
}

void driver::expand() {
   if (Controller1.ButtonY.pressing()) {
    //DigitalOutG.set(true);
    DigitalOutH.set(true);
    //DigitalOutA.set(true);
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
 
 
 
 }
else if (Controller1.ButtonR2.pressing()){

//dr4b.setBrake(brakeType::undefined);
dr4b.spin(vex::reverse, 100, vex::percent);
fourBar2.spin(vex::reverse, 100, vex::percent);




}
else {
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