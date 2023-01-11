#include <vex.h>
#include "pid.h" 
#include "odom.h"
#include <vector>
#include <cmath>
#include <math.h>
#include "iostream"

using namespace std;

typedef long long ll;
typedef int i;
typedef double d;
typedef string s;
typedef odom o;
typedef pid p;

// Wheel 
double wheelradius = 4; 
double wheelCircumference = M_PI * (wheelradius * 2);

double LT = 0; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE
double RT = 0; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE
double BT = 0; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE 
double theta0 = 0;
double theta1 = 0;
double dTheta = theta1 - theta0;

double odom::x = 0;
double odom::y = 0;
double odom::a = 180;

bool odom::active = true;

#define INIT_GYRO_HEADING 90

odom* pOdom = 0;

//odom* tracker_Timer = 0; Trial

odom::odom() {
  DB = 0;
  dx = 0;
  thetaM = 0;
  dy = 0;

  z = 0;

  DR = 0;

  // Wheel
  wheelradius = 2.5/2; 
  wheelCircumference = 3.1415926535 * (wheelradius * 2);


  TL = 5.75; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE
  TR = 5.75; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE
  TB = 5.25; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE

  theta1 = 0;
  // ONLY WHILE TESTING
  //odom::testCalibration();
  //
  Gyro.setHeading(INIT_GYRO_HEADING, degrees);
  //frontRight.setPosition(0, degrees);
  //frontLeft.setPosition(0, degrees);
  //middleRight.setPosition(0, degrees);
  //middleLeft.setPosition(0, degrees);
  //backLeft.setPosition(0, degrees);
  //backRight.setPosition(0, degrees);
  //dTheta = theta1 - theta0;
  //LE.setPosition(0, degrees);
  //RE.setPosition(0, degrees);
}

// GYRO CALIBRATION FOR AUTON TESTING
void odom::testCalibration() {
  Gyro.calibrate();
  while(Gyro.isCalibrating()) {
    wait(1, msec);
  }
}

double odom::radianConversion(double theta) {
  return (theta * 3.1415926535)/(180);
}

/*
void odom::TrackPosition() {
  //double conversion = 90/37.23;
  while(odom::active) { 
    
    DL = (LE.position(degrees)/360) * wheelCircumference;
    DR = (RE.position(degrees)/360) * wheelCircumference;
    z = (DL + DR)/2;
      
    if ((DL + DR) <= 5 && (DL + DR) >= -5) {    
      dTheta = ((abs(DL) + abs(DR))/(TL + TR));     
       if (DL > 0) {
         theta1 -= dTheta;
       } else if (DL < 0){
         theta1 += dTheta;
       }
    } 
    else if (DL > 0 && DR > 0) {
      if (DR>DL) {     
        dTheta = (DR-DL)/(TL+TR);
        theta1 = theta1 + dTheta;
      } else if (DL>DR){
        dTheta = (DL-DR)/(TL+TR);
        theta1 = theta1 - dTheta;
      } 
    }
    else if (DL < 0 && DR < 0) {
      if (DR<DL) {
        dTheta = (abs(DR)-abs(DL))/(TL+TR);
        theta1 -= dTheta;  
      } else if (DL<DR) {
        dTheta = (abs(DL)-abs(DR))/(TL+TR);
        theta1 += dTheta;
      }
    }
    else if ((abs(DL) - abs(DR)) <= 0.01 && (abs(DL) - abs(DR)) >= -0.01) {
      dTheta = 0;
      theta1 += dTheta;
    }
      
    dx = z*cos(theta1);
    dy = z*sin(theta1);

    odom::x += dx;
    odom::y += dy;
    Brain.Screen.print("RE: %f, LE: %f", RE.position(degrees), LE.position(degrees));
    //Brain.Screen.print("theta1: %f, heading: %f ", (theta1 * 180)/3.1415926535, Gyro.heading(degrees));
    //Brain.Screen.print("X: %f, Y: %f, theta1: %f", DL, DR, (theta1 * 180)/3.1415926535);
    Brain.Screen.newLine();
    //LE.setPosition(0, degrees);
    //RE.setPosition(0, degrees);
    //wait(10, msec);
    

    
    wait(10, msec);
  }
  wait(1, sec);
}
*/
/*
void odom::TrackPosition() {
   while(odom::active) { 
    DL = (LE.position(degrees)/360) * wheelCircumference;
    DR = (RE.position(degrees)/360) * wheelCircumference;
    DB = (BE.position(degrees)/360) * wheelCircumference;
    z = (DL + DR)/2;

    dTheta = (DL - DR)/(TL + TR); 

    //dTheta = (DR - DL)/(TL +TR);

    theta1 += dTheta;
    //dx = -z*cos(theta1);
    //dy = z*sin(theta1);
    //dx = 2 * sin(theta1/2) * ((DB/theta1) + TB);
    //dy = 2 * sin(theta1/2) * ((z/theta1) + TR);
    dy = z * cos(theta1) + DB * sin(theta1);
    dx = DB * cos(theta1) - z * sin(theta1);
    //dx = floor(dx*100 + 0.5)/100;
    //dy = floor(dy*100 + 0.5)/100;

    odom::x += dx;
    odom::y += dy;
    if (theta1 >= 0) {
      odom::a = fmod((90 - (theta1 * 180)/3.1415926535), 360);
      //odom::a = fmod((180 - (theta1 * 180)/3.1415926535), 360); PREVIOUS
      //odom::a = fmod((180 - (theta1 * 180)/3.1415926535), 360); WRONG
    } else {
      odom::a = 180 - fmod((90-(abs(theta1) * 180)/3.1415926535), 360);
      //odom::a = 360 - fmod((180-(abs(theta1) * 180)/3.1415926535), 360); PREVIOUS
      //odom::a = 360 - fmod((180 - (abs(theta1) * 180)/3.1415926535), 360);
    } 
    //odom::a = 360 - odom::a;
    Brain.Screen.print("RE: %f, LE: %f, BE: %f", RE.position(degrees), LE.position(degrees), BE.position(degrees));
    //Brain.Screen.print("theta1: %f, heading: %f ", (theta1 * 180)/3.1415926535, Gyro.heading(degrees));
    //Brain.Screen.print(360 - Gyro.heading());
    //Brain.Screen.print("X: %f, Y: %f, theta1: %f", odom::x, odom::y, odom::a);
    //cout << "X: " << odom::x << " Y: " << odom::y << " odom::a: " << odom::a << "\n";
    //Brain.Screen.print(RotationTilter.angle());
    //Brain.Screen.print("dx: %f, dy: %f", dx, dy);
    Brain.Screen.newLine();
    //LE.setPosition(0, degrees);
    //RE.setPosition(0, degrees);
    //BE.setPosition(0, degrees);
    wait(1, msec);
   }
}*/
void odom::TrackPosition() {
   while(odom::active) { 
    DL = (frontLeft.position(degrees) + middleLeft.position(degrees) + backLeft.position(degrees))/3;
    DR = (frontRight.position(degrees) + middleRight.position(degrees) + backRight.position(degrees))/3;
    //DB = backRot.angle();
    
    z = (DL + DR)/2;

    dTheta = (z * (3.1415926535))/ 180;//(DL - DR)/(TL + TR); 

    //dTheta = (DR - DL)/(TL +TR);
    
    theta1 += dTheta;
    dx = -z*cos(theta1);
    dy = z*sin(theta1);
    //dx = -z*cos(theta1);
    //dy = z*sin(theta1);
    //dx = 2 * sin(theta1/2) * ((DB/theta1) + TB);
    //dy = 2 * sin(theta1/2) * ((z/theta1) + TR);
    /////////////////////////dy = z * cos(theta1) + DB * sin(theta1); <---- Good Encoder Code // Uncomment if Tracking Wheels
    /////////////////////////dx = DB * cos(theta1) - z * sin(theta1); <---- Good Encoder Code // Uncomment if Tracking Wheels
    //dx = floor(dx*100 + 0.5)/100;
    //dy = floor(dy*100 + 0.5)/100;

    

    odom::x += dx;
    odom::y += dy;
    /*
    if (theta1 >= 0) {
      odom::a = fmod((90 - (theta1 * 180)/3.1415926535), 360);
      //odom::a = fmod((180 - (theta1 * 180)/3.1415926535), 360); PREVIOUS
      //odom::a = fmod((180 - (theta1 * 180)/3.1415926535), 360); WRONG
    } else {
      odom::a = 180 - fmod((90-(abs(theta1) * 180)/3.1415926535), 360);
      //odom::a = 360 - fmod((180-(abs(theta1) * 180)/3.1415926535), 360); PREVIOUS
      //odom::a = 360 - fmod((180 - (abs(theta1) * 180)/3.1415926535), 360);
    } */
    odom::a = Gyro.rotation(degrees);//z;
    //Brain.Screen.print("DL: %f, DR: %f", odom::x, odom::y);
    //odom::a = 360 - odom::a;
    Brain.Screen.print("frontLeftPos: %f", frontLeft.position(degrees) * (600/360));
    //Brain.Screen.print("RE: %f, LE: %f, BE: %f", RE.position(degrees), LE.position(degrees), BE.position(degrees));
    //Brain.Screen.print("theta1: %f, heading: %f ", (theta1 * 180)/3.1415926535, Gyro.heading(degrees));
    //Brain.Screen.print("Heading: %f", Gyro.heading(degrees));
    //Brain.Screen.print("X: %f, Y: %f, theta1: %f", odom::x, odom::y, odom::a);
    //cout << "X: " << odom::x << " Y: " << odom::y << " odom::a: " << odom::a << "\n";
    //Brain.Screen.print(RotationTilter.angle());
    //Brain.Screen.print("dx: %f, dy: %f", dx, dy);
    Brain.Screen.newLine();
    //frontLeft.setPosition(0, degrees);
    //frontRight.setPosition(0, degrees);
    //backLeft.setPosition(0, degrees);
    //backRight.setPosition(0, degrees);
    //middleLeft.setPosition(0, degrees);
    //middleRight.setPosition(0, degrees);
    //LE.setPosition(0, degrees);
    //RE.setPosition(0, degrees);
    //BE.setPosition(0, degrees);
    wait(1, msec);
   }
}
void odom::positionTracking() {
  if (pOdom) {
    pOdom->TrackPosition();
  } 
}

void odom::moveForwardPIDDistance(double cap) {
  pid* PIDActivator = new pid();
  bool vel1 = true;
  double d = -1*(Distance.objectDistance(inches)-4.4);
  //double x, y;
  frontRight.setStopping(coast);
  frontLeft.setStopping(coast);
  middleRight.setStopping(coast);
  middleLeft.setStopping(coast);
  backLeft.setStopping(coast);
  backRight.setStopping(coast);
  
  frontRight.setPosition(0, degrees);
  frontLeft.setPosition(0, degrees);
  middleRight.setPosition(0, degrees);
  middleLeft.setPosition(0, degrees);
  backLeft.setPosition(0, degrees);
  backRight.setPosition(0, degrees);

  

  double curr_pos = backLeft.position(degrees) * ((double)36.0/60.0) + backRight.position(degrees) * ((double)36.0/60.0); 
  curr_pos = (curr_pos/(double)2.0);
  double dist = ((d/(3.25*3.1415926535)) * 360.0) - curr_pos;
  double axial = PIDActivator->axial(dist, cap);



  while(abs(dist) > 5) {

    axial = PIDActivator->axial(dist, cap);
    curr_pos = backLeft.position(degrees) * ((double)36.0/60.0) + backRight.position(degrees) * ((double)36.0/60.0);
    curr_pos = (curr_pos/(double)2.0);
    dist = ((d/(3.25*3.1415926535)) * 360.0) - curr_pos;

    odom::standardDrive(axial, 0);

    wait(15, msec);
  }

  frontRight.stop();
  frontLeft.stop();
  middleRight.stop();
  middleLeft.stop();
  backLeft.stop();
  backRight.stop();
  //return d;
}


void odom::moveForward(double lon) {
  double deg = (lon/(3.25 * 3.1415926535)) * 360;
  frontRight.setVelocity(60, percent);
  backRight.setVelocity(60, percent);
  middleRight.setVelocity(60, percent);
  frontLeft.setVelocity(60, percent);
  backLeft.setVelocity(60, percent);
  middleLeft.setVelocity(60, percent);
   frontRight.startRotateFor(vex::directionType::fwd, deg, vex::rotationUnits::deg);
   backRight.startRotateFor(vex::directionType::fwd, deg, vex::rotationUnits::deg);
   middleRight.startRotateFor(vex::directionType::fwd, deg, vex::rotationUnits::deg);
   frontLeft.startRotateFor(vex::directionType::fwd, deg, vex::rotationUnits::deg);
   middleLeft.startRotateFor(vex::directionType::fwd, deg, vex::rotationUnits::deg);
   backLeft.rotateFor(vex::directionType::fwd, deg, vex::rotationUnits::deg);

   frontRight.stop();
   frontLeft.stop();
   backRight.stop();
   backLeft.stop();
   middleRight.stop();
   middleLeft.stop();
  
  //frontRight.spinFor(vex::forward, deg, degrees);
  //frontLeft.spinFor(vex::forward, deg, degrees);
  //backRight.spinFor(vex::forward, deg, degrees);
  //backLeft.spinFor(vex::forward, deg, degrees);
}



void odom::moveReverse(double lon) {
  double deg = (lon/(3.25 * 3.1415926535)) * 360;
  frontRight.spinFor(vex::reverse, deg, degrees);
  frontLeft.spinFor(vex::reverse, deg, degrees);
  backRight.spinFor(vex::reverse, deg, degrees);
  backLeft.spinFor(vex::reverse, deg, degrees);
}

void odom::moveChainBar(double pos) {
  twoBar.spinFor(vex::forward, pos, degrees);
}



void odom::standardDrive(double lon, double rot) {  
  // Respective standard drive kinematic equations for each motor
    
    frontRight.spin(vex::forward, lon - rot, vex::rpm);
    frontLeft.spin(vex::forward, lon + rot, vex::rpm);
    middleRight.spin(vex::forward, lon - rot, vex::rpm);
    middleLeft.spin(vex::forward, lon + rot, vex::rpm);
    backRight.spin(vex::forward, lon - rot, vex::rpm);
    backLeft.spin(vex::forward, lon + rot, vex::rpm);
    /*
    frontRight.spin(vex::forward, lon - rot, voltageUnits::volt);
    frontLeft.spin(vex::forward, lon + rot, voltageUnits::volt);
    middleRight.spin(vex::forward, lon - rot, voltageUnits::volt);
    middleLeft.spin(vex::forward, lon + rot, voltageUnits::volt);
    backRight.spin(vex::forward, lon - rot, voltageUnits::volt);
    backLeft.spin(vex::forward, lon + rot, voltageUnits::volt);*/
}

void odom::rotStandardDrive(double lon, double rot) {
  frontRight.spin(vex::forward, lon - rot, vex::rpm);
  frontLeft.spin(vex::forward, lon + rot, vex::rpm);
  middleRight.spin(vex::forward, lon - rot, vex::rpm);
  middleLeft.spin(vex::forward, lon + rot, vex::rpm);
  backRight.spin(vex::forward, lon - rot, vex::rpm);
  backLeft.spin(vex::forward, lon + rot, vex::rpm);
}

void odom::turnTo(double desiredHeading) {
  
  pid* PIDActivator = new pid();
  frontRight.setStopping(hold);
  frontLeft.setStopping(hold);
  middleRight.setStopping(hold);
  middleLeft.setStopping(hold);
  backRight.setStopping(hold);
  backLeft.setStopping(hold);
  double a_error =  desiredHeading - (Gyro.heading(degrees)); //desiredHeading - (360 - Gyro.heading(degrees))//odom::a - desiredHeading;
  while(abs(a_error) > 2) {
  //while((INIT_GYRO_HEADING - desiredHeading)>=0?(a_error > 0.5):(a_error < -0.5)){
    double rotational = PIDActivator->gyroPid(a_error);
    //cout <<"Heading: " << Gyro.heading(degrees) << " Angle error: " << a_error << " Rotational: " << rotational << "\n";
    a_error = desiredHeading - (Gyro.heading(degrees));//odom::a - desiredHeading;
    odom::rotStandardDrive(0, rotational);
    //cout << "Angle error: " << a_error << "\n";
    wait(1, msec);
  } 
  
  frontRight.stop();
  frontLeft.stop();
  middleRight.stop();
  middleLeft.stop();
  backLeft.stop();
  backRight.stop();
}

void odom::reverseTurn(double desiredHeading) {
  pid* PIDActivator = new pid();
  frontRight.setStopping(hold);
  frontLeft.setStopping(hold);
  middleRight.setStopping(hold);
  middleLeft.setStopping(hold);
  backRight.setStopping(hold);
  backLeft.setStopping(hold);
  double sub = 0;
  if (Gyro.heading(degrees) <= 180) {
    sub = Gyro.heading(degrees) + 360;
  } else {
    sub = Gyro.heading(degrees);
  }
  double a_error =  desiredHeading - sub;//odom::a - desiredHeading;
  while(abs(a_error) > 2) {
  //while((INIT_GYRO_HEADING - desiredHeading)>=0?(a_error > 0.5):(a_error < -0.5)){
    double rotational = PIDActivator->gyroPid(a_error);
    //cout <<"Heading: " << Gyro.heading(degrees) << " Angle error: " << a_error << " Rotational: " << rotational << "\n";
    if (Gyro.heading(degrees) <= 180) {
      sub = Gyro.heading(degrees) + 360;
    } else {
      sub = Gyro.heading(degrees);
    }
    a_error =  desiredHeading - sub;//odom::a - desiredHeading;
    odom::standardDrive(0, rotational);
    //cout << "Angle error: " << a_error << "\n";
    wait(1, msec);
  } 
  
  frontRight.stop();
  frontLeft.stop();
  middleRight.stop();
  middleLeft.stop();
  backLeft.stop();
  backRight.stop();
}

/*
void odom::moveForwardPID(double x, double y, bool f) {
  pid* PIDActivator = new pid();
  double dist = sqrt((x - odom::x) * (x - odom::x) + (y - odom::y) * (y - odom::y));
  double axial = PIDActivator->axial((abs(dist)/(4*3.1415926535)) * 360);
  while(abs(dist) > 2) {
    axial = PIDActivator->axial((abs(dist)/(4*3.1415926535)) * 360);
    if (!f) axial *= -1;
    //cout << "Rotational: " << rotational << "\n";
    dist = sqrt((x - odom::x) * (x - odom::x) + (y - odom::y) * (y - odom::y));
    
    odom::standardDrive(axial, 0); 
    cout << "Dist error: " << dist << "\n";
    wait(1, msec);
  } 
  
  frontRight.stop();
  frontLeft.stop();
  backLeft.stop();
  backRight.stop();
}*/
void odom::moveForwardPID(double d, double cap) {
  pid* PIDActivator = new pid();
  bool vel = true;
  //double x, y;
  frontRight.setStopping(coast);
  frontLeft.setStopping(coast);
  middleRight.setStopping(coast);
  middleLeft.setStopping(coast);
  backLeft.setStopping(coast);
  backRight.setStopping(coast);
  
  frontRight.setPosition(0, degrees);
  frontLeft.setPosition(0, degrees);
  middleRight.setPosition(0, degrees);
  middleLeft.setPosition(0, degrees);
  backLeft.setPosition(0, degrees);
  backRight.setPosition(0, degrees);

  

  //x = odom::x + d * cos(odom::a * (3.14159268/180));
  //y = odom::y + d * sin(odom::a * (3.14159268/180));
  //cout << x << y;
  
  double curr_pos = backLeft.position(degrees) * ((double)36.0/60.0) + backRight.position(degrees) * ((double)36.0/60.0); //+ frontRight.position(degrees) * ((double)36/60);//frontLeft.position(degrees) * (600/360);
  //curr_pos += middleLeft.position(degrees) * ((double)36.0/60.0) + middleRight.position(degrees) * ((double)36.0/60.0);
  //curr_pos += backLeft.position(degrees) * ((double)36.0/60.0) + backRight.position(degrees) * ((double)36.0/60.0);
  curr_pos = (curr_pos/(double)2.0);//(curr_pos/(double)6.0);
  //double dist = ((abs(d)/(3.25*3.1415926535)) * 360.0) - abs(curr_pos);
  double dist = ((d/(3.25*3.1415926535)) * 360.0) - curr_pos;
  double axial = PIDActivator->axial(dist, cap);//PIDActivator->axial(((abs(dist)/(4*3.1415926535)) * 360),cap);

  //if (d < 0) {
   // vel = false;
  //}

  while(abs(dist) > 5) {

    axial = PIDActivator->axial(dist, cap);//PIDActivator->axial(((abs(dist)/(4*3.1415926535)) * 360), cap);
    //if (!vel) axial *= -1;
    //Brain.Screen.print("%lf\n", axial);
    cout << "Dist: " << dist << "\n";
    curr_pos = backLeft.position(degrees) * ((double)36.0/60.0) + backRight.position(degrees) * ((double)36.0/60.0);
    //curr_pos += middleLeft.position(degrees) * ((double)36.0/60.0) + middleRight.position(degrees) * ((double)36.0/60.0);
    //curr_pos += backLeft.position(degrees) * ((double)36.0/60.0) + backRight.position(degrees) * ((double)36.0/60.0);
    curr_pos = (curr_pos/(double)2.0);//(curr_pos/(double)6.0);
    //dist = ((abs(d)/(3.25*3.1415926535)) * 360.0) - abs(curr_pos); 
    dist = ((d/(3.25*3.1415926535)) * 360.0) - curr_pos;

    odom::standardDrive(axial, 0);

    wait(15, msec);
    //wait(50, msec);
  }
  /* 
  while(abs(dist) > 2) {
    odom::standardDrive(50, 0);
  }*/

  //odom::x = 0;
  //odom::y = 0;

  frontRight.stop();
  frontLeft.stop();
  middleRight.stop();
  middleLeft.stop();
  backLeft.stop();
  backRight.stop();
}


void odom::moveForwardOdom(double d, double v) {
  //pid* PIDActivator = new pid();
  bool vel = true;
  double x, y;
  //if (d<0){
   //x = odom::x-d * cos(odom::a * (3.14159268/180));
   //y = odom::y-d * sin(odom::a* (3.14159268/180));
  //vel = false;
  if (d < 0) {
    vel = false;
  }
  x = odom::x + d * cos(odom::a * (3.14159268/180));
  y = odom::y + d * sin(odom::a * (3.14159268/180));
  
  cout << x <<y;
 
 
 

  double dist = sqrt(x*x + y*y) - sqrt(odom::x * odom::x + odom::y * odom::y);//sqrt((x - odom::x) * (x - odom::x) + (y - odom::y) * (y - odom::y));
  double axial = v;
  while(abs(dist) > 2) {
    axial = v;
    if (!vel) axial *= -1;
    Brain.Screen.print("%lf\n", axial);
    cout << "Dist: " << dist << "\n";
    dist = sqrt(x*x + y*y) - sqrt(odom::x * odom::x + odom::y * odom::y);

    odom::standardDrive(axial, 0);

    wait(1, msec);
  } 

  //odom::x = 0;
  //odom::y = 0;

  //frontRight.stop();
  //frontLeft.stop();
  //backLeft.stop();
  //backRight.stop();
  frontRight.setStopping(brake);
  frontLeft.setStopping(brake);
  backLeft.setStopping(brake);
  backRight.setStopping(brake);
}



void odom::moveTo(double target_x, double target_y, bool f, bool head_x) {
  double y_error = target_y - odom::y;
  double x_error = target_x - odom::x;
  double theta;
  if (x_error == 0) {
    theta = 0;
  } else {
    theta = odom::a - ((atan2(y_error,x_error) * 180)/3.1415926535);
  } 
  double longitudinal = y_error;
  //Brain.Screen.print("X_error: %d, Y_error: %d", x_error, y_error);
  //Brain.Screen.newLine();
  //float start_Time = Brain.Timer.value();
  //float end_Time;
  //float movement_Time;
  //vector<double> vector = {x_error, y_error}; 

  pid* PIDActivator = new pid();
  /*
  while ((abs(phi) > 1 && abs(y_error) > 1 && abs(x_error) > 1) && (odom::a != (phi * 180)/3.1415926535)) {
    double longitudinal = (y_error * std::cos(phi) + x_error * std::sin(phi));
    double lateral = (x_error * std::cos(phi) + y_error * std::sin(phi));
    
    double rotational = PIDActivator->gyroPid(phi);
    double axial = PIDActivator->axial(sqrt( pow(longitudinal, 2) + pow(lateral, 2) ));
    
    odom::standardDrive(longitudinal, rotational);
  }*/
  double target_heading = 0;
  double add  = 0;
  while (!(abs(x_error) < 1 && abs(y_error) < 1 && abs(theta) < 1) ) {
    y_error = target_y - odom::y;
    x_error = target_x - odom::x;

    if (head_x) {
      longitudinal = x_error;
      add = (y_error < 0) ? 180 : 0;
    } else {
      longitudinal = y_error;
      add = (x_error < 0) ? 180 : 0;
    }
    //longitudinal = (abs(x_error)>=abs(y_error)) ? x_error : y_error;

    //add = (longitudinal >= 0) ? 0 : 180;
  
    target_heading = (atan(target_y/target_x) * 180)/3.1415926535 + add;

    if (target_heading < 0) {
      target_heading += 360;
    }

    if (abs(x_error)<1 || abs(y_error) < 1) {
      theta = 0;
    } else {
      if (abs(x_error)>=abs(y_error)){
        theta = odom::a - target_heading;
      } else {
        theta = odom::a - target_heading;
      }
    } 

    if (abs(theta) > 1 && !f) {
      theta += 180;
    }
    //theta = odom::a - (atan(y_error/x_error) * 180)/3.1415926535;//((atan((y_error/x_error)) * 180)/3.1415926535);
    cout << "X error: " << x_error << " Y error: " << y_error << " theta: " << theta << " add:" << add << " target-head:" << target_heading << "\n";
    //if (theta < 0) {
      //theta += 180;
    //}
    if (abs(x_error) < 1 && abs(y_error) < 1 && abs(theta) < 1) break;
    
    

    //cout << "longitudinal: " << longitudinal << "\n";
    
    double rotational;
    double axial;

    if(abs(theta) > 1) {
      rotational = PIDActivator->gyroPid(theta);
      axial = PIDActivator->axial(((abs(longitudinal)/(4*3.1415926535)) * 360), 10000000000000);
    } else {
      rotational = 0;
      axial = PIDActivator->axial(((abs(longitudinal)/(4*3.1415926535)) * 360), 100000000000000);
    }
    
    if(axial <= 10 && axial >= 10.5) break; 

    if (!f) {
      axial *= -1;
    } 

    //Brain.Screen.print("axial: %f, rotational: %f", axial, rotational);
    //Brain.Screen.newLine();
    odom::standardDrive(axial, rotational);

    wait(1, msec);
  }
  //create reinitialize method in PID
  //add tolerance for odom::x and odom::y in while loop
  frontRight.stop();
  frontLeft.stop();
  backLeft.stop();
  backRight.stop();
  
}