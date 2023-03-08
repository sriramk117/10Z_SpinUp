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

#define INIT_GYRO_HEADING 90
// Wheel 
double wheelradius = 3.25; 
double wheelCircumference = M_PI * (wheelradius * 2);

double LT = 0; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE
double RT = 0; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE
double BT = 0; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE 
double theta0 = 0;
double theta1 = 0;
double dTheta = theta1 - theta0;

double odom::x = 36; // up for modification
double odom::y = 12;  // up for modification
double odom::a = INIT_GYRO_HEADING;

bool odom::active = true;



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
  wheelradius = 3.25/2; 
  wheelCircumference = 3.1415926535 * (wheelradius * 2);


  TL = 5.5;//5.75; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE
  TR = 5.5;//5.75; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE
  TB = 5.25; // CURRENTLY ASSIGNED RANDOM VALUE, CHANGE LATER AFTER TESTING DRIVE

  theta1 = 3.1415926535/2;
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

double correctAngle(double angleDeg) { 
  while(!(angleDeg >= -180 && angleDeg < 180)) {
    if( angleDeg < -180 ) { 
      angleDeg += 360; 
    }
    if(angleDeg >= 180) { 
      angleDeg -= 360;
    }
  }
  return(angleDeg);
}

double odom::radianConversion(double theta) {
  return (theta * 3.1415926535)/(180);
}


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
    DL = (double)(frontLeft.position(degrees) + middleLeft.position(degrees) + backLeft.position(degrees))/3.0;
    DR = (double)(frontRight.position(degrees) + middleRight.position(degrees) + backRight.position(degrees))/3.0;
    
    DL *= (48.0/72.0);//(48.0/72.0) * 1.6;
    DR *= (48.0/72.0);//(48.0/72.0) * 1.6;

    DL /= 360.0; DL *= 3.25*3.1415926535;
    DR /= 360.0; DR *= 3.25*3.1415926535;
    
    z = (DL + DR)/2;

    //dTheta = (DL - DR)/(TL + TR);//(z * (3.1415926535))/ 180;//(DL - DR)/(TL + TR); 
    
    //theta1 += dTheta;
    theta1 = (Gyro.heading(degrees) * 3.1415926535) / 180.0;
    dx = -z*cos(theta1);//theta1
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
    odom::a = Gyro.heading(degrees);//z;
    
    //Brain.Screen.print("DL: %f, DR: %f", odom::x, odom::y);
    //odom::a = 360 - odom::a;
    //Brain.Screen.print("frontLeftPos: %f, frontRightPos: %f", DL, DR);
    //Brain.Screen.print("RE: %f, LE: %f, BE: %f", RE.position(degrees), LE.position(degrees), BE.position(degrees));
    //Brain.Screen.print("theta1: %f, heading: %f ", (theta1 * 180)/3.1415926535, Gyro.heading(degrees));
    //Brain.Screen.print("Heading: %f", Gyro.heading(degrees));
    Brain.Screen.print("X: %f, Y: %f, theta1: %f", odom::x, odom::y, odom::a);
    Brain.Screen.newLine();


    frontLeft.setPosition(0, degrees);
    frontRight.setPosition(0, degrees);
    backLeft.setPosition(0, degrees);
    backRight.setPosition(0, degrees);
    middleLeft.setPosition(0, degrees);
    middleRight.setPosition(0, degrees);
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

  if(PIDActivator) {
    delete PIDActivator;
    PIDActivator = 0;
  }
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
    
}

void odom::rotStandardDrive(double lon, double rot) {
  frontRight.spin(vex::forward, lon - rot, vex::rpm);
  frontLeft.spin(vex::forward, lon + rot, vex::rpm);
  middleRight.spin(vex::forward, lon - rot, vex::rpm);
  middleLeft.spin(vex::forward, lon + rot, vex::rpm);
  backRight.spin(vex::forward, lon - rot, vex::rpm);
  backLeft.spin(vex::forward, lon + rot, vex::rpm);
}

void odom::turnTo(double desiredHeading, double cap) {
  pid* PIDActivator = new pid();
  frontRight.setStopping(hold);
  frontLeft.setStopping(hold);
  middleRight.setStopping(hold);
  middleLeft.setStopping(hold);
  backRight.setStopping(hold);
  backLeft.setStopping(hold);
  double a_error =  desiredHeading - (Gyro.heading(degrees)); //desiredHeading - (360 - Gyro.heading(degrees))//odom::a - desiredHeading;
  while(abs(a_error) > 2) {
    double rotational = PIDActivator->gyroPid(a_error, cap);
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
  
  if(PIDActivator) {
    delete PIDActivator;
    PIDActivator = 0;
  }
}

void odom::reverseTurn(double desiredHeading, double cap) {
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
  while(fabs(a_error) > 2) {
  //while((INIT_GYRO_HEADING - desiredHeading)>=0?(a_error > 0.5):(a_error < -0.5)){
    double rotational = PIDActivator->gyroPid(a_error, cap);
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

  if(PIDActivator) {
    delete PIDActivator;
    PIDActivator = 0;
  }
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

  

  x = odom::x + d * cos(odom::a * (3.14159268/180)) * -1;
  y = odom::y + d * sin(odom::a * (3.14159268/180));
  //cout << x << y;

  // Relative PID
  
  /*
  double curr_pos = backLeft.position(degrees) * ((double)36.0/60.0) + backRight.position(degrees) * ((double)36.0/60.0); 
  curr_pos = (curr_pos/(double)2.0);
  double dist = ((abs(d)/(3.25*3.1415926535)) * 360.0) - abs(curr_pos);
  double dist = ((d/(3.25*3.1415926535)) * 360.0) - curr_pos;
  double axial = PIDActivator->axial(dist, cap);
  */

  // Global PID

  double x_error = x - odom::x;
  double y_error = y - odom::y;
  double dist = sqrt(x_error*x_error + y_error*y_error);
  double axial = PIDActivator->axial(((dist/(3.25*3.1415926535)) * 360.0), cap);
  

  while(abs(dist) > 2) {
    axial = PIDActivator->axial(((dist/(3.25*3.1415926535)) * 360.0), cap);
    
    // Relative PID
    /*
    curr_pos = backLeft.position(degrees) * ((double)48.0/72.0) + backRight.position(degrees) * ((double)48.0/72.0);
    curr_pos = (curr_pos/(double)2.0);
    dist = ((d/(3.25*3.1415926535)) * 360.0) - curr_pos;*/

    // Global PID
    x_error = x - odom::x;
    y_error = y - odom::y;
    dist = sqrt(x_error*x_error + y_error*y_error);

    odom::standardDrive(axial, 0);

    wait(15, msec);
  }
  
  frontRight.stop();
  frontLeft.stop();
  middleRight.stop();
  middleLeft.stop();
  backLeft.stop();
  backRight.stop();

  if(PIDActivator) {
    delete PIDActivator;
    PIDActivator = 0;
  }
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
  
  //cout << x << y; 

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

pair<double, double> normalize(double x, double y) {
  double imag = sqrt(x * x + y * y);
  if (imag == 0) {
    return {x, y};
  }
  return {x / imag, y / imag};
}

pair<double, double> closest(pair<double, double> curr, pair<double, double> target) {
  double curr_heading = (Gyro.heading(degrees) * 3.1415926535)/180.0;
  double head_x = -cos(curr_heading);
  double head_y = sin(curr_heading);
  
  pair<double, double> n = normalize(head_x, head_y); 
  pair<double, double> v = {target.first - curr.first, target.second - curr.second};
  
  double d = v.first * n.first + v.second * n.second;
  
  pair<double, double> scalar = {n.first * d, n.second * d};
  
  return {curr.first + scalar.first, curr.second + scalar.second};
}

void odom::moveTo(double target_x, double target_y, double dir, double turnScale, double tolD,  double capD, double capA, double settleTime) {
  double start = Brain.timer(sec); 
  
  frontRight.setStopping(hold);
  frontLeft.setStopping(hold);
  middleRight.setStopping(hold);
  middleLeft.setStopping(hold);
  backLeft.setStopping(hold);
  backRight.setStopping(hold);

  double y_error = target_y - odom::y;
  double x_error = target_x -  odom::x;
  
  double curr_a = Gyro.heading(degrees);
  double target = ((atan2(x_error,y_error) * 180.0)/3.1415926535); 
  target += 90.0;
  if (target < 0) {
    target = 360.0 + target;
  }

  if (dir < 0) {
    curr_a = odom::a + 180.0;
    if (curr_a > 360.0) {
      curr_a -= 360.0;
    }
  }

  double theta = correctAngle(target - curr_a);

  double distTarget = sqrt(y_error*y_error + x_error*x_error);
  double distClosest = sqrt(y_error*y_error + x_error*x_error);
  double dist = sqrt(y_error*y_error + x_error*x_error);
  
  pid* PIDActivator = new pid();
  while (!(abs(dist) < tolD) && (Brain.timer(sec) < start + settleTime)) {
    y_error = target_y - odom::y;
    x_error = target_x - odom::x;

    distTarget = sqrt(y_error*y_error + x_error*x_error);
    distTarget *= dir;
    
    target = ((atan2(x_error,y_error) * 180.0)/3.1415926535);
    target += 90.0;
    if (target < 0) {
      target = 360.0 + target;
    }
    
    curr_a = Gyro.heading(degrees);
    if (dir < 0) {
      curr_a = odom::a + 180.0;
      if (curr_a > 360.0) {
        curr_a -= 360.0;
      }
    }

    // testing this out (remove and uncomment code if doesn't work)
    pair<double, double> closestPoint = closest({odom::x, odom::y}, {target_x, target_y}); 
    if (abs(distTarget) < 15) {
      y_error = closestPoint.second - odom::y;
      x_error = closestPoint.first - odom::x;
      distClosest = sqrt(y_error*y_error + x_error*x_error);
      distClosest *= dir;
      dist = distClosest;
      theta = 0;
    } else {
      dist = distTarget;
      theta = correctAngle(target - curr_a);
    }
    
    
    double rotational;
    double axial;

    rotational = PIDActivator->gyroPid(theta, capA);
    axial = PIDActivator->axial(((dist/(3.25*3.1415926535)) * 360.0), capD);
    
    odom::standardDrive(axial, rotational*turnScale);

    wait(2, msec); //wait(10, msec)
  }
  
  frontRight.stop();
  frontLeft.stop();
  middleRight.stop();
  middleLeft.stop();
  backLeft.stop();
  backRight.stop();

  if(PIDActivator) {
    delete PIDActivator;
    PIDActivator = 0;
  }
  
}


void odom::setPoint(double target_x, double target_y, double dir, double turnScale, double tolD, double capD, double capA, double settleTime) {
  double start = Brain.timer(sec); 
  
  frontRight.setStopping(hold);
  frontLeft.setStopping(hold);
  middleRight.setStopping(hold);
  middleLeft.setStopping(hold);
  backLeft.setStopping(hold);
  backRight.setStopping(hold);

  double y_error = target_y - odom::y;
  double x_error = target_x -  odom::x;
  
  double curr_a = Gyro.heading(degrees);//odom::a;
  double target = ((atan2(x_error,y_error) * 180.0)/3.1415926535); //((atan2(x_error,y_error) * 180.0)/3.1415926535);
  target += 90.0;
  if (target < 0) {
    target = 360.0 + target;
  }

  if (dir < 0) {
    curr_a = odom::a + 180.0;
    if (curr_a > 360.0) {
      curr_a -= 360.0;
    }
  }

  double theta = correctAngle(target - curr_a);

  double thetaRad = theta/180.0 * 3.1415926535;
  //double turnScale = cos(thetaRad);

  double dist = sqrt(y_error*y_error + x_error*x_error);
  
  //theta = 180.0/3.1415926535 * atan2(x_error, y_error) - target_a;
  //thetaRad = theta/180.0 * 3.1415926535;
  
  //double sideError = dist * sin(thetaRad);
  //theta = sideError*agg - odom::a;
  
  pid* PIDActivator = new pid();
  bool thetaTol = false;
  
  while ((!(abs(dist) < tolD)) && (Brain.timer(sec) < start + settleTime)) {
    y_error = target_y - odom::y;
    x_error = target_x - odom::x;

    dist = sqrt(y_error*y_error + x_error*x_error);
    
    dist *= dir;
    
    
    target = ((atan2(x_error,y_error) * 180.0)/3.1415926535);
    target += 90.0;
    if (target < 0) {
      target = 360.0 + target;
    }
    
    curr_a = Gyro.heading(degrees);
    if (dir < 0) {
      curr_a = odom::a + 180.0;
      if (curr_a > 360.0) {
        curr_a -= 360.0;
      }
    }

    theta = correctAngle(target - curr_a);
    

    thetaRad = theta/180.0 * 3.1415926535;
    //turnScale = cos(thetaRad);
    
    
    //if (abs(dist) < tolD && abs(theta) < tolA) break;
    

    /*
    if (abs(dist) < tolD) {
      dist = 0;
    }

    if (abs(theta) < tolA) {
      theta = 0;
    }*/


    // testing this out (remove and uncomment code above if doesnt work)
    if (abs(theta) < 2 || thetaTol) {
      theta = 0;
      thetaTol = true;
    }
    
    double rotational;
    double axial;

    rotational = PIDActivator->gyroPid(theta, capA);
    axial = PIDActivator->axial(((dist/(3.25*3.1415926535)) * 360.0), capD);
    
    //Brain.Screen.print("axial: %f, rotational: %f", axial, rotational);
    //Brain.Screen.newLine();
    odom::standardDrive(axial, rotational*turnScale);

    wait(10, msec);
  }

  if(PIDActivator) {
    delete PIDActivator;
    PIDActivator = 0;
  }
  
}

void odom::turnToPoint(double target_x, double target_y, double cap, double settleTime) {
  double start = Brain.timer(sec); 
  
  frontRight.setStopping(hold);
  frontLeft.setStopping(hold);
  middleRight.setStopping(hold);
  middleLeft.setStopping(hold);
  backLeft.setStopping(hold);
  backRight.setStopping(hold);

  double y_error = target_y - odom::y; 
  double x_error = target_x -  odom::x; 
  
  double target = ((atan2(x_error,y_error) * 180.0)/3.1415926535); 
  target += 90.0;
  if (target < 0) {
    target = 360.0 + target;
  }
    
  double theta = correctAngle(target - odom::a);
  double thetaRad = theta/180.0 * 3.1415926535;  
  pid* PIDActivator = new pid(); 
  
  while (abs(theta) > 2 && (Brain.timer(sec) < start + settleTime)) { 
    double rotational = PIDActivator->gyroPid(theta, cap);
    target = ((atan2(x_error,y_error) * 180.0)/3.1415926535);
    
    target += 90.0;
    if (target < 0) {
      target = 360.0 + target;
    }
    
    theta = correctAngle(target - odom::a);

    thetaRad = theta/180.0 * 3.1415926535;

    if (abs(theta) < 2) break;
    
    odom::standardDrive(0, rotational);

    wait(10, msec);
  }
  
  frontRight.stop();
  frontLeft.stop();
  middleRight.stop();
  middleLeft.stop();
  backLeft.stop();
  backRight.stop();

  if(PIDActivator) {
    delete PIDActivator;
    PIDActivator = 0;
  }
}