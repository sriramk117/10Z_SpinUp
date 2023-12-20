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

double LT = 0; 
double RT = 0; 
double BT = 0; 
double theta0 = 0;
double theta1 = 0;
double dTheta = theta1 - theta0;

double odom::x = 36; // up for modification
double odom::y = 12;  // up for modification
double odom::a = INIT_GYRO_HEADING;

bool odom::active = true;

odom* pOdom = 0;

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


  TL = 5.5;
  TR = 5.5;
  TB = 5.25; 

  theta1 = 3.1415926535/2;

  Gyro.setHeading(INIT_GYRO_HEADING, degrees);
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

void odom::TrackPosition() {
   while(odom::active) { 
    DL = (double)(frontLeft.position(degrees) + middleLeft.position(degrees) + backLeft.position(degrees))/3.0;
    DR = (double)(frontRight.position(degrees) + middleRight.position(degrees) + backRight.position(degrees))/3.0;
    
    DL *= (48.0/72.0);
    DR *= (48.0/72.0);

    DL /= 360.0; DL *= 3.25*3.1415926535;
    DR /= 360.0; DR *= 3.25*3.1415926535;
    
    z = (DL + DR)/2;

    theta1 = (Gyro.heading(degrees) * 3.1415926535) / 180.0;
    dx = -z*cos(theta1);
    dy = z*sin(theta1);

    odom::x += dx;
    odom::y += dy;
    odom::a = Gyro.heading(degrees);
     
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
  double a_error =  correctAngle(desiredHeading - (Gyro.heading(degrees))); 
  while(abs(a_error) > 2) {
    double rotational = PIDActivator->gyroPid(a_error, cap);
    a_error = correctAngle(desiredHeading - (Gyro.heading(degrees)));
    odom::rotStandardDrive(0, rotational);
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
  double a_error =  desiredHeading - sub;
  while(fabs(a_error) > 2) {
    double rotational = PIDActivator->gyroPid(a_error, cap);
    if (Gyro.heading(degrees) <= 180) {
      sub = Gyro.heading(degrees) + 360;
    } else {
      sub = Gyro.heading(degrees);
    }
    a_error =  desiredHeading - sub;
    odom::standardDrive(0, rotational);
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
  
  // Global PID
  double x_error = x - odom::x;
  double y_error = y - odom::y;
  double dist = sqrt(x_error*x_error + y_error*y_error);
  double axial = PIDActivator->axial(((dist/(3.25*3.1415926535)) * 360.0), cap);
  

  while(abs(dist) > 2) {
    axial = PIDActivator->axial(((dist/(3.25*3.1415926535)) * 360.0), cap);
    
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
  bool vel = true;
  double x, y;

  if (d < 0) {
    vel = false;
  }
  x = odom::x + d * cos(odom::a * (3.14159268/180));
  y = odom::y + d * sin(odom::a * (3.14159268/180));

  double dist = sqrt(x*x + y*y) - sqrt(odom::x * odom::x + odom::y * odom::y);
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
    if (abs(distTarget) < 15) { //15
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

    wait(2, msec); 
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

  double thetaRad = theta/180.0 * 3.1415926535;
  double dist = sqrt(y_error*y_error + x_error*x_error);

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

    if (abs(theta) < 2 || thetaTol) {
      theta = 0;
      thetaTol = true;
    }
    
    double rotational;
    double axial;

    rotational = PIDActivator->gyroPid(theta, capA);
    axial = PIDActivator->axial(((dist/(3.25*3.1415926535)) * 360.0), capD);

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
